"""Camera-to-paper maze prototype. NEVER issues commands to physical actuators."""
import argparse
import json
from collections import deque
from pathlib import Path

import cv2
import numpy as np


def perspective_board(frame, corners, width=600, height=600):
    if len(corners) != 4:
        raise ValueError('Four ordered corners are required (TL TR BR BL).')
    src = np.asarray(corners, dtype=np.float32)
    dst = np.asarray([(0, 0), (width-1, 0), (width-1, height-1), (0, height-1)], dtype=np.float32)
    if abs(cv2.contourArea(src)) < 100:
        raise ValueError('Selected board corners are nearly collinear or too small.')
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(frame, matrix, (width, height)), matrix


def candidate_walkable(board, invert=False, threshold=160, clearance=2):
    if board.ndim == 3:
        gray = cv2.cvtColor(board, cv2.COLOR_BGR2GRAY)
    else:
        gray = board
    gray = cv2.GaussianBlur(gray, (3,3), 0)
    if invert:
        binary = (gray < threshold).astype(np.uint8)
    else:
        binary = (gray > threshold).astype(np.uint8)
    if clearance:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*clearance+1,2*clearance+1))
        binary = cv2.erode(binary, kernel, iterations=1)
    return binary


def snap_point(walkable, pt, max_radius=28):
    h, w = walkable.shape
    x,y = [int(round(v)) for v in pt]
    if not (0 <= x < w and 0 <= y < h):
        raise ValueError('Start/goal outside cropped maze.')
    if walkable[y,x]: return (x,y)
    for radius in range(1, max_radius+1):
        for dy in range(-radius,radius+1):
            for dx in (-radius,radius):
                xx, yy = x+dx, y+dy
                if 0 <= xx < w and 0 <= yy < h and walkable[yy,xx]: return (xx,yy)
        for dx in range(-radius+1,radius):
            for dy in (-radius,radius):
                xx,yy=x+dx,y+dy
                if 0 <= xx < w and 0 <= yy < h and walkable[yy,xx]: return (xx,yy)
    raise ValueError('Start or finish not near a white corridor. Check maze contrast or clearance.')


def solve(board, start, goal, *, invert=False, threshold=160, clearance=2):
    walk = candidate_walkable(board, invert=invert, threshold=threshold, clearance=clearance)
    s, g = snap_point(walk, start), snap_point(walk, goal)
    h,w=walk.shape
    parent = np.full((h,w), -1, dtype=np.int32)
    queue = deque([(s[0],s[1])])
    parent[s[1],s[0]] = s[1]*w+s[0]
    while queue:
        x,y = queue.popleft()
        if (x,y)==g: break
        for nx,ny in ((x+1,y),(x,y+1),(x-1,y),(x,y-1)):
            if 0<=nx<w and 0<=ny<h and walk[ny,nx] and parent[ny,nx]<0:
                parent[ny,nx] = y*w+x
                queue.append((nx,ny))
    if parent[g[1],g[0]]<0:
        raise ValueError('No traversable route. Adjust threshold / invert / corners / start & finish.')
    dense=[]; current = g[1]*w+g[0]; start_idx=s[1]*w+s[0]
    while True:
        yy,xx=divmod(int(current),w);dense.append((xx,yy))
        if current==start_idx:break
        current=int(parent[yy,xx])
    dense.reverse()
    # Preserve every direction change to avoid cutting through walls.
    waypoints=[dense[0]]
    for i in range(1,len(dense)-1):
        ax,ay=dense[i-1];bx,by=dense[i];cx,cy=dense[i+1]
        if (bx-ax,by-ay)!=(cx-bx,cy-by) or i%10==0:
            waypoints.append(dense[i])
    if dense[-1]!=waypoints[-1]:waypoints.append(dense[-1])
    return waypoints,walk


def render(board, path, walkable=None):
    out=cv2.cvtColor(board, cv2.COLOR_GRAY2BGR) if board.ndim==2 else board.copy()
    for a,b in zip(path,path[1:]):cv2.line(out,tuple(a),tuple(b),(0,0,255),2)
    cv2.circle(out,path[0],7,(0,200,0),-1)
    cv2.circle(out,path[-1],7,(255,0,0),-1)
    return out


def trajectory(path, image_shape):
    """Normalized [0,1] path only; NOT robot-space or safe motor commands."""
    h,w=image_shape[:2]
    coords=[[round(x/max(1,w-1),6),round(y/max(1,h-1),6)] for x,y in path]
    return {
        'reference_frame':'rectified_maze_image',
        'unit':'fraction_of_image_width_height',
        'robot_coordinates_calibrated':False,
        'ready_for_execution':False,
        'pen_sequence':[
            {'action':'move_above_start','xy':coords[0],'pen':'up'},
            {'action':'lower_pen_at_start','xy':coords[0],'pen':'down'},
            *[{'action':'trace','xy':p,'pen':'down'} for p in coords[1:]],
            {'action':'raise_pen_at_finish','xy':coords[-1],'pen':'up'}
        ]
    }


def make_sample(seed=7,cells=7,scale=24):
    rng=np.random.default_rng(seed)
    walls=np.ones((2*cells+1,2*cells+1),np.uint8)
    stack=[(0,0)];visited={(0,0)};walls[1,1]=0
    while stack:
        r,c=stack[-1]
        neighbors=[(r+dr,c+dc) for dr,dc in ((1,0),(-1,0),(0,1),(0,-1))
                   if 0<=r+dr<cells and 0<=c+dc<cells and (r+dr,c+dc) not in visited]
        if not neighbors:stack.pop();continue
        rr,cc=neighbors[int(rng.integers(len(neighbors)))]; walls[r+rr+1,c+cc+1]=0
        walls[2*rr+1,2*cc+1]=0;visited.add((rr,cc));stack.append((rr,cc))
    img=np.where(walls==0,255,0).astype(np.uint8)
    img=cv2.resize(img,None,fx=scale,fy=scale,interpolation=cv2.INTER_NEAREST)
    return img,(int(1.5*scale),int(1.5*scale)),(int((2*cells-.5)*scale),int((2*cells-.5)*scale))


def point_selection(window, frame, instructions, n):
    points=[]
    def cb(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and len(points)<n:points.append((x,y))
    cv2.namedWindow(window,cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window,cb)
    while True:
        display=frame.copy()
        if display.ndim==2:display=cv2.cvtColor(display,cv2.COLOR_GRAY2BGR)
        for i,p in enumerate(points):
            cv2.circle(display,p,6,(0,0,255),-1)
            cv2.putText(display,str(i+1),(p[0]+7,p[1]-7),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
        cv2.putText(display,instructions,(12,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(20,0,255),2)
        cv2.imshow(window,display)
        key=cv2.waitKey(30)&0xff
        if key==ord('q') or key==27:raise KeyboardInterrupt('Selection canceled')
        if key==ord('r'):points.clear()
        if key==32 and len(points)==n:return points


def capture(camera_index):
    cap=cv2.VideoCapture(camera_index)
    if not cap.isOpened():raise RuntimeError(f'Cannot open webcam {camera_index}')
    try:
        cv2.namedWindow('Maze camera',cv2.WINDOW_NORMAL)
        while True:
            ok,frame=cap.read()
            if not ok:raise RuntimeError('Webcam frame unavailable')
            preview=frame.copy()
            cv2.putText(preview,'SPACE: capture | Q: quit',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
            cv2.imshow('Maze camera',preview)
            key=cv2.waitKey(30)&0xff
            if key==32:return frame.copy()
            if key==ord('q') or key==27:raise KeyboardInterrupt('Capture canceled')
    finally:cap.release();cv2.destroyWindow('Maze camera')


def main(argv=None):
    p=argparse.ArgumentParser(description='Webcam paper maze -> path and pen trajectory preview; MOCK ONLY')
    p.add_argument('--source',choices=['camera','image','demo'],default='camera')
    p.add_argument('--camera-index',type=int,default=0)
    p.add_argument('--image')
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--start',type=int,nargs=2)
    p.add_argument('--goal',type=int,nargs=2)
    p.add_argument('--invert',action='store_true',help='Use dark corridors instead of white')
    p.add_argument('--threshold',type=int,default=160)
    p.add_argument('--clearance',type=int,default=2)
    p.add_argument('--output-dir',default='out')
    p.add_argument('--robot-mode',choices=['mock','real'],default='mock')
    args=p.parse_args(argv)
    if args.robot_mode!='mock':raise PermissionError('Physical motion disabled: camera-to-robot calibration and safety are NOT commissioned')
    if args.source=='demo':
        board,start,goal=make_sample(seed=args.seed)
    elif args.source=='image':
        if not args.image: p.error('--image is required with --source image')
        board=cv2.imread(args.image)
        if board is None:raise ValueError(f'Cannot open image: {args.image}')
        if args.start is None or args.goal is None:
            start,goal=point_selection('Pick start then finish',board,'Click START, FINISH then SPACE; R reset; Q quit',2)
        else:start,goal=tuple(args.start),tuple(args.goal)
    else:
        frame=capture(args.camera_index)
        corners=point_selection('Select paper maze corners',frame,'Click TL TR BR BL; SPACE accept; R reset',4)
        cv2.destroyWindow('Select paper maze corners')
        board,_=perspective_board(frame,corners)
        start,goal=point_selection('Select maze endpoints',board,'Click START then FINISH; SPACE accept; R reset',2)
        cv2.destroyWindow('Select maze endpoints')
    out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(out/'paper_maze_rectified.png'),board)
    path,walk=solve(board,start,goal,invert=args.invert,threshold=args.threshold,clearance=args.clearance)
    cv2.imwrite(str(out/'paper_maze_walkable.png'),walk*255)
    cv2.imwrite(str(out/'paper_maze_solved.png'),render(board,path))
    plan=trajectory(path,board.shape)
    with open(out/'paper_maze_pen_plan.json','w',encoding='utf-8') as fp:json.dump(plan,fp,indent=2)
    print(f'[MOCK] Camera/image maze solved: {len(path)} waypoints, start={path[0]}, finish={path[-1]}')
    print('[MOCK] Pen up -> approach -> pen down -> trace -> pen up -> home (simulation only)')
    print('Saved:',out/'paper_maze_solved.png',out/'paper_maze_pen_plan.json')
    print('REAL ARM NOT MOVED. No gripper movement performed.')
    if args.source!='demo':
        cv2.imshow('Solved maze / RED trajectory',render(board,path))
        print('Press any key in preview window to close.')
        cv2.waitKey(0);cv2.destroyAllWindows()

if __name__=='__main__':main()
