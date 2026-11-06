from pathlib import Path
import cv2
import numpy as np
from .maze import generate_maze
from .tictactoe import render_board

def make(output):
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    for number,label in [('1','TICTACTOE'),('2','MAZE'),('3','EXIT')]:
        img=np.full((400,700,3),255,np.uint8)
        cv2.rectangle(img,(10,10),(690,390),(0,0,0),4)
        cv2.putText(img,number,(290,190),cv2.FONT_HERSHEY_SIMPLEX,5,(0,0,0),12)
        cv2.putText(img,label,(120,310),cv2.FONT_HERSHEY_SIMPLEX,1.6,(0,0,0),4)
        cv2.imwrite(str(out/f'card_{number}.png'),img)
    cv2.imwrite(str(out/'tic_board.png'),render_board([[' ',' ',' '],[' ','X',' '],[' ',' ',' ']]))
    for seed in (7,11,25):
        img,_,_=generate_maze(seed)
        cv2.imwrite(str(out/f'maze_{seed}.png'),img)
    print('Generated samples:',out)
if __name__=='__main__':make(Path(__file__).resolve().parents[1]/'assets')
