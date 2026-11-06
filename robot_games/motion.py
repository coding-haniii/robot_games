"""No-actuation mock, image -> calibrated robot-plane plan, and guarded ROS driver."""
import numpy as np

class MockRobot:
    def __init__(self): self.events=[]
    def draw(self, path, label='path'): self.events.append(('draw',label,len(path))); print(f'[MOCK] drawing {label}: {len(path)} points')
    def home(self): self.events.append(('home',)); print('[MOCK] home')
    def celebrate(self): self.events.append(('celebrate',)); print('[MOCK] straight-arm celebration (symbolic)')

def image_path_to_base(path, shape, config):
    """Calibrated XY plane: origin at image top-left, axes span image width and height.
    Requires onsite measured orthogonal/nonorthogonal vectors and absolute Z.
    """
    required=['board_origin_base_mm','board_x_axis_base_mm','board_y_axis_base_mm',
              'drawing_z_base_mm','safe_z_base_mm','tool_rpy_deg']
    if any(config.get(k) is None for k in required): raise ValueError('Robot board/TCP calibration incomplete')
    h,w=shape[:2]
    origin=np.asarray(config['board_origin_base_mm'],float)
    xaxis=np.asarray(config['board_x_axis_base_mm'],float)
    yaxis=np.asarray(config['board_y_axis_base_mm'],float)
    if origin.size!=3 or xaxis.size!=3 or yaxis.size!=3: raise ValueError('Axes/origin must be 3D')
    if np.linalg.norm(xaxis)>config.get('max_board_side_mm',300) or np.linalg.norm(yaxis)>config.get('max_board_side_mm',300):
        raise ValueError('Board too large for configured limit')
    if len(path)>config.get('max_path_points',1500): raise ValueError('Path point count exceeds limit')
    z=float(config['drawing_z_base_mm']); safe=float(config['safe_z_base_mm'])
    if safe <= z: raise ValueError('Safe Z must be greater than drawing Z for horizontal top-down setup')
    orientation=list(map(float,config['tool_rpy_deg']))
    output=[]
    for px,py in path:
        p=origin+(float(px)/max(1,w-1))*xaxis+(float(py)/max(1,h-1))*yaxis
        output.append([float(p[0]),float(p[1]),z,*orientation])
    return output

class GuardedDoosan:
    """Only runs motion when armed AND calibration confirmed; requires sourced Jazzy DSR_ROBOT2."""
    def __init__(self, config):
        if not (config.get('armed') and config.get('calibration_confirmed')):
            raise PermissionError('Robot not armed and calibrated')
        if config.get('home_joints_deg') is None or config.get('celebration_joints_deg') is None:
            raise ValueError('Home/celebration must be taught and validated on real arm')
        self.cfg=config
        import rclpy
        import DR_init
        self.rclpy=rclpy
        rclpy.init()
        DR_init.__dsr__id=config.get('namespace','dsr01')
        DR_init.__dsr__model=config.get('model','e0509')
        self.node=rclpy.create_node('games_operator',namespace=DR_init.__dsr__id)
        DR_init.__dsr__node=self.node
        from DSR_ROBOT2 import movej,movel,posj,posx
        self.movej,self.movel,self.posj,self.posx=movej,movel,posj,posx
    def _joint(self, vals):
        if len(vals)!=6: raise ValueError('Six joint angles required')
        self.movej(self.posj(*vals),vel=self.cfg['joint_speed_deg_s'],acc=self.cfg['joint_accel_deg_s2'])
    def home(self): self._joint(self.cfg['home_joints_deg'])
    def celebrate(self): self._joint(self.cfg['celebration_joints_deg'])
    def draw(self, path, label='path'):
        """Only path positions in BASE frame with verified TCP/tool settings; low speed.
        No ROS startup or emergency stop logic is bypassed. Execute only with operator approval.
        """
        if not path: return
        safe=float(self.cfg['safe_z_base_mm']); vel=float(self.cfg['speed_mm_s']); acc=float(self.cfg['accel_mm_s2'])
        if not (0<vel<=20 and 0<acc<=50): raise ValueError('Restrict speed/acceleration during commissioning')
        first=path[0].copy(); first[2]=safe
        self.movel(self.posx(*first),vel=vel,acc=acc)
        for point in path:
            self.movel(self.posx(*point),vel=vel,acc=acc)
        last=path[-1].copy(); last[2]=safe
        self.movel(self.posx(*last),vel=vel,acc=acc)
    def close(self): self.node.destroy_node(); self.rclpy.shutdown()
