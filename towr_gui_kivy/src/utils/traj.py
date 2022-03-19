from cmath import pi
import math
from utils.waypoint import Point, WayPoint
from utils.waypoint import world2canvas
from kivy.graphics import Color, Bezier, Line, Rectangle, Ellipse

SET_TRAJ_POINT_COLOR = lambda : Color(0, 0.5, 0)
# TRAJ_STEP_SIZE = 1 * 2/3
TRAJ_STEP_SIZE = 1 * 1
class Trajectory():
  def __init__(self, current_pos_ang, n_future) -> None:
    # trajectory starts from current goal(waypoint)
    self.current_pos_ang = current_pos_ang
    self.future_pos_angs = None 
    self.n_future = n_future # fix number of futre trajectory to predict
    
    self.init_trajectory()

  def init_trajectory(self):
    self.future_pos_angs = []
    c_x, c_y, c_theta = self.current_pos_ang
    for i in range(self.n_future):
      i_x = c_x + TRAJ_STEP_SIZE * (i+1) * math.cos(c_theta)
      i_y = c_y + TRAJ_STEP_SIZE * (i+1) * math.sin(c_theta)
      i_theta = c_theta
      self.future_pos_angs.append([i_x, i_y, i_theta])
  
  def update_trajectory(self):
    pass


class TrajectoryManager():
  def __init__(self, current_waypoint, canvas) -> None:
    self.current_waypoint = current_waypoint
    self.canvas = canvas

    self.goal_size = (8, 8)
    self.n_future = 4 # fix number of futre trajectory to predict

    current_pos_ang = [current_waypoint.w_x, current_waypoint.w_y,
                        current_waypoint.yaw]
    self.traj = Trajectory(current_pos_ang, self.n_future)

    self.trajpoints = []
    self.init_trajpoints()

  def init_trajpoints(self):
    with self.canvas:
      for i in range(self.n_future):
        w_x, w_y, _ = self.traj.future_pos_angs[i]
        SET_TRAJ_POINT_COLOR()
        new_trajpoint = Point(w_x, w_y, size=self.goal_size)
        self.trajpoints.append(new_trajpoint)
  
  def update_trajpoints(self):
    for i in range(self.n_future):
      w_x, w_y, _ = self.traj.future_pos_angs[i]
      self.trajpoints[i].set_world_pos(w_x, w_y)
      # update ellipse position
      # __import__('pdb').set_trace()
      self.trajpoints[i].disp_obj.pos = self.trajpoints[i].disp_pos

  def update_traj(self, text):
    # we cannot do trajectory blend as in PFNN/MANN
    # Using Heuristic update rule
    x_prev, y_prev, theta_prev = self.traj.current_pos_ang
    new_x_prev, new_y_prev, new_theta_prev = x_prev, y_prev, theta_prev
    if text == 'a':
      for i in range(self.n_future):
        x, y, theta = self.traj.future_pos_angs[i]
        # if theta - theta_prev < 1e-8:
        new_theta = new_theta_prev + (theta-theta_prev) + pi/24
        # else:
        #   new_theta = new_theta_prev + (theta-theta_prev) * 1.2
        dx = x- x_prev
        dy = y - y_prev
        d = math.sqrt(dx*dx + dy*dy)
        new_x = new_x_prev + d * math.cos(new_theta)
        new_y = new_y_prev + d * math.sin(new_theta)
        self.traj.future_pos_angs[i] = [new_x, new_y, new_theta]
        x_prev, y_prev, theta_prev = x, y, theta
        new_x_prev, new_y_prev, new_theta_prev = new_x, new_y, new_theta
    
    elif text == 'd':
      for i in range(self.n_future):
        x, y, theta = self.traj.future_pos_angs[i]
        # if theta - theta_prev > -1e-8:
        new_theta = new_theta_prev + (theta-theta_prev) - pi/24
        # else:
        #   new_theta = new_theta_prev + (theta-theta_prev) * 1.2
        dx = x- x_prev
        dy = y - y_prev
        d = math.sqrt(dx*dx + dy*dy)
        new_x = new_x_prev + d * math.cos(new_theta)
        new_y = new_y_prev + d * math.sin(new_theta)
        self.traj.future_pos_angs[i] = [new_x, new_y, new_theta]
        x_prev, y_prev, theta_prev = x, y, theta
        new_x_prev, new_y_prev, new_theta_prev = new_x, new_y, new_theta

    self.update_trajpoints()   

  def update_current_waypoint(self, current_waypoint):
    self.current_waypoint = current_waypoint
    current_pos_ang = [current_waypoint.w_x, current_waypoint.w_y,
                    current_waypoint.yaw]

    self.traj.current_pos_ang = current_pos_ang
    # print(self.traj.future_pos_angs)
    self.traj.init_trajectory()
    # print(self.traj.future_pos_angs)
    self.update_trajpoints()

