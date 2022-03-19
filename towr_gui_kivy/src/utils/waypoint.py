
from re import I
from kivy.graphics import Color, Bezier, Line, Rectangle, Ellipse

SET_COLOR = lambda : Color(0, 0, 0)
SET_POINT_COLOR = lambda : Color(0, 0, 0)
SET_INIT_POINT_COLOR = lambda : Color(1, 0, 0)
SET_LINE_COLOR = lambda : Color(0, 0, 1)


s = [50, 50, 1]
t = [250, 275, 0]

def canvas2world(canvas_pos):
    print(canvas_pos)
    return [(d - t[j]) / s[j] for j, d in enumerate(canvas_pos)]
def world2canvas(world_pos):
    print(world_pos)
    return [s[j] * d + t[j] for j, d in enumerate(world_pos)]

class Point:
    def __init__(self, x, y, coord_type='w', size=(15,15),
                c_scale=None, c_origin=None):
        self.c_scale = s
        self.c_origin = t
        if coord_type =='w':
            self.set_world_pos(x,y)
        elif coord_type == 'c':
            self.set_canvas_pos(x,y)
        else:
            __import__('pdb').set_trace()
        
        self.size = size
        self.disp_obj = Ellipse(pos=self.disp_pos, size=self.size)

    @property
    def disp_pos(self):
        # position of Ellipse is left bottom, not its center.
        return (self.c_x-self.size[0]/2, self.c_y-self.size[1]/2)

    @property
    def world_pos(self):
        return [self.w_x, self.w_y]
    
    @property
    def canvas_pos(self):
        return [self.c_x, self.c_y]

    def set_world_pos(self, w_x, w_y):
        self.w_x = w_x
        self.w_y = w_y
        self.c_x = self.c_scale[0] * w_x + self.c_origin[0]
        self.c_y = self.c_scale[1] * w_y + self.c_origin[1]

    def set_canvas_pos(self, c_x, c_y):
        self.c_x = c_x
        self.c_y = c_y
        self.w_x = (c_x - self.c_origin[0])/self.c_scale[0]
        self.w_y = (c_y + self.c_origin[1])/self.c_scale[1]

class WayPoint(Point):
    def __init__(self, x, y, size=(15,15), vx=0, vy=0 , roll=0, pitch=0, yaw=0,
                        coord_type='w'):
        print("Creating New Waypoint")
        super().__init__(x, y, coord_type=coord_type, size=size)
        self.w_z = 0 # ignore it
        self.vx = vx
        self.vy = vy
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw  # probabily just going to use yaw only....
        self.active = False

    @property
    def world_ang(self):
        return [self.roll, self.pitch, self.yaw]

    def mouse_down(self, touch_x, touch_y):
        self.touch_x_prev = touch_x
        self.touch_y_prev = touch_y

        self.active = True

        # enlargen shape
        self.disp_obj.pos= (self.c_x - self.size[0], self.c_y - self.size[1])
        self.disp_obj.size = (self.size[0] * 2, self.size[1] * 2)

    def mouse_drag(self, touch_x, touch_y):
        self.set_canvas_pos(
            self.c_x + touch_x - self.touch_x_prev,
            self.c_y + touch_y - self.touch_y_prev
        )
        self.touch_x_prev = touch_x
        self.touch_y_prev = touch_y

        self.disp_obj.pos= (self.c_x - self.size[0], self.c_y - self.size[1])
        print("dragging pos[canv]: ", self.c_x, self.c_y)
        print("dragging pos[world]: ", self.w_x, self.w_y)

    def mouse_up(self):
        self.touch_x_prev = None
        self.touch_y_prev = None
        
        self.active = False

        self.disp_obj.pos = self.disp_pos
        self.disp_obj.size = self.size

class WayPointManager():
    def __init__(self, waypoints_pos, waypoints_ang, canvas):
        self.n_wp = len(waypoints_pos)
        self.waypoints = []
        self.start_idx = 0
        self.active_idx = self.n_wp-1  # index of dragged or last-clicked
        self.clicked_wp = None
        self.clicked_idx = -1
        self.canvas = canvas
        
        self.lines = []
        self.initialize(waypoints_pos, waypoints_ang)

    def initialize(self, waypoints_pos, waypoints_ang):
        with self.canvas:
            line_points = []
            for i in range(self.n_wp):
                w_pos, ang = waypoints_pos[i], waypoints_ang[i]
                if i==0:
                    SET_INIT_POINT_COLOR()
                else:
                    SET_POINT_COLOR()
                new_wp = WayPoint(w_pos[0], w_pos[1], coord_type='w',
                                roll=ang[0], pitch=ang[1], yaw=ang[2])
                self.waypoints.append(new_wp)
                line_points.extend(new_wp.canvas_pos)

            SET_LINE_COLOR()
            self.line = Line(points=line_points,
                            width=2)
    def update_lines(self):
        line_points = []
        for wp in self.waypoints:
            line_points.extend([wp.canvas_pos])
        self.line.points = line_points

    def clear_waypoints(self):
        print("clearing objects")
        for wp in self.waypoints:
            self.canvas.remove(wp.disp_obj)
        self.waypoints.clear()
        self.line.points = []
        self.n_wp = 0

    def add_waypoint(self, pos, ang, coord_type='c'):
        with self.canvas:
            if self.n_wp==0:
                SET_INIT_POINT_COLOR()
            else:   
                SET_POINT_COLOR()
            new_wp = WayPoint(pos[0], pos[1], coord_type=coord_type,
                            roll=ang[0], pitch=ang[1], yaw=ang[2])
        
        self.waypoints.append(new_wp)
        self.n_wp += 1
        # todo: make use of active_idx some day
        self.active_idx += 1

        print("new pos[canv]: ", new_wp.c_x, new_wp.c_y)
        print("new pos[world]: ", new_wp.w_x, new_wp.w_y)
        self.update_lines()

        return new_wp

    def add_waypoint_c(self, c_pos, ang=[0,0,0], coord_type='c'):
        new_wp = self.add_waypoint(c_pos, ang, coord_type='c')
        new_wp.mouse_down(c_pos[0], c_pos[1])
        self.clicked_wp = new_wp
        # self.line.points = self.line.points.extend([new_wp.canvas_pos])

    def check_waypoint_clicked(self, touch_x, touch_y, d):
        for i, wp in enumerate(self.waypoints):
            if (abs(touch_x - wp.c_x) < d and
                    abs(touch_y - wp.c_y) < d):
                self.clicked_idx = i
                self.clicked_wp = wp
                self.clicked_wp.mouse_down(touch_x, touch_y)
                return True
        return False