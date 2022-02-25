#! /home/whitealex95/anaconda3/bin/python
import rospy
import std_msgs.msg
from functools import partial

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.label import Label

from kivy.uix.slider import Slider
from kivy.graphics import Color, Bezier, Line, Rectangle, Ellipse

from kivy.core.window import Window
from kivy.utils import get_color_from_hex

# custom msg
from towr_gui_kivy.msg import TowrCommand2
# Clear entire color to whitish./
Window.clearcolor = get_color_from_hex('#f0f9f9')

POINT_SIZE = 10
# SET_UNACTIVE_COLOR = lambda : Color(0, 1, 0)
# SET_ACTIVE_COLOR = lambda : Color(1, 0, 0)
SET_COLOR = lambda : Color(1, 0, 0)


class WayPoint:
    def __init__(self, x, y, size=(10,10), vx=0, vy=0):
        print("Creating New Waypoint")
        self.x = x
        self.y = y
        self.size = size
        self.active = False
        # self.disp = Rectangle(pos=(self.x,self.y), size=self.size)
        self.disp = Ellipse(pos=self.disp_center, size=self.size)
    
    @property
    def disp_center(self):
        return (self.x-self.size[0]/2, self.y-self.size[1]/2)

    def mouse_down(self, touch_x, touch_y):
        self.active = True
        self.touch_x_prev = touch_x
        self.touch_y_prev = touch_y

    def mouse_drag(self, touch_x, touch_y):
        self.x += touch_x - self.touch_x_prev
        self.y += touch_y - self.touch_y_prev
        self.touch_x_prev = touch_x
        self.touch_y_prev = touch_y

        self.disp.pos= (self.x - self.size[0], self.y - self.size[1])
        self.disp.size = (self.size[0] * 2, self.size[1] * 2)

    def mouse_up(self):
        self.touch_x_prev = None
        self.touch_y_prev = None
        
        self.active = False

        self.disp.pos = self.disp_center
        self.disp.size = self.size

class TestApp(FloatLayout):

    def __init__(self, points, *args, **kwargs):
        super(TestApp, self).__init__(*args, **kwargs)
        self.d = 10  # pixel tolerance when clicking on a point
        self.waypoints = []
        self.current_waypoint = None
        Color(1.0, 1.0, 1.0)
        with self.canvas:
            SET_COLOR()
            line_points = []
            for point in points:
                self.waypoints.append(WayPoint(point[0], point[1]))
                line_points.extend([point[0], point[1]])

            Color(1.0, 0.0, 1.0)
            self.line = Line(
                    points=line_points,
                    dash_offset=10,
                    dash_length=100)


        s = Slider(y=50, pos_hint={'x': .3}, size_hint=(.7, None), height=50)
        s.bind(value=self._set_line_dash_offset)
        self.add_widget(s)

    def _set_line_dash_offset(self, instance, value):
        # effect to reduce length while increase offset
        self.line.dash_length = 100 - value
        self.line.dash_offset = value

    def on_touch_down(self, touch):
        if self.collide_point(touch.pos[0], touch.pos[1]):
            # Check if any waypoint is clicked. Selection based on waypoint order
            for waypoint in self.waypoints:
                p = (waypoint.x, waypoint.y)
                if (abs(touch.pos[0] - self.pos[0] - p[0]) < self.d and
                        abs(touch.pos[1] - self.pos[1] - p[1]) < self.d):
                    self.current_waypoint = waypoint
                    self.current_waypoint.mouse_down(touch.pos[0], touch.pos[1])
                    return True

            # Create new waypoint if clicked on empty space
            with self.canvas:
                SET_COLOR()
                self.current_waypoint = WayPoint(touch.pos[0], touch.pos[1]) 
                self.current_waypoint.mouse_down(touch.pos[0], touch.pos[1])
                self.waypoints.append(self.current_waypoint)

                line_points = []
                for waypoint in self.waypoints:
                    line_points.extend([waypoint.x, waypoint.y])
                self.line.points=line_points

                return True
            # return super(TestApp, self).on_touch_down(touch)
        else:
            print("Impossible")
            

    def on_touch_up(self, touch):
        if self.collide_point(touch.pos[0], touch.pos[1]):
            if self.current_waypoint:
                with self.canvas:
                    self.current_waypoint.mouse_up()
                self.current_waypoint = None
                return True
            return super(TestApp, self).on_touch_up(touch)

    def on_touch_move(self, touch):
        if self.collide_point(touch.pos[0], touch.pos[1]):
            if self.current_waypoint:
                with self.canvas:
                    self.current_waypoint.mouse_drag(touch.pos[0], touch.pos[1])

                line_points = []
                for waypoint in self.waypoints:
                    line_points.extend([waypoint.x, waypoint.y])
                self.line.points=line_points

                return True
            return super(TestApp, self).on_touch_move(touch)

class PublishManager:
    def __init__(self):
        msg = TowrCommand2()
        msg.goal_lin.pos.x = 1.3
        msg.goal_lin2.pos.x = 2.6

        msg.total_duration = 3          # reach in 3sec
        msg.replay_trajectory = False
        msg.replay_speed = 1            # x1.00 speed
        msg.play_initialization = False
        msg.plot_trajectory = False
        msg.optimize = True             # optimize trajectory
        msg.robot = 0                   # 0: monoped, 1: biped, 4: Aliengo
        msg.terrain = 0
        msg.gait = 0                    # 0: walk, 1: trot
        msg.gait_n = 6                  # default: 6 gait blocks prob?
        msg.optimize_phase_durations = False

        self.msg = msg

    def change_and_publish(self, **kwargs):
        for key, val in kwargs.items():
            print(key, val)
            if key=='robot':
                self.msg.robot = val
            elif key=='terrain':
                self.msg.terrain = val

        pub.publish(self.msg)

class SubscribeManager:
    def __init__(self) -> None:
        pass

class Main(App):
    def btn_callback(self, *largs, **kwargs):
        # use it with partial()
        print(kwargs)
        pm.change_and_publish(**kwargs)

    def build(self):
        from math import cos, sin, radians
        x = y = 150
        z = 100
        # Pacman !
        points = [[x, y]]
        for i in range(45, 360, 45):
            i = radians(i)
            points.append([x + cos(i) * z, y + sin(i) * z])

        
        # Setting up Layouts
        canvas = TestApp(points=points)

        label = Label(text='0')
        btn_robot1 = Button(text='TOPIC1')
        btn_robot1.bind(on_press=partial(self.btn_callback, robot=0))

        btn_robot2 = Button(text='TOPIC2')
        btn_robot2.bind(on_press=partial(self.btn_callback, robot=1))

        btn_robot3 = Button(text='TOPIC3')
        btn_robot3.bind(on_press=partial(self.btn_callback, robot=4))

        layout = BoxLayout(size_hint=(1, None), height=50)
        layout.add_widget(btn_robot1)
        layout.add_widget(btn_robot2)
        layout.add_widget(btn_robot3)
        layout.add_widget(label)

        root = BoxLayout(orientation='vertical')
        rfl = FloatLayout()
        rfl.add_widget(canvas)
        root.add_widget(rfl)
        root.add_widget(layout)

        return root

def sub_callback(msg):
    import os
    raw_cmd = msg.data.split(' ')  # becareful, rospy doesn't output valid error...
    path = '~/.ros/' + raw_cmd[-1]
    fixed_command = ' '.join(raw_cmd[:-1] + [path])
    print("callback: ",fixed_command)
    os.system(fixed_command)

if __name__ == '__main__':

    pub = rospy.Publisher('/towr/user_command2', TowrCommand2, queue_size=1)
    rospy.Subscriber("/towr/ros_vis_traj", std_msgs.msg.String, sub_callback)
    rospy.init_node('ros_gui_kivy', anonymous=False)
    pm = PublishManager()
    # sm = SubscribeManager()
    Main().run()
