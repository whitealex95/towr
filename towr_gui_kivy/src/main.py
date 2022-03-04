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
from kivy.uix.textinput import TextInput

from kivy.uix.slider import Slider
from kivy.graphics import Color, Bezier, Line, Rectangle, Ellipse

from kivy.core.window import Window
from kivy.utils import get_color_from_hex

# custom msg
from towr_gui_kivy.msg import TowrCommand2, TowrCommandSeq
from xpp_msgs.msg import StateLin3d
# Clear entire color to whitish./
Window.clearcolor = get_color_from_hex('#f0f9f9')

POINT_SIZE = 10
# SET_UNACTIVE_COLOR = lambda : Color(0, 1, 0)
# SET_ACTIVE_COLOR = lambda : Color(1, 0, 0)
SET_COLOR = lambda : Color(0, 0, 0)
SET_INIT_POINT_COLOR = lambda : Color(1, 0, 0)
SET_LINE_COLOR = lambda : Color(0, 0, 1)

BUTTON_COLOR = (129/255, 212/255, 250/255, 1)


class WayPoint:
    def __init__(self, x, y, size=(15,15), vx=0, vy=0 , roll=0, pitch=0, yaw=0):
        print("Creating New Waypoint")
        self.x = x
        self.y = y
        self.z = 0 # ignore it
        self.vx = vx
        self.vy = vy
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw  # probabily just going to use yaw only....
        self.size = size
        self.active = False
        # self.disp = Rectangle(pos=(self.x,self.y), size=self.size)
        self.disp = Ellipse(pos=self.disp_center, size=self.size)
    
    @property
    def disp_center(self):
        return (self.x-self.size[0]/2, self.y-self.size[1]/2)

    @property
    def world_pos(self):
        return canvas2world([self.x, self.y, self.z])
        
    @property
    def world_ang(self):
        return [self.roll, self.pitch, self.yaw]

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
        print("dragging: ", self.x, self.y)

    def mouse_up(self):
        self.touch_x_prev = None
        self.touch_y_prev = None
        
        self.active = False

        self.disp.pos = self.disp_center
        self.disp.size = self.size

class MouseManager():
    def __init__(self) -> None:
        # Todo: use mouse separate class for complicated operation
        pass

s = [50, 50, 1]
t = [250, 275, 0]
def world2canvas(world_pos):
    print(world_pos)
    return [s[j] * d + t[j] for j, d in enumerate(world_pos)]

def canvas2world(canvas_pos):
    print(canvas_pos)
    return [(d - t[j]) / s[j] for j, d in enumerate(canvas_pos)]

class TestApp(FloatLayout):

    def __init__(self, waypoints_pos, waypoints_ang, *args, **kwargs):
        super(TestApp, self).__init__(*args, **kwargs)
        self.d = 10  # pixel tolerance when clicking on a point
        initpoint_pos = [0, 0, 0]
        self.waypoints = []
        self.current_waypoint = None
        Color(1.0, 1.0, 1.0)
        with self.canvas:
            line_points = []
            SET_INIT_POINT_COLOR()
            point = world2canvas(initpoint_pos)
            self.initpoint = WayPoint(point[0], point[1])
            line_points.extend([point[0], point[1]])
            SET_COLOR()
            for pos, ang in list(zip(waypoints_pos, waypoints_ang)):
                point = world2canvas(pos)
                print("debug")
                print(point)
                self.waypoints.append(WayPoint(point[0], point[1], roll=ang[0], pitch=ang[1], yaw=ang[2]))
                line_points.extend([point[0], point[1]])

            SET_LINE_COLOR()
            self.line = Line(
                    points=line_points,
                    width = 2
                    ) #,

                    # dash_offset=10,
                    # dash_length=100)
            print("HIHIHIHIHI")
            print(self.center)
    
    def clear_waypoints(self):
        print("clearing objects")
        self.waypoints.clear()
        self.current_waypoint = None

        print("clearing canvas")
        self.canvas.clear()
        with self.canvas:
            SET_LINE_COLOR()
            self.line = Line(points=[],
                        dash_offset=10,
                        dash_length=100)

    def debug_coordinate(self):
        print(self.x, self.y)
        print(self.pos)
        print(self.center)

    def add_waypoint(self, pos:list, ang:list=[0,0,0]):
        with self.canvas:
            SET_COLOR()

            self.debug_coordinate()

            new_waypoint = WayPoint(x=pos[0], y=pos[1], roll=ang[0], pitch=ang[1], yaw=ang[2])
            self.waypoints.append(new_waypoint)

            line_points = []
            for waypoint in self.waypoints:
                line_points.extend([waypoint.x, waypoint.y])
            self.line.points=line_points
        self.current_waypoint = new_waypoint
        return new_waypoint


    def on_touch_down(self, touch):
        if self.collide_point(touch.pos[0], touch.pos[1]):
            # Check if any waypoint is clicked. Selection based on waypoint order
            for waypoint in self.waypoints:
                p = (waypoint.x, waypoint.y)
                if (abs(touch.pos[0] - p[0]) < self.d and
                        abs(touch.pos[1] - p[1]) < self.d):
                # Original Code:
                # if (abs(touch.pos[0] - self.pos[0] - p[0]) < self.d and
                #         abs(touch.pos[1] - self.pos[1] - p[1]) < self.d):
                    self.current_waypoint = waypoint
                    self.current_waypoint.mouse_down(touch.pos[0], touch.pos[1])
                    return True

            # Create new waypoint if clicked on empty space
            self.add_waypoint(pos=touch.pos)
            self.current_waypoint.mouse_down(touch.pos[0], touch.pos[1])
            return True
            # return super(TestApp, self).on_touch_down(touch)
        else:
            print("Out of Canvas")
            

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


class PublishManager_bk:
    def __init__(self):
        msg = TowrCommand2()
        msg.goal_lin.pos.x = 1.3
        msg.goal_lin2.pos.x = 2.6

        # msg.total_duration = 3          # reach in 3sec
        msg.total_duration = 4          # reach in 3sec
        msg.replay_trajectory = False
        msg.replay_speed = 1            # x1.00 speed
        msg.play_initialization = False
        msg.plot_trajectory = False
        msg.optimize = True             # optimize trajectory
        msg.robot = 0                   # 0: monoped, 1: biped, 4: Aliengo
        msg.terrain = 0
        msg.gait = 1                    # 0: walk, 1: trot
        # msg.gait_n = 8                  # default: 6 gait blocks prob?
        msg.gait_n = 12                  # default: 6 gait blocks prob?
        msg.optimize_phase_durations = False
        # msg.optimize_phase_durations = True

        self.msg = msg
    

    def change_and_publish(self, **kwargs):
        for key, val in kwargs.items():
            print(key, val)
            if key=='robot':
                self.msg.robot = val
            elif key=='terrain':
                self.msg.terrain = val

        pub.publish(self.msg)


class PublishManager:
    def __init__(self):
        msg = TowrCommandSeq()
        for j in range(3):
            goal_lin = StateLin3d()
            goal_lin.pos.x = (j+1) * 1.3
            msg.goal_lin_seq.append(goal_lin)

            goal_ang = StateLin3d()
            msg.goal_ang_seq.append(goal_ang)

        # msg.total_duration = 3          # reach in 3sec
        msg.total_duration = 3          # reach in 3sec
        msg.replay_trajectory = False
        msg.replay_speed = 1            # x1.00 speed
        msg.play_initialization = False
        msg.plot_trajectory = False
        msg.optimize = True             # optimize trajectory
        msg.robot = 0                   # 0: monoped, 1: biped, 4: Aliengo
        msg.terrain = 0
        msg.gait = 0                    # 0: walk, 1: trot
        msg.gait = 1                    # 0: walk, 1: trot
        msg.gait_n = 9                  # default: 6 gait blocks prob?
        # msg.gait_n = 9                  # default: 6 gait blocks prob?
        msg.optimize_phase_durations = False
        # msg.optimize_phase_durations = True

        self.msg = msg

    def set_waypoints(self, waypoints):
        self.msg.goal_lin_seq.clear()
        self.msg.goal_ang_seq.clear()

        for waypoint in waypoints:
            goal_lin = StateLin3d()
            [x,y,z] = waypoint.world_pos
            goal_lin.pos.x = x
            goal_lin.pos.y = y
            goal_lin.pos.z = z
            self.msg.goal_lin_seq.append(goal_lin)

            goal_ang = StateLin3d()
            [x, y, z] = waypoint.world_ang
            goal_ang.pos.x = x
            goal_ang.pos.y = y
            goal_ang.pos.z = z
            self.msg.goal_ang_seq.append(goal_ang)

    def change_and_publish(self, **kwargs):
        for key, val in kwargs.items():
            print(key, val)
            if key=='robot':
                self.msg.robot = val
            elif key=='terrain':
                self.msg.terrain = val
        print(self.msg)
        pub.publish(self.msg)


class SubscribeManager:
    def __init__(self) -> None:
        pass

class Main(App):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        PI = 3.14
        waypoints_pos = [[1, 0, 0],
                        [1, 1, 0],#,
                        [0, 1, 0],#,
                        [0, 0, 0],#,
                        [1, 0, 0]]

        waypoints_ang = [[0, 0, PI/2],
                         [0, 0, PI],
                         [0, 0, PI*3/2],
                         [0, 0, PI*2],
                         [0, 0, PI*5/2]]
        self.canvas = TestApp(waypoints_pos, waypoints_ang)
        self.canvas_editor = self.build_right_layout()


    def btn_callback_publish(self, *largs, **kwargs):
        # use it with partial()
        print(kwargs)
        pm.set_waypoints(self.canvas.waypoints)
        pm.change_and_publish(**kwargs)

    def btn_callback_clear(self, *largs, **kwargs):
        print("clear clicked")
        self.canvas.clear_waypoints()

    def btn_callback_add(self, *largs, **kwargs):
        print("add clicked")
        print(self.canvas.center_x, self.canvas.center_y)
        pos = [self.canvas.center_x, self.canvas.center_y]
        ang = [0, 0, 0]
        self.canvas.add_waypoint(pos, ang)

    def build_bottom_layout(self):
        # Buttons for Sending Topic Messages
        bottom_layout = BoxLayout(size_hint=(1, None), height=50)

        btn_robot1 = Button(text='Plan Monoped')
        btn_robot1.bind(on_press=partial(self.btn_callback_publish, robot=0))

        btn_robot2 = Button(text='Plan Biped')
        btn_robot2.bind(on_press=partial(self.btn_callback_publish, robot=1))

        btn_robot3 = Button(text='Plan Aliengo')
        btn_robot3.bind(on_press=partial(self.btn_callback_publish, robot=4))

        bottom_layout.add_widget(btn_robot1)
        bottom_layout.add_widget(btn_robot2)
        bottom_layout.add_widget(btn_robot3)

        return bottom_layout

    def build_right_layout(self):
        right_layout = BoxLayout(orientation='vertical', 
                                    size_hint=(None, 1), width=300)
        
        l1 = BoxLayout(orientation='horizontal', size_hint=(1, None), height=50)
        l1_btn_clear = Button(text="Clear")
        l1_btn_clear.bind(on_press=self.btn_callback_clear)
        l1.add_widget(l1_btn_clear)
        l1_btn_add = Button(text="Add Waypoint")
        l1_btn_add.bind(on_press=self.btn_callback_add)
        l1.add_widget(l1_btn_add)

        right_layout.add_widget(l1)

        n_waypoints = len(self.canvas.waypoints)
        l2_list = [None] * n_waypoints
        for i, waypoint in enumerate(self.canvas.waypoints):
            pos = waypoint.world_pos
            ang = waypoint.world_ang
            l2 = BoxLayout(orientation='vertical', size_hint=(1, None), height=90)
            l2_title = BoxLayout(orientation='horizontal', size_hint=(1, None), height=30)
            l2_title_label = Label(text='waypoint ['+str(i)+']', color=[0,0,0], size_hint_x=4)
            l2_title_button = Button(text='Remove')
            l2_title.add_widget(l2_title_label)
            l2_title.add_widget(l2_title_button)

            l2_pos = BoxLayout(orientation='horizontal', size_hint=(1, None), height=30)
            l2_pos_label = Label(text='pos', color=[0,0,0])
            l2_pos_x = TextInput(multiline=False, text="%.2f"%pos[0])
            l2_pos_y = TextInput(multiline=False, text="%.2f"%pos[1])
            l2_pos_z = TextInput(multiline=False, text="0")
            l2_pos.add_widget(l2_pos_label)
            l2_pos.add_widget(l2_pos_x)
            l2_pos.add_widget(l2_pos_y)
            l2_pos.add_widget(l2_pos_z)

            l2_ang = BoxLayout(orientation='horizontal', size_hint=(1, None), height=30)
            l2_ang_label = Label(text='ang', color=[0,0,0])
            l2_ang_x = TextInput(multiline=False, text="%.2f"%ang[0])
            l2_ang_y = TextInput(multiline=False, text="%.2f"%ang[1])
            l2_ang_z = TextInput(multiline=False, text="%.2f"%ang[2])
            l2_ang.add_widget(l2_ang_label)
            l2_ang.add_widget(l2_ang_x)
            l2_ang.add_widget(l2_ang_y)
            l2_ang.add_widget(l2_ang_z)

            def on_text(instance, value):
                def str2float(str):
                    if str=='': return 0
                    return float(str)
                pos = [str2float(l2_pos_x.text), str2float(l2_pos_y.text), str2float(l2_pos_z.text)]
                ang = [str2float(l2_ang_x.text), str2float(l2_ang_y.text), str2float(l2_ang_z.text)]
                print('on text event')
                print(pos, ang, value)
                # waypoint.move_point(pos=pos, ang=ang)

            l2_pos_x.bind(text=on_text)

            l2.add_widget(l2_title)
            l2.add_widget(l2_pos)
            l2.add_widget(l2_ang)

            right_layout.add_widget(l2)

        # btn_tmp2 = Button(text="Random", background_color = BUTTON_COLOR)
        # # btn_tmp2.background_down = ''
        # btn_tmp2.background_normal = ''
        # btn_tmp3 = Button(text="Random")
        # right_layout.add_widget(btn_tmp2)
        # right_layout.add_widget(btn_tmp3)
        placeholder = Label(text='')
        right_layout.add_widget(placeholder)

        return right_layout

    def build_main_layout(self):
        main_layout = BoxLayout(orientation='horizontal')
        # let's wrap this...
        rfl = FloatLayout()
        rfl.add_widget(self.canvas)
        main_layout.add_widget(rfl)

        # Right layout for setting up stuffs
        self.right_layout = self.build_right_layout()
        
        main_layout.add_widget(self.right_layout)
        return main_layout

    def build(self):

        self.bottom_layout = self.build_bottom_layout()
        self.main_layout = self.build_main_layout()
        
        root = BoxLayout(orientation='vertical')
        root.add_widget(self.main_layout)
        root.add_widget(self.bottom_layout)

        return root

def sub_callback(msg):
    return  # sub callback is handled by vis_pub.py
    import os
    raw_cmd = msg.data.split(' ')  # becareful, rospy doesn't output valid error...
    path = '~/.ros/' + raw_cmd[-1]
    fixed_command = ' '.join(raw_cmd[:-1] + [path])
    print("callback: ",fixed_command)
    os.system(fixed_command)

if __name__ == '__main__':

    pub = rospy.Publisher('/towr/user_command_seq', TowrCommandSeq, queue_size=1)
    rospy.Subscriber("/towr/ros_vis_traj", std_msgs.msg.String, sub_callback)
    rospy.init_node('ros_gui_kivy', anonymous=False)
    pm = PublishManager()
    Main().run()
