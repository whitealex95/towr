#! /home/whitealex95/anaconda3/bin/python
from mimetypes import init
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
from kivy.uix.widget import Widget

from kivy.uix.slider import Slider
from kivy.graphics import Color, Bezier, Line, Rectangle, Ellipse, InstructionGroup

from kivy.core.window import Window
from kivy.utils import get_color_from_hex

# custom msg
from towr_gui_kivy.msg import TowrCommand2, TowrCommandSeq
from xpp_msgs.msg import StateLin3d

from utils.waypoint import SET_POINT_COLOR, WayPoint, WayPointManager
from utils.traj import TrajectoryManager, TRAJ_STEP_SIZE


# Clear entire color to whitish./
Window.clearcolor = get_color_from_hex('#f0f9f9')

POINT_SIZE = 10
# SET_UNACTIVE_COLOR = lambda : Color(0, 1, 0)
# SET_ACTIVE_COLOR = lambda : Color(1, 0, 0)
SET_AXIS_COLOR = lambda : Color(0.2, 0.2, 0.2)
SET_COLOR = lambda : Color(0, 0, 0)
SET_INIT_POINT_COLOR = lambda : Color(1, 0, 0)
SET_LINE_COLOR = lambda : Color(0, 0, 1)

BUTTON_COLOR = (129/255, 212/255, 250/255, 1)

class MouseManager():
    def __init__(self) -> None:
        # Todo: use mouse separate class for complicated operation
        pass


s = [50, 50, 1]
t = [250, 275, 0]

class AxisManager():
    def __init__(self, canvas):
        self.canvas = canvas
        self.c_scale = s
        self.c_origin = t
        self.draw_axis()

        
    def draw_axis(self):
        with self.canvas:
            SET_AXIS_COLOR()
            self.x_axis = Line(points=[0, self.c_origin[1], 
                                    3*self.c_origin[0], self.c_origin[1]],
                                width=0.7)
            self.y_axis = Line(points=[self.c_origin[0], 0,
                                    self.c_origin[0], 3*self.c_origin[1]],
                                width=0.7)

class TestApp(FloatLayout):
    def __init__(self, waypoints_pos, waypoints_ang, *args, **kwargs):
        super(TestApp, self).__init__(*args, **kwargs)
        self.d = 10  # pixel tolerance when clicking on a point
        self.axisgrid = AxisManager(self.canvas)
        self.wm = WayPointManager(waypoints_pos, waypoints_ang, self.canvas)
        self.tm = TrajectoryManager(self.wm.waypoints[1],
                                    canvas=self.canvas)
        print(self.center)

        Window.bind(on_key_down = self._on_keyboard_down)

    @property
    def waypoints(self):
        return self.wm.waypoints

    def clear_waypoints(self):
        self.wm.clear_waypoints()

    def _on_keyboard_down(self, instance, keyboard, keycode, text, modifiers):
        if text in ['w', 'a', 's', 'd']:
            self.tm.update_traj(text)
        if text == ' ':
            self.proceed_next_trajectory()
    
    def proceed_next_trajectory(self):
        next_x, next_y, next_yaw = self.tm.traj.future_pos_angs[0]
        # __import__('pdb').set_trace()
        new_wp = self.wm.add_waypoint([next_x, next_y], 
                            ang=[0, 0, next_yaw],
                            coord_type='w')
        self.tm.update_current_waypoint(new_wp)

    def on_touch_down(self, touch):
        touch_x, touch_y = touch.pos
        if self.collide_point(touch.pos[0], touch.pos[1]):
            # Check if any waypoint is clicked. Selection based on waypoint order
            if self.wm.check_waypoint_clicked(touch_x, touch_y, self.d):
                return True
            self.wm.add_waypoint_c(c_pos = touch.pos)
            return True
        # return super(TestApp, self).on_touch_down(touch)
        else:
            print("Out of Canvas")
            

    def on_touch_up(self, touch):
        if self.collide_point(touch.pos[0], touch.pos[1]):
            if self.wm.clicked_wp is not None:
                self.wm.clicked_wp.mouse_up()
                self.wm.clicked_wp = None
                return True
            return super(TestApp, self).on_touch_up(touch)

    def on_touch_move(self, touch):
        if self.collide_point(touch.pos[0], touch.pos[1]):
            # modify waypoint
            if self.wm.clicked_wp is not None:
                self.wm.clicked_wp.mouse_drag(touch.pos[0], touch.pos[1])
                self.wm.update_lines()

                return True
            return super(TestApp, self).on_touch_move(touch)



class PublishManager:
    def __init__(self):
        msg = TowrCommandSeq()
        N = 1 / TRAJ_STEP_SIZE
        for j in range(3):
            goal_lin = StateLin3d()
            goal_lin.pos.x = (j+1) * 1.3
            msg.goal_lin_seq.append(goal_lin)

            goal_ang = StateLin3d()
            msg.goal_ang_seq.append(goal_ang)

        # msg.total_duration = 3          # reach in 3sec
        msg.total_duration = 3 / N         # reach in 3sec
        msg.replay_trajectory = False
        msg.replay_speed = 1            # x1.00 speed
        msg.play_initialization = False
        msg.plot_trajectory = False
        msg.optimize = True             # optimize trajectory
        msg.robot = 0                   # 0: monoped, 1: biped, 4: Aliengo
        msg.terrain = 0
        msg.gait = 0                    # 0: walk, 1: trot
        msg.gait = 1                    # 0: walk, 1: trot
        # msg.gait_n = int(9 / N)                 # default: 6 gait blocks prob?
        msg.gait_n = 6                  # default: 6 gait blocks prob?
        msg.optimize_phase_durations = False
        # msg.optimize_phase_durations = True

        self.msg = msg

    def set_waypoints(self, waypoints):
        self.msg.goal_lin_seq.clear()
        self.msg.goal_ang_seq.clear()

        for waypoint in waypoints:
            print('here')
            goal_lin = StateLin3d()
            [x,y,z] = waypoint.world_pos + [0]
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
        waypoints_pos = [
                        [0, 0, 0],  # init point
                        [TRAJ_STEP_SIZE, 0, 0],
                        ]
        waypoints_ang = [
                        [0, 0, 0],  # init point
                        [0, 0, 0],
                        ]

        # waypoints_pos = [
        #                 [0, 0, 0],  # init point
        #                 [1, 0, 0],
        #                 [1, 1, 0],#,
        #                 [0, 1, 0],#,
        #                 [0, 0, 0],#,
        #                 [1, 0, 0]]

        # waypoints_ang = [
        #                 [0, 0, 0],  # init point
        #                 [0, 0, PI/2],
        #                 [0, 0, PI],
        #                 [0, 0, PI*3/2],
        #                 [0, 0, PI*2],
        #                 [0, 0, PI*5/2]]
        self.map = TestApp(waypoints_pos, waypoints_ang)
        self.map_editor = self.build_right_layout()


    def btn_callback_publish(self, *largs, **kwargs):
        # use it with partial()
        print(kwargs)
        pm.set_waypoints(self.map.waypoints[1:])
        pm.change_and_publish(**kwargs)

    def btn_callback_clear(self, *largs, **kwargs):
        print("clear clicked")
        self.map.clear_waypoints()

    def btn_callback_add(self, *largs, **kwargs):
        print("add clicked")
        print(self.map.center_x, self.map.center_y)
        pos = [self.map.center_x, self.map.center_y]
        ang = [0, 0, 0]
        self.map.add_waypoint(pos, ang)

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

        n_waypoints = len(self.map.waypoints)
        l2_list = [None] * n_waypoints
        for i, waypoint in enumerate(self.map.waypoints):
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
        rfl.add_widget(self.map)
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
