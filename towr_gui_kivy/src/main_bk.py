import rospy
from std_msgs.msg import Bool
from kivymd.app import MDApp
from kivymd.uix.behaviors import HoverBehavior
from kivy.lang import Builder
# you need to compile to make custom msg work.
from towr_gui_kivy.msg import TowrCommandGUI

from kivy.uix.button import Button
from kivy.uix.dropdown import DropDown


dropdown = DropDown()
robot_list = ['monoped', 'biped', 'hyq', 'anymal', 'aliengo']

def change_color(btn):
  console.log('hi')
  btn.md_bg_color = (1,1,1,1)
for robot in robot_list:
  btn = Button(text=robot, size_hint_y=None, height=44)
  btn.bind(on_release=lambda btn: dropdown.select(btn.text))

  btn.bind(on_enter=lambda btn: change_color(btn))
  dropdown.add_widget(btn)

mainbutton = Button(text='monoped', size_hint = (None, None), pos=(0,0))
mainbutton.bind(on_release = dropdown.open)
dropdown.bind(on_select = lambda instance, x: setattr(mainbutton, 'text', x))

# class TOWRDropdown(DropDown, HoverBehavior):
#   def __init__(self, item_list, )

class MainApp(MDApp):
  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.screen = Builder.load_file('ros_gui.kv')

  def build(self):
    self.screen.add_widget(mainbutton)
    return self.screen

  def my_function(self, *args):
    print("button pressed")
    msg = TowrCommandGUI()
    print(msg.goal_lin.pos.x)
    msg.goal_lin.pos.x = 2.3
    print(msg)
    pub.publish(msg)


if __name__=='__main__':
  pub = rospy.Publisher('/button', TowrCommandGUI, queue_size=1)
  rospy.init_node('simple_gui', anonymous=False)
  MainApp().run()