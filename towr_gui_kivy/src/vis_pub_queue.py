from time import sleep
from nbformat import read
from sqlalchemy import false
import rosbag
import rospy
from xpp_msgs.msg import RobotStateCartesian, RobotParameters
import std_msgs
import asyncio
from datetime import datetime
from queue import Queue


# async def publish_msg_queue_loop():
#   global msg_queue, pub
#   d0 = datetime.now()

#   while True:
#     if msg_queue.empty():
#       await asyncio.sleep(0.01)
#       continue
#     msg = msg_queue.get()
#     pub.publish(msg.message)
#     print(msg.timestamp, (datetime.now()-d0).microseconds)
#     await asyncio.sleep(0.01)

pub_alive = False # flag to check if publisher is running
async def publish_msg_queue():
  global pub_alive
  pub_alive = True
  global msg_queue, pub
  d0 = datetime.now()
  while not msg_queue.empty():
    msg = msg_queue.get()
    pub.publish(msg.message)
    dt = (datetime.now()-d0)
    print(msg.timestamp, dt.seconds, dt.microseconds)
    await asyncio.sleep(0.01)
  pub_alive = False

def sub_callback(msg):
  global msg_queue, pub
  print("sub received")
  raw_cmd = msg.data.split(' ')  # becareful, rospy doesn't output valid error...
  bagname = '/home/whitealex95/.ros/' + raw_cmd[-1]
  msgs = rosbag.Bag(bagname).read_messages()
  msgs_list = list(msgs)

  for msg in msgs_list:
    if msg.topic == '/xpp/state_des':
      print(msg.timestamp)
      msg_queue.put(msg)
  
  if not pub_alive:
    asyncio.run(publish_msg_queue())

def main():
  global msg_queue, pub

  pub = rospy.Publisher('/xpp/state_des', RobotStateCartesian, queue_size=1)
  rospy.Subscriber("/towr/ros_vis_traj", std_msgs.msg.String, sub_callback)
  rospy.init_node('ros_vis_pub_queue', anonymous=False)

  # msg_queue = asyncio.Queue()
  msg_queue = Queue()
  rospy.spin()


if __name__ == "__main__":
    # asyncio.run(main())
    main()
