import rosbag
import rospy

input_bag = '/home/whitealex95/.ros/towr_trajectory-1.bag'
output_bag = '/home/whitealex95/.ros/towr_trajectory-fixed.bag'

input_bag 
t_margin = rospy.Duration.from_sec(3)
print('start')
with rosbag.Bag(output_bag, 'w') as outbag:

  for topic, msg, t in rosbag.Bag(input_bag).read_messages():
    outbag.write(topic, msg, t+t_margin)