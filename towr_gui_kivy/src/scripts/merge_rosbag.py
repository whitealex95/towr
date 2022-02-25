import rosbag
import rospy

input_bag_list = ['/home/whitealex95/.ros/towr_trajectory-0.bag',
                  '/home/whitealex95/.ros/towr_trajectory-1.bag',
                  ]

output_bag = '/home/whitealex95/.ros/towr_trajectory-merged.bag'

print('start')
with rosbag.Bag(output_bag, 'w') as outbag:
  t_margin = rospy.Duration.from_sec(0)
  for input_bag in input_bag_list:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
      outbag.write(topic, msg, t+t_margin)
    # __import__('pdb').set_trace()
    t_margin = rospy.Duration.from_sec((t+t_margin).to_sec())