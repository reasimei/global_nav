#!/usr/bin/env python
# -*- coding=utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

class CmdVelRepublisher:
    def __init__(self):
        rospy.init_node('cmd_vel_republisher')
        self.rate = rospy.Rate(1000)  # 提高到1000Hz
        
        # 获取机器人名称参数
        robot_name = rospy.get_param('~robot_name', 'epuck_robot_0')
        robot_number = robot_name.split('_')[-1]  # 获取机器人编号
        
        # 创建发布者 - 使用完整的话题名称
        self.pub = rospy.Publisher(
            f'/epuck_robot_{robot_number}/mobile_base/cmd_vel',
            Twist,
            queue_size=1
        )
        
        # 创建订阅者 - 订阅本地命名空间下的话题
        self.sub = rospy.Subscriber(
            'cmd_vel',  # 使用相对话题名
            Twist,
            self.callback
        )
        
        rospy.loginfo(f"Started cmd_vel republisher for epuck_robot_{robot_number}")
        
    def callback(self, msg):
        self.pub.publish(msg)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CmdVelRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass