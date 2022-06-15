#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FollowWall:

    def __init__(self):
        self.angel = [0]*30
        self.distance = [10]*30

        print('code Running')
        self.pub_vel = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)
        self.sub_laser = rospy.Subscriber(
            '/tb3_1/scan', LaserScan, self.callback_laser)
        self.rate = rospy.Rate(20)

    def callback_laser(self, msg):
        speed = Twist()
        g = [10]*30
        self.distance = g

        a = []
        j = 0
        for i in range(360):

            a.append(msg.ranges[i])
            # print "At "+str(i)+" : "+str(a[i])
            if a[i] < 5:
                flag = True
                self.angel[j] = i
                self.distance[j] = a[i]
                print "j "+str(j)
                print str(self.angel[j])+"   :  " + str(self.distance[j])
                j += 1
        rospy.loginfo(self.angel)
        rospy.loginfo(self.distance)
        min_dis = min(self.distance)
        dist_index = self.distance.index(min_dis)
        obs_angel = self.angel[dist_index]
        print "min_dis = "+str(min_dis)
        if min_dis == 10:
            speed.linear.x = 0
            speed.angular.z = 0.3
        elif 0 < min_dis < 0.5:
            speed.linear.x = 0
            speed.angular.z = 0
        elif obs_angel < 30:
            speed.linear.x = 0.5
            speed.angular.z = 0
        elif 30 <= obs_angel < 90:
            speed.linear.x = 0.5
            speed.angular.z = 0.7
        elif 90 <= obs_angel < 180:
            speed.linear.x = 0
            speed.angular.z = 0.7
        elif 180 <= obs_angel < 270:
            speed.linear.x = 0.0
            speed.angular.z = -0.7
        elif 270 <= obs_angel < 329:
            speed.linear.x = 0.5
            speed.angular.z = - 0.7
        elif 330 <= obs_angel < 360:
            speed.linear.x = 0.5
            speed.angular.z = 0
        self.pub_vel.publish(speed)


if __name__ == '__main__':

    rospy.init_node('follow_wall')
    FollowWall()
    rospy.spin()
