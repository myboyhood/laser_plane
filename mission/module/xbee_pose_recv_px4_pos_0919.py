#!/usr/bin/env python
#coding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from tf.transformations import euler_from_quaternion
from serial import Serial
from struct import pack,unpack
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
#  need install serial first!!
#  sudo apt-get install ros-melodic-serial
import math


class XbeeUAV:
    def __init__(self, id=""):
        # self.uav_id = id
        self.car_gps_local_pos = PoseStamped()
        self.car_px4_local_pos = PoseStamped()
        self.plane_pos = PoseStamped()
        self.car_gps = NavSatFix()
        self.plane_home_gps = NavSatFix()
        self.plane_home_local = Odometry()
        self.start_velocity_esti_msg = Bool()
        self.start_trace_msg = Bool()
        self.plane_home_gps_static_lon = 0
        self.plane_home_gps_static_lat = 0
        self.get_home_gps = True
        self.get_home_local = True
        self.home_local_x_sum = 0
        self.home_local_y_sum = 0
        self.home_local_x = 0
        self.home_local_y = 0
        self.car_to_plane_bias_x = 0
        self.car_to_plane_bias_y = 0
        self.count = 0
        self.avg_num = 100
        self.local_x = 0
        self.local_y = 0
        self.local_vx = 0
        self.local_vy = 0
        self.local_sum = 0
        # self.velocity = TwistStamped()
        # port
        self.ser = Serial("/dev/xbee_base", 57600, timeout=0.03)
        self.car_gps_local_pub = rospy.Publisher('/car_gps_local_position', PoseStamped, queue_size=1)
        self.car_px4_local_pub = rospy.Publisher('/car_px4_local_position', PoseStamped, queue_size=1)
        # lon_x lat_y ratio
        self.lon_x = 1.33021075925e-05*10e7
        self.lat_y = 8.98488381215e-06*10e7
        # data
        self._lon = 0
        self._lat = 0
        self._sum = 0
        self.car_gps_local_pos.pose.position.x = 0
        self.car_gps_local_pos.pose.position.y = 0
        self.yaw_bias = 0.0
        # self.car_vel_esti_start_x = self.car_pos.pose.position.x - 20*math.cos(self.yaw_bias)
        # self.car_vel_esti_start_y = self.car_pos.pose.position.y - 20*math.sin(self.yaw_bias)
        # self.car_vel = 0.6 # 0.6*looprate = velocity
        # self.delta_x = self.car_vel*math.cos(self.yaw_bias)
        # self.delta_y = self.car_vel*math.sin(self.yaw_bias)


    def plane_home_pos_cb(self,data):
        self.plane_home_gps = data
        if self.get_home_gps and abs(self.plane_home_gps.longitude) > 1:
            self.get_home_gps = False
            self.plane_home_gps_static_lon = self.plane_home_gps.longitude*10e7%10e6
            self.plane_home_gps_static_lat = self.plane_home_gps.latitude*10e7%10e5

    def plane_local_pos_cb(self,data):
        self.plane_home_local = data
        if self.get_home_local and abs(self.plane_home_local.pose.pose.position.x) > 0.1:
            self.home_local_x_sum += self.plane_home_local.pose.pose.position.x
            self.home_local_y_sum += self.plane_home_local.pose.pose.position.y
            self.count += 1
            print("local_sum count = %d", self.count)
            if self.count >= self.avg_num:
                print("sum over , avg value")
                self.get_home_local = False
                self.home_local_x = self.home_local_x_sum/self.avg_num
                self.home_local_y = self.home_local_y_sum/self.avg_num

    def start_velocity_esti_cb(self,data):
        self.start_velocity_esti_msg = data

    def start_trace_cb(self,data):
        self.start_trace_msg = data

    def recv_pose(self):
        rospy.init_node('xbee_pose_recv', anonymous=True)
        rospy.loginfo("start send xbee pose recv..")
        rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, self.plane_home_pos_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.plane_local_pos_cb)
        rospy.Subscriber('/start_velocity_esti_topic', Bool, self.start_velocity_esti_cb)
        rospy.Subscriber('/start_trace_topic', Bool, self.start_trace_cb)
        rate = rospy.Rate(30)
        # rate.sleep()
        # while not rospy.is_shutdown() and not self.start_trace_msg.data:
        #     print("wait drone hover...")
        #     rate.sleep()

        while not rospy.is_shutdown():
            data = self.ser.read(29)
            print(len(data))
            if data != "":
                head1, head2,self._lon,self._lat, self.local_x, self.local_y, self.local_vx, self.local_vy, end = unpack('@2b6ib', data)
                self.local_x = self.local_x / 100.0
                self.local_y = self.local_y / 100.0
                self.local_vx = self.local_vx / 100.0
                self.local_vy = self.local_vy / 100.0
                print(self._lon,self._lat)
                print("plane_origin_lon", self.plane_home_gps.longitude)
                print("plane_origin_lat", self.plane_home_gps.latitude)
                print("car_local_x", self.local_x)
                print("car_local_y", self.local_y)
                print("home_lon", self.plane_home_gps_static_lon)
                print("home_lat", self.plane_home_gps_static_lat)
                print("home_local_x:", self.home_local_x)
                print("home_local_y:", self.home_local_y)
                self.car_gps.longitude = self._lon
                self.car_gps.latitude = self._lat
                """ change to local position , plane home is origin, x axis towards east, y axis towards west"""
                self.car_gps_local_pos.pose.position.x = (self.car_gps.longitude - self.plane_home_gps_static_lon) / self.lon_x
                self.car_gps_local_pos.pose.position.y = (self.car_gps.latitude - self.plane_home_gps_static_lat) / self.lat_y
            """  use px4 local pos"""
            self.car_px4_local_pos.pose.position.x = self.local_x+self.car_to_plane_bias_x
            self.car_px4_local_pos.pose.position.y = self.local_y+self.car_to_plane_bias_y
            rospy.loginfo("gps_local_car_x: %f", self.car_gps_local_pos.pose.position.x)
            rospy.loginfo("gps_local_car_y: %f", self.car_gps_local_pos.pose.position.y)
            rospy.loginfo("px4_local_car_x: %f", self.local_x+self.car_to_plane_bias_x)
            rospy.loginfo("px4_local_car_y: %f", self.local_y+self.car_to_plane_bias_y)
            self.car_gps_local_pos.pose.position.z = 3 # the height of car is 3m

            # print("self.car_pos.pose.position_x", self.car_pos.pose.position.x)
            # print("self.car_pos.pose.position_y", self.car_pos.pose.position.y)
            self.car_gps_local_pub.publish(self.car_gps_local_pos)
            self.car_px4_local_pub.publish(self.car_px4_local_pos)
            rate.sleep()

if __name__ == '__main__':
    uav = XbeeUAV()
    uav.recv_pose()