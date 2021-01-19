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
        self.car_px4_local_pos_from_power = PoseStamped()
        self.car_px4_local_pos_from_start = PoseStamped()
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
        self.plane_home_local_x_sum = 0
        self.plane_home_local_y_sum = 0
        self.plane_home_local_x = 0
        self.plane_home_local_y = 0                             
        self.car_to_plane_bias_x = 5*math.cos(-0.45)  # the distance between car and plane in real env
        self.car_to_plane_bias_y = 5*math.sin(-0.45)  # the distance between car and plane in real env
        self.car_home_x = 0  # accept initial car_px4_local_pos as home bias
        self.car_home_y = 0  # accept initial car_px4_local_pos as home bias
        """local_xy + car_to_plane_bias_xy - """
        # self.car_px4_local_setp_follow_x = 0  # consider plane and car initial drift
        # self.car_px4_local_setp_follow_y = 0  # consider plane and car initial drift
        self.get_car_home_bias = True
        self.count = 0
        self.avg_num = 100
        self.car_local_x_from_power = 0
        self.car_local_y_from_power = 0
        self.car_local_vx = 0
        self.car_local_vy = 0
        # self.car_local_sum = 0
        # self.velocity = TwistStamped()
        # port
        self.ser = Serial("/dev/xbee_base", 57600, timeout=0.03)
        self.car_gps_local_pub = rospy.Publisher('/car_gps_local_position', PoseStamped, queue_size=1)
        self.car_px4_local_from_power_pub = rospy.Publisher('/car_px4_local_position_from_power', PoseStamped, queue_size=1)
        self.car_px4_local_from_start_pub = rospy.Publisher('/car_px4_local_position_from_start', PoseStamped, queue_size=1)
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
        if self.get_home_local and abs(self.plane_home_local.pose.pose.position.x) > 0.001:
            self.plane_home_local_x_sum += self.plane_home_local.pose.pose.position.x
            self.plane_home_local_y_sum += self.plane_home_local.pose.pose.position.y
            self.count += 1
            print("local_sum count = %d", self.count)
            if self.count >= self.avg_num:
                print("sum over , avg value")
                self.get_home_local = False
                self.plane_home_local_x = self.plane_home_local_x_sum*1.0/self.avg_num
                self.plane_home_local_y = self.plane_home_local_y_sum*1.0/self.avg_num


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
            data = self.ser.read(1)
            print(len(data))
            if len(data) == 1:
                head1 = unpack('@b', data)
                print("data:",data)
                self.car_local_x_from_power = self.car_local_x_from_power / 100.0
                self.car_local_y_from_power = self.car_local_y_from_power / 100.0
                self.car_local_vx = self.car_local_vx / 100.0
                self.car_local_vy = self.car_local_vy / 100.0
                """ acqurie car home bias"""
                if self.get_car_home_bias and abs(self.car_local_x_from_power) > 0.01:
                    self.car_home_x = self.car_local_x_from_power
                    self.car_home_y = self.car_local_y_from_power
                    self.get_car_home_bias = False

                # print("car lon and lat: ", self._lon, self._lat)
                # print("plane_origin_lon", self.plane_home_gps.longitude)
                # print("plane_origin_lat", self.plane_home_gps.latitude)
                # print("car_local_x_from_power", self.car_local_x_from_power)
                # print("car_local_y_from_power", self.car_local_y_from_power)
                # print("home_lon", self.plane_home_gps_static_lon)
                # print("home_lat", self.plane_home_gps_static_lat)
                # print("plane_home_local_x:", self.plane_home_local_x)
                # print("plane_home_local_y:", self.plane_home_local_y)
                self.car_gps.longitude = self._lon
                self.car_gps.latitude = self._lat
                """ change to local position , plane home is origin, x axis towards east, y axis towards west"""
                self.car_gps_local_pos.pose.position.x = (self.car_gps.longitude - self.plane_home_gps_static_lon) / self.lon_x
                self.car_gps_local_pos.pose.position.y = (self.car_gps.latitude - self.plane_home_gps_static_lat) / self.lat_y
                """  use px4 local pos"""
                self.car_px4_local_pos_from_power.pose.position.x = self.car_local_x_from_power
                self.car_px4_local_pos_from_power.pose.position.y = self.car_local_y_from_power
                """ car_px4_local_setp_follow_xy , consider plane and car initial drift"""
                self.car_px4_local_pos_from_start.pose.position.x = self.car_local_x_from_power - self.car_home_x 
                self.car_px4_local_pos_from_start.pose.position.y = self.car_local_y_from_power - self.car_home_y 
                # rospy.loginfo("gps_local_car_x: %f", self.car_gps_local_pos.pose.position.x)
                # rospy.loginfo("gps_local_car_y: %f", self.car_gps_local_pos.pose.position.y)
                # rospy.loginfo("px4_local_car_x_from_power: %f", self.car_local_x_from_power)
                # rospy.loginfo("px4_local_car_y_from_power: %f", self.car_local_y_from_power)
                # rospy.logwarn("car_px4_local_from_start: %f", self.car_px4_local_pos_from_start.pose.position.x)
                # rospy.logwarn("car_px4_local_from_start: %f", self.car_px4_local_pos_from_start.pose.position.y)
                # rospy.logwarn('car_px4_local_x_from_planehome: %f', self.car_px4_local_pos_from_start.pose.position.x + self.car_to_plane_bias_x)
                # rospy.logwarn('car_px4_local_y_from_planehome: %f', self.car_px4_local_pos_from_start.pose.position.y + self.car_to_plane_bias_y)
            else:
                print('the data length is not 29 , but {}'.format(len(data)))

            self.car_gps_local_pub.publish(self.car_gps_local_pos)
            self.car_px4_local_from_power_pub.publish(self.car_px4_local_pos_from_power)
            self.car_px4_local_from_start_pub.publish(self.car_px4_local_pos_from_start)

            rate.sleep()

if __name__ == '__main__':
    uav = XbeeUAV()
    uav.recv_pose()