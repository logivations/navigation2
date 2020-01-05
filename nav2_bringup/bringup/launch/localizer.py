import sys
import time

from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv._get_entity_state import GetEntityState_Request

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from scripts.spawn_entity import quaternion_from_euler
from std_msgs.msg import Header

import tf2_ros



rclpy.init(args=sys.argv)
node = rclpy.create_node('fake_camera')

br = tf2_ros.TransformBroadcaster(node)


odom_pub = node.create_publisher(Odometry, '/fake_odom', qos_profile=0)

odom = Odometry()
header = Header()
header.frame_id = '/odom'

add_two_ints = node.create_client(GetEntityState, '/get_entity_state')
while not add_two_ints.wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')

while True:
    req = GetEntityState_Request(name="turtlebot3_waffle")
    resp = add_two_ints.call_async(req)
    rclpy.spin_until_future_complete(node, resp, timeout_sec=1)

    resp = resp.result()

    if not resp:
        print('no pos found')
        time.sleep(1)
        continue

    print(resp)
    odom.pose.pose = resp.state.pose
    odom.twist.twist = resp.state.twist

    header.stamp = Time().to_msg()
    odom.header = header

    odom_pub.publish(odom)

    t = TransformStamped(header=Header(frame_id="map", stamp=resp.header.stamp))

    pose = resp.state.pose

    #t.header.stamp = resp.header.stamp
    #t.header.frame_id = "map"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = 0.0

    t.transform.rotation.x = pose.orientation.x
    t.transform.rotation.y = pose.orientation.y
    t.transform.rotation.z = pose.orientation.z
    t.transform.rotation.w = pose.orientation.w

    br.sendTransform(t)
    t = TransformStamped(header=Header(frame_id="odom", stamp=resp.header.stamp))#

    t.child_frame_id = "base_footprint"
    t.transform.translation.x = 0.
    t.transform.translation.y = 0.
    t.transform.translation.z = 0.0

    #br.sendTransform(t)

    time.sleep(0.03)

