#!/usr/bin/env python
import rospy
import roslaunch
from ngrok_ros.srv import StartTunnel
from urlparse import urlparse
import os
import sys

def request_ngrok_url(port):
    rospy.wait_for_service('/ngrok_ros/start_tunnel')
    try:
        start_tunnel = rospy.ServiceProxy('/ngrok_ros/start_tunnel', StartTunnel)
        resp1 = start_tunnel(str(port),'tcp', 'ros_bridge')
        return resp1.public_url
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':

    rospy.init_node('web_rosbridge_query_and_launch', anonymous=True)
    rosbridge_server_port = rospy.get_param('/web_rosbridge_interface/rosbridge_server_port', 9090)

    robot_url_param_name = rospy.get_param('/robot_url_param_name', '/robot_url')

    # raw_url = request_ngrok_url(rosbridge_server_port)
    # host = urlparse(raw_url)
    # hostname = host.hostname
    # port = host.port

    hostname = rospy.get_param('/web_rosbridge_interface/web_rosbridge_external_address', 'localhhost')
    port = rospy.get_param('/web_rosbridge_interface/web_rosbridge_external_port', 9999)

    rospy.loginfo("Robot web url is " + str(hostname) + ":" + str(port))

    try:
        rospy.set_param(robot_url_param_name, hostname + ':' + str(port))
    except:
        port = 9000
        rospy.set_param(robot_url_param_name, "localhost:12700")

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    certs_path = os.path.dirname(sys.argv[0]) + "/certs"

    # cli_args = ['rosbridge_server', 'rosbridge_websocket.launch', 'port:=' + str(rosbridge_server_port), 'websocket_external_port:=' + str(8100),
    #             'ssl:=true', 'certfile:=' + certs_path + '/server_cert.pem', 'keyfile:=' + certs_path + '/server_key.pem', 'authenticate:=false']

    cli_args = ['rosbridge_server', 'rosbridge_websocket.launch', 'port:=' + str(rosbridge_server_port), 'websocket_external_port:=' + str(port)]

    roslaunch_args = cli_args[2:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
    parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)


    try:
        parent.start()
        # If we press control + C, the node will stop.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
