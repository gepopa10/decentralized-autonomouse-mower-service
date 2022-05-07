#!/usr/bin/env python
import rospy
import roslaunch
from ngrok_ros.srv import StartTunnel
from urlparse import urlparse

def request_ngrok_url(port):
    rospy.wait_for_service('/ngrok_ros/start_tunnel')
    try:
        start_tunnel = rospy.ServiceProxy('/ngrok_ros/start_tunnel', StartTunnel)
        resp1 = start_tunnel(str(port),'tcp', 'ros_bridge')
        return resp1.public_url
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':

    rospy.init_node('ngrok_rosbridge_query_and_launch', anonymous=True)
    rosbridge_server_port = rospy.get_param('/rosbridge_server_port', 9090)

    raw_url = request_ngrok_url(rosbridge_server_port)
    rospy.loginfo("NGROK url is " + raw_url)

    host = urlparse(raw_url)

    hostname = host.hostname
    port = host.port

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

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
