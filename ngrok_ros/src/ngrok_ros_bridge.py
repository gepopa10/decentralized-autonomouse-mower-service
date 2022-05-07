#!/usr/bin/env python
import rospy
import requests
import json
from std_msgs.msg import String
import os
from ngrok_ros.srv import StartTunnel, StartTunnelResponse
from ngrok_ros.msg import PublicURLs


class ngrokROS(object):
    def __init__(self):
        """
        ROS package for ngrok to expose local ports to the internet
        """
        self.message_pub_ = rospy.Publisher("/ngrok_ros/public_addr", PublicURLs, queue_size=1, latch=True)
        self.urls_msg_ = PublicURLs()
        rospy.Subscriber("/ngrok_ros/start_tunnel", String, self.start_tunnel_callback_)
        service_server = rospy.Service("ngrok_ros/start_tunnel", StartTunnel, self.start_tunnel_service_)
        rospy.spin()

    def start_tunnel_service_(self, req):
        public_url = self.start_tunnel_(req.addr, req.proto, req.name)
        response = StartTunnelResponse()
        response.public_url = public_url
        return response

    def start_tunnel_callback_(self, msg):
        addr, proto, name = msg.data.split(",")
        public_url = self.start_tunnel_(addr, proto, name)
        self.urls_msg_.urls.append(name + ":" + public_url)
        self.message_pub_.publish(self.urls_msg_)

    def start_tunnel_(self, addr, proto, name):
        json_request = {"addr": addr, "proto": proto, "name": name}
        try:
            r = requests.post("http://localhost:4040/api/tunnels", json=json_request)
            public_url = r.json()["public_url"]
        except Exception as e:
            print(e)
            rospy.logerr("Check if ngrok is running and try again")
            public_url = ""
        return public_url


if __name__ == "__main__":
    rospy.init_node("ngrok_ros")
    ngrok_ros = ngrokROS()

