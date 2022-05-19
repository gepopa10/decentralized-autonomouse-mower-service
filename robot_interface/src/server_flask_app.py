#!/usr/bin/env python
import os
import time
# import rospy
# from threading import Thread
from flask import Flask, jsonify
# from nav_msgs.msg import Odometry
# from rospy_message_converter import json_message_converter

# odom = Odometry()

# def update_odom(data):
#     global odom
#     odom = data

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.
# Thread(target=lambda: rospy.init_node('robot_interface_server', disable_signals=True)).start()

# A subscriber to Odometry which is called when a message is received
# odom_subscriber = rospy.Subscriber("/odom", Odometry, update_odom)
app = Flask(__name__)


# @app.route('/get_odom', methods = ['GET'])
# def get_odom():
#     global odom
#     json_str = json_message_converter.convert_ros_message_to_json(odom)
#     return json_str


@app.route('/get_robot_url', methods = ['GET'])
def get_robot_url():
    # robot_url_param_name = rospy.get_param('/robot_url_param_name', '/robot_url')
    # robot_url = rospy.get_param(robot_url_param_name, 'localhost:17777')
    return jsonify(robot_url='localhost:17777')


if __name__ == '__main__':
    # try:
        # flask_app_server_port = rospy.get_param('/robot_interface_server/flask_app_server_port', 6000)
    
    app.run(debug=True, host='0.0.0.0', port=6000, threaded=True)

    while True:
        time.sleep(1)
        # If we press control + C, the node will stop.
    #     rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass
