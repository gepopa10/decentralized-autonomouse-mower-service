#!/usr/bin/env python
import os
import rospy
from threading import Thread
from flask import Flask, jsonify
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rospy_message_converter import json_message_converter

odom = Odometry()

def update_odom(data):
    global odom
    odom = data


image = Image()

def update_image(data):
    global image
    image = data

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.
Thread(target=lambda: rospy.init_node('robot_interface_server', disable_signals=True)).start()

# A subscriber to Odometry which is called when a message is received
odom_subscriber = rospy.Subscriber("/odom", Odometry, update_odom)
# A subscriber to Image which is called when a message is received
image_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, update_image)

app = Flask(__name__)


@app.route('/get_odom', methods = ['GET'])
def get_odom():
    global odom
    json_str = json_message_converter.convert_ros_message_to_json(odom)
    return jsonify(odometry=json_str)


@app.route('/get_image', methods = ['GET'])
def get_image():
    global image
    json_str = json_message_converter.convert_ros_message_to_json(image)
    # print(image.data)
    # print("len(image.data)", len(image.data))
    # print("height", image.height)
    # print("width", image.width)
    # print("step", image.step)
    # j = {"data" : image.data}
    return jsonify(test = [20, 30 ,40])


@app.route('/get_robot_url', methods = ['GET'])
def get_robot_url():
    robot_url_param_name = rospy.get_param('/robot_url_param_name', '/robot_url')
    robot_url = rospy.get_param(robot_url_param_name, 'localhost:17777')
    return jsonify(robot_url=robot_url)


@app.route("/")
def hello():
    return jsonify("Chainlink Robot API")


if __name__ == '__main__':
    try:
        flask_app_server_port = rospy.get_param('/robot_interface_server/flask_app_server_port', 6000)
        app.run(debug=False, host='0.0.0.0', port=flask_app_server_port, threaded=True)
        # debug=False otherwise there is a bug with ROS
        # IOError: [Errno 11] Resource temporarily unavailable
        # See https://github.com/coleifer/micawber/issues/59 where it ways that ran with debug there is a socket issue

        # If we press control + C, the node will stop.
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
