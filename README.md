# Decentralized Autonomous Grass Mower Service

![Simulation Rviz](images/rviz.png?raw=true "Title")

A decentralized autonomous lawn mowing service powered by chainlink with additional live snapshot and mint capabilities.

## Prerequisites
Developed and tested in an Ubuntu 18.04 environment.
To be able to run the demo:
- Install [ros-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) package on your machine.
- Install [Diode Network client](https://diode.io/products/diode-network/) which will expose the website, robot and robot api port over a decentralized network. You can configure a custom dns domain for free so the website url is more human readable. For example, during the video demo, the dns was chainlink-robot.diode.link. For more details check this [how-to](https://support.diode.io/article/ss32engxlq-publish-your-local-webserver).

## Run instructions
- Launch the bash script `./run.sh` at the root of this repo. This will do a few things:
    - Start diode and publish the http port (default = 4443) for the website and the flask server port (default = 3001).
    - Run an http server to have access to the website (over port 4443).
    - Launch Gazebo for the robot simulation.
    - Launch RVIZ to visualize data.
    - Launch the flask server of the robot to query the internal services of the robot from the outside world.
    - Launch the robot controller (a ROS node).
    - Launch the path planner (a ROS node).

- Run the chainlink job that supports calls on the smart contract external adapter.
    - Run a [local chainlink node](https://www.youtube.com/watch?v=DO3O6ZUtwbs&ab_channel=Chainlink)
    - Launch the `brownie/get_robot_request_job_big_word.toml` job on your local node

## Usage
Users can interact with the mowing service and the robot via its public website. The website url depends on the dns domain you choose when launching the diode network client locally. For the demo, the website was https://chainlink-robot.diode.link.

### WebApp

![WebApp](images/webapp.png?raw=true "Title")

1. Connect your wallet on the top-right.
2. Connect to the robot. This will use an external adapter to query the robot url to connect to using the rest-api on the robot.
3. Start a mission, you will have to pay ETH for this.
4. Take snapshots along the way which will be minted into your wallet.
5. View all your snapshots at any time.
