# TurtleBot Data Acquisition and Visualization System

This robust project is designed to capture, store, and visually represent telemetry data from a TurtleBot using the Robot Operating System (ROS). The framework employs a custom-built ROS node for real-time data acquisition, which is then stored in the high-performance InfluxDB time-series database.

One of the unique features of this system is the seamless data transfer mechanism from InfluxDB to a MySQL database, facilitated by a specific task built using the powerful Django web framework. This process allows for enhanced data manipulation and the ability to leverage SQL-based analysis on collected telemetry data.

To provide users with a comprehensive visual understanding of the TurtleBot's operations, this project utilizes Grafana, a multi-platform open-source analytics and interactive visualization web application. Data is presented through custom-built Grafana dashboards, providing users with insights into the TurtleBot's operations at a glance.

Additionally, a custom Django webpage offers another layer of data visualization, enabling users to interact with and analyze the TurtleBot data from various perspectives.

This project serves as a comprehensive solution for gathering, storing, and visualizing TurtleBot telemetry, effectively catering to research, testing, and monitoring purposes. It stands as a testament to the power of integrating multiple open-source tools to deliver a highly functional and flexible telemetry data management platform.
## Installation

### Prerequisites

- Install ROS. Refer to this [documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) for a comprehensive guide on how to install ROS and calibrate and control TurtleBot.
- Install MySQL. Follow the steps in this [DigitalOcean tutorial](https://www.digitalocean.com/community/tutorials/how-to-install-mysql-on-ubuntu-20-04) to install MySQL on Ubuntu 20.04. After installation, create a new database named rosdb using the following SQL command:

```sql
  CREATE DATABASE rosdb;
```

- Install Grafana using this [documentation](https://grafana.com/docs/grafana/latest/setup-grafana/installation/debian/).
- Setup Grafana to connect with InfluxDB using this [guide](https://grafana.com/docs/grafana/latest/getting-started/get-started-grafana-influxdb/).

### Clone the repository

- Clone the repository into your catkin src directory with the command:

 ```bash
cd ~/catkin_ws/src
git clone https://github.com/abujarad21/CPS_Monitoring.git
```

### Build the project

- Navigate back to the root of your catkin workspace and build the project using `catkin_make`:

 ```bash
cd ~/catkin_ws
catkin_make
```

### Set up the Django project

- Navigate to the directory containing the Django project (it should be within the cloned repository).
- Install the necessary Python packages by running:
```bash
cd ~/catkin_ws/src/CPS_Monitoring/djangoproject/
pip install -r requirements.txt
```
- Run the following commands for database migrations:

 ```bash
python manage.py makemigrations
python manage.py migrate
```
Note: The Django project will not be built using `catkin_make`, as it is not a ROS node.

## Usage

1. Start the ROS core by running the command `roscore` in the host PC.
2. On the TurtleBot, run the following commands each one in one robot resberrypi:

 ```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch set_base_frame:=tb3_0/base_footprint set_odom_frame:=tb3_0/odom set_map_frame:=tb3_0/map
```

 ```bash
ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch set_base_frame:=tb3_1/base_footprint set_odom_frame:=tb3_1/odom set_map_frame:=tb3_1/map
```
3. On the host PC, run the following commands each one in one tab:
 ```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_remote.launch multi_robot_name:=tb3_0
```
 ```bash
ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_remote.launch multi_robot_name:=tb3_1
```
4. To control the TurtleBot, run the following commands each one in one tab:
 ```bash
ROS_NAMESPACE=tb3_0 rosrun turtlebot3_teleop turtlebot3_teleop_key
```
 ```bash
ROS_NAMESPACE=tb3_1 rosrun turtlebot3_teleop turtlebot3_teleop_key
```
5. Run the `influxdb_monitoring` node:
```bash
rosrun influxdb_monitoring msgs_to_influxdb_basic.py
```
6. Inside the Django project directory, start the Django task to transfer data from InfluxDB to MySQL each in one tab:
```bash
cd ~/catkin_ws/src/CPS_Monitoring/djangoproject/
celery -A influxdb2mysqldb worker --loglevel=info
```
```bash
cd ~/catkin_ws/src/CPS_Monitoring/djangoproject/
celery -A influxdb2mysqldb beat --loglevel=info
```
7. Run the Django server on port 8000:
```bash
cd ~/catkin_ws/src/CPS_Monitoring/djangoproject/
python manage.py runserver 8000
```

## Contributing

If you would like to contribute, please fork the repository and use a feature branch. Pull requests are warmly welcome.

## License

The project is licensed under the MIT license.

## Contact

If you have any questions, please feel free to contact me at Abujarad73@gmail.com.
