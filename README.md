# fiware-ros-gopigo
This ros package acts as a bridge between [FIWARE](https://www.fiware.org) and [ROS](http://wiki.ros.org/) through MQTT.

## Description
### `fiware2gopigo`
This ROS node receives a command from [FIWARE orion context broker](https://catalogue-server.fiware.org/enablers/publishsubscribe-context-broker-orion-context-broker) through MQTT.

When receiving a command, this node publish a series of `geometry_msgs/Twist` messages to `/cmd_vel` topic.

### `gopigo`
This ROS node subscribes for `/cmd_vel` topic.

When receiving a `geometry_msgs/Twist` message from its topic, this node controls `motor1` and `motor2` to operate gopigo according to the message.

## Requirement

**ROS kinetic**

## Install libraries

```bash
$ /bin/bash update_tools_for_ubuntu.sh
$ pip install -r requirements/common.txt
$ pip install -r requirements/gopigo.txt
```

## How to Run
1. ssh to gopigo and start `roscore`.

    ```bash
    terminal1$ source devel/setup.bash
    terminal1$ roscore
    ```
1. configure `config/params.yaml`

    * set `mqtt.host` and `mqtt.port`
    * If necessary, set `mqtt.cafile`, `mqtt.sername` and `mqtt.password`
1. ssh to gopigo and start `fiware-ros-turtlesim`.

    ```bash
    terminal2$ source devel/setup.bash
    terminal2$ roslaunch fiware-ros-gopigo fiware-ros-gopigo.launch
    ```

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 TIS Inc.
