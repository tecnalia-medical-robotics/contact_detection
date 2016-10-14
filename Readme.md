# Description
This package is dealing with the detection of a contact with a force sensor.
This is a preliminary version, in which the non-contact state is defined by the standard deviation of the signal recorded during a given period of time. Then if the signal goes beyound k standard deviations, a contact is detected.

# instructions

## Launch

We assume is force information is being published.

### Basic example

```bash
rosrun contact_detection contact_detection_node.py wrench:=/optoforce_node/wrench_IRE0A004
```

This program is just printing on the screen when a contact is perceived

### Using action mechanisms
The following command is lauching the server
```bash
rosrun contact_detection contact_detection_action_server.py wrench:=/optoforce_node/wrench_IRE0A004
```

An illustration of client is present in file script/contact_detection_action_client.py

```bash
rosrun contact_detection contact_detection_action_client_node.py
```

# Pending improvements
The current setup is currently designed to detect the initial contact. It is not designed yet for detecting when the sensor reading is stabilized.

Another improvement would be to enable some parameter definition either from the action goal or from the ros param mechanism.

