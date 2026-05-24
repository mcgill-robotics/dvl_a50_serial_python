# dvl_a50_serial_python

A ROS 2 driver for the Water Linked DVL A50, written in Python and communicating over a serial connection. 

This package parses velocity, transducer, and dead reckoning data from the DVL hardware, translating it into standard ROS 2 topics and service handlers.

---

## Features

- Parses DVL protocols: modern velocity reports (`wrz`), transducer metrics (`wru`), dead reckoning (`wrp`), version metadata (`wrv`), and configurations (`wrc`).
- Verifies checksums on incoming messages using a standard CRC-8 calculation.
- Supports dynamic parameter updates at runtime for device configuration.
- Provides standard service interfaces to calibrate the device, toggle acoustics, trigger pings, reset dead reckoning, and query configuration.

---

## Configuration Parameters

All parameters can be configured via YAML parameters or overridden at runtime:

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `port` | `string` | `"/dev/dvl"` | Serial device path. |
| `baud_rate` | `int` | `115200` | Baud rate of the serial connection. |
| `frame` | `string` | `"dvl_a50_link"` | Header frame ID for published messages. |
| `speed_of_sound` | `double` | `1500.0` | Target speed of sound in water (m/s). |
| `mounting_rotation_offset` | `double` | `0.0` | Mounting rotation offset (degrees). |
| `led_enabled` | `bool` | `true` | Enable or disable physical LEDs on the device. |
| `range_mode` | `string` | `"auto"` | Altitude search range specifier: `"auto"`, `"=a"` (locked to mode 0-4), `"a<=b"` (search modes `a` through `b`), or `"wt"` (water tracking). |
| `periodic_cycling_enabled` | `bool` | `true` | Toggle periodic cycling modes. |
| `enable_on_activate` | `bool` | `true` | Auto-start acoustic pinging when the node starts. |
| `topic_velocity` | `string` | `"velocity"` | Topic name for DVL velocity data. |
| `topic_dead_reckoning` | `string` | `"dead_reckoning"` | Topic name for DVL dead reckoning data. |
| `topic_odometry` | `string` | `"odometry"` | Topic name for standard Odometry message outputs. |
| `timeout_configure_ms` | `int` | `5000` | Timeout limit for sending configurations. |
| `timeout_reset_dead_reckoning_ms` | `int` | `5000` | Timeout limit for resetting dead reckoning. |
| `timeout_calibrate_gyro_ms` | `int` | `20000` | Timeout limit for persistently calibrating the gyroscope. |
| `timeout_trigger_ping_ms` | `int` | `5000` | Timeout limit for manual ping triggers. |
| `timeout_set_protocol_ms` | `int` | `5000` | Timeout limit for setting the communication protocol. |
| `timeout_get_config_ms` | `int` | `5000` | Timeout limit for retrieving configurations. |

---

## ROS Interfaces

### Published Topics

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| **`velocity`** | `dvl_msgs/msg/DVL` | Measured linear velocities, calculated FOM, altitude, status, and beam-by-beam metrics. |
| **`dead_reckoning`** | `dvl_msgs/msg/DVLDR` | Integrated 3D coordinates, orientation (Euler), status, and standard deviation. |
| **`odometry`** | `nav_msgs/msg/Odometry` | Fused robot state estimation: velocities and covariance are derived from `wrz`, while position and orientation are derived from `wrp`. |

### Services

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| **`enable`** | `std_srvs/srv/Trigger` | Activates acoustic pings on the device. |
| **`disable`** | `std_srvs/srv/Trigger` | Deactivates acoustic pings on the device. |
| **`calibrate_gyro`** | `std_srvs/srv/Trigger` | Starts the persistent gyroscope calibration sequence (`wcg\n`). |
| **`reset_dead_reckoning`** | `std_srvs/srv/Trigger` | Resets the DVL's integrated coordinate frame back to zero (`wcr\n`). |
| **`trigger_ping`** | `std_srvs/srv/Trigger` | Triggers a single manual acoustic ping (`wct\n`). |
| **`get_config`** | `dvl_msgs/srv/GetConfig` | Queries the active operational parameters from DVL non-volatile memory. |

---

## Build and Run

### Build the Package
From your ROS 2 workspace root directory:
```bash
colcon build --packages-select dvl_a50_serial_python
```

### Launch the Driver
Run the driver using the default YAML parameters under the `/dvl` namespace:
```bash
ros2 launch dvl_a50_serial_python dvl_a50_serial_python.launch.py
```
