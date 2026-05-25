#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time
import math
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from .dvl_parser import verify_checksum, parse_wrz, parse_wru, parse_wrp, parse_wrv, parse_wrc, parse_wrw, crc8_func

from dvl_msgs.msg import DVL, DVLDR, DVLBeam
from dvl_msgs.srv import GetConfig
from nav_msgs.msg import Odometry
import tf_transformations

class DvlSerialPythonNode(Node):
    def __init__(self):
        super().__init__('dvl_a50_serial')
        
        self.declare_parameter('port', '/dev/dvl')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame', 'dvl_a50_link')
        self.declare_parameter('speed_of_sound', 1500.0)
        self.declare_parameter('mounting_rotation_offset', 0.0)
        self.declare_parameter('led_enabled', True)
        self.declare_parameter('range_mode', 'auto')
        self.declare_parameter('periodic_cycling_enabled', True)
        self.declare_parameter('enable_on_activate', True)
        self.declare_parameter('topic_velocity', 'velocity')
        self.declare_parameter('topic_dead_reckoning', 'dead_reckoning')
        self.declare_parameter('topic_odometry', 'odometry')
        self.declare_parameter('timeout_configure_ms', 1000)
        self.declare_parameter('timeout_reset_dead_reckoning_ms', 5000)
        self.declare_parameter('timeout_calibrate_gyro_ms', 20000)
        self.declare_parameter('timeout_trigger_ping_ms', 5000)
        self.declare_parameter('timeout_set_protocol_ms', 1000)
        self.declare_parameter('timeout_get_config_ms', 5000)
        
        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.frame = self.get_parameter('frame').value
        self.sos = self.get_parameter('speed_of_sound').value
        self.mounting_offset = self.get_parameter('mounting_rotation_offset').value
        self.led_enabled = self.get_parameter('led_enabled').value
        self.range_mode = self.get_parameter('range_mode').value
        self.periodic_cycling = self.get_parameter('periodic_cycling_enabled').value
        self.enable_on_activate = self.get_parameter('enable_on_activate').value
        
        self.timeout_configure = self.get_parameter('timeout_configure_ms').value / 1000.0
        self.timeout_reset_dr = self.get_parameter('timeout_reset_dead_reckoning_ms').value / 1000.0
        self.timeout_calibrate_gyro = self.get_parameter('timeout_calibrate_gyro_ms').value / 1000.0
        self.timeout_trigger_ping = self.get_parameter('timeout_trigger_ping_ms').value / 1000.0
        self.timeout_set_protocol = self.get_parameter('timeout_set_protocol_ms').value / 1000.0
        self.timeout_get_config = self.get_parameter('timeout_get_config_ms').value / 1000.0
        
        topic_velocity = self.get_parameter('topic_velocity').value
        topic_dead_reckoning = self.get_parameter('topic_dead_reckoning').value
        topic_odometry = self.get_parameter('topic_odometry').value
        
        self.pub = self.create_publisher(DVL, topic_velocity, 10)
        self.dr_pub = self.create_publisher(DVLDR, topic_dead_reckoning, 10)
        self.odom_pub = self.create_publisher(Odometry, topic_odometry, 10)
        
        self.reset_srv = self.create_service(Trigger, 'reset_dead_reckoning', self.reset_dr_callback)
        self.enable_srv = self.create_service(Trigger, 'enable', self.enable_callback)
        self.disable_srv = self.create_service(Trigger, 'disable', self.disable_callback)
        self.calibrate_srv = self.create_service(Trigger, 'calibrate_gyro', self.calibrate_callback)
        self.trigger_ping_srv = self.create_service(Trigger, 'trigger_ping', self.trigger_ping_callback)
        self.get_config_srv = self.create_service(GetConfig, 'get_config', self.get_config_callback)
        
        # State for accumulating transducer beams and odometry
        self.current_beams = [DVLBeam() for _ in range(4)]
        self.odom_msg = Odometry()
        
        self.running = False
        self.get_logger().info(f"Connecting to DVL on {port} at {baud_rate} baud")
        
        success = False
        attempts = 25
        for i in range(attempts):
            self.get_logger().info(f"Attempting to connect to DVL A50 ({i+1}/{attempts})")
            try:
                self.serial = serial.Serial(port, baud_rate, timeout=0.1)
                success = True
                self.get_logger().info("Port opened. Waiting 2 seconds for USB serial chip initialization...")
                time.sleep(2.0)
                break
            except Exception as e:
                self.get_logger().warn(f"Connection attempt failed: {e}")
                time.sleep(1.0)
                
        if not success:
            self.get_logger().error("Serial connection failed.")
            return
            
        self.running = True
        self.wait_for_ack = False
        self.command_success = False
        self.command_event = threading.Event()
        self.wrc_event = threading.Event()
        self.wrc_data = None
        
        # Start reading first so we can catch ACKs during configuration
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()
        
        self.configure_dvl()
        self.add_on_set_parameters_callback(self.parameters_callback)

    def reset_dr_callback(self, request, response):
        self.get_logger().info("Resetting dead reckoning...")
        return self._trigger_command("wcr", self.timeout_reset_dr, response)

    def enable_callback(self, request, response):
        self.get_logger().info("Enabling acoustics...")
        if self.send_configuration(acoustic_enabled=True):
            response.success = True
            response.message = "Acoustics enabled"
        else:
            response.success = False
            response.message = "Failed to enable acoustics"
        return response

    def disable_callback(self, request, response):
        self.get_logger().info("Disabling acoustics...")
        if self.send_configuration(acoustic_enabled=False):
            response.success = True
            response.message = "Acoustics disabled"
        else:
            response.success = False
            response.message = "Failed to disable acoustics"
        return response

    def calibrate_callback(self, request, response):
        self.get_logger().info("Calibrating gyro...")
        return self._trigger_command("wcg", self.timeout_calibrate_gyro, response)

    def trigger_ping_callback(self, request, response):
        self.get_logger().info("Triggering ping...")
        return self._trigger_command("wcx", self.timeout_trigger_ping, response)

    def get_config_callback(self, request, response):
        self.get_logger().info("Fetching configuration from DVL...")
        if hasattr(self, 'serial') and self.serial.is_open:
            self.wrc_event.clear()
            self.wrc_data = None
            self.write_command("wcc")
            if self.wrc_event.wait(self.timeout_get_config) and self.wrc_data:
                response.success = True
                response.speed_of_sound = int(self.wrc_data['speed_of_sound'])
                response.mounting_rotation_offset = int(self.wrc_data['mounting_rotation_offset'])
                response.acoustic_enabled = self.wrc_data['acoustic_enabled']
                response.dark_mode_enabled = self.wrc_data['dark_mode_enabled']
                response.range_mode = self.wrc_data['range_mode']
                response.periodic_cycling_enabled = self.wrc_data['periodic_cycling_enabled']
                response.error_message = ""
            else:
                response.success = False
                response.error_message = "Failed to receive config from DVL"
        else:
            response.success = False
            response.error_message = "Serial port not open"
        return response

    def _trigger_command(self, cmd, timeout, response):
        if hasattr(self, 'serial') and self.serial.is_open:
            if self.send_command_wait_ack(cmd, timeout=timeout):
                response.success = True
                response.message = "Command sent and ACK received"
            else:
                response.success = False
                response.message = f"Command sent but no ACK received within {timeout}s"
        else:
            response.success = False
            response.message = "Serial port not open"
        return response

    def write_command(self, cmd: str):
        cmd = cmd.strip()
        if '*' in cmd:
            cmd = cmd.split('*')[0]
        # Calculate CRC-8 checksum
        crc = crc8_func(cmd.encode('utf-8'))
        full_cmd = f"{cmd}*{crc:02x}\r\n"
        self.get_logger().debug(f"Writing command to port: {full_cmd.strip()}")
        self.serial.write(full_cmd.encode('utf-8'))
        self.serial.flush()

    def send_command_wait_ack(self, cmd: str, timeout: float = 1.0) -> bool:
        self.command_success = False
        self.command_event.clear()
        self.wait_for_ack = True
        self.write_command(cmd)
        
        success = False
        if self.command_event.wait(timeout):
            success = self.command_success
            
        self.wait_for_ack = False
        return success

    def configure_dvl(self):
        self.get_logger().info("Flushing DVL RX buffer...")
        self.serial.write(b"\n\n")
        time.sleep(0.1)
        self.serial.reset_input_buffer()
        
        # Request Version
        self.write_command("wcv")
        time.sleep(0.1)
        
        # Request Product Details
        self.write_command("wcw")
        time.sleep(0.1)
        
        # Stop acoustics first to allow reliable command configuration
        self.get_logger().info("Stopping acoustics initially for configuration...")
        self.send_configuration(False)
        
        attempts = 5
        protocol_set = False
        for i in range(attempts):
            self.get_logger().info(f"Attempting to set DVL serial protocol ({i+1}/{attempts})")
            if self.send_command_wait_ack("wcp,3", timeout=self.timeout_set_protocol):
                protocol_set = True
                break
            time.sleep(0.2)
            
        if not protocol_set:
            self.get_logger().warn("Failed to set DVL protocol to 3. DVL might be ignoring ACKs or using older firmware.")
            
        self.get_logger().info("Sending final Configuration...")
        if self.send_configuration(self.enable_on_activate):
            self.get_logger().info("Configuration sequence complete and ACKed.")
        else:
            self.get_logger().warn("DVL configuration over serial timed out. This is expected if the DVL is wired for one-way telemetry (RX-only). Proceeding with active telemetry streaming!")

    def send_configuration(self, acoustic_enabled) -> bool:
        sos_str = f"{self.sos:g}"
        offset_str = f"{self.mounting_offset:g}"
        acoustic_str = "y" if acoustic_enabled else "n"
        dark_mode_str = "n" if self.led_enabled else "y"
        periodic_str = "y" if self.periodic_cycling else "n"
        range_mode = self.range_mode
        
        cmd1 = f"wcs,{sos_str},{offset_str},{acoustic_str},{dark_mode_str},{range_mode},{periodic_str}"
        cmd2 = f"wcs,{sos_str},{offset_str},{acoustic_str},{dark_mode_str},{range_mode}"
        cmd3 = f"wcs,{sos_str},{offset_str},{acoustic_str},{dark_mode_str}"
        
        config_set = False
        attempts = 20
        for i in range(attempts):
            self.get_logger().info(f"Attempting to configure DVL A50 ({i+1}/{attempts})")
            for cmd in [cmd1, cmd2, cmd3]:
                self.get_logger().debug(f"Sending config command: {cmd.strip()}")
                if self.send_command_wait_ack(cmd, timeout=self.timeout_configure):
                    config_set = True
                    break
                time.sleep(0.1)
                
            if config_set:
                break
            time.sleep(0.2)
            
        return config_set

    def parameters_callback(self, params):
        reconfigure = False
        for param in params:
            if param.name == 'speed_of_sound':
                self.sos = param.value
                reconfigure = True
            elif param.name == 'mounting_rotation_offset':
                self.mounting_offset = param.value
                reconfigure = True
            elif param.name == 'led_enabled':
                self.led_enabled = param.value
                reconfigure = True
            elif param.name == 'range_mode':
                self.range_mode = param.value
                reconfigure = True
            elif param.name == 'periodic_cycling_enabled':
                self.periodic_cycling = param.value
                reconfigure = True
            elif param.name == 'timeout_configure_ms':
                self.timeout_configure = param.value / 1000.0
            elif param.name == 'timeout_reset_dead_reckoning_ms':
                self.timeout_reset_dr = param.value / 1000.0
            elif param.name == 'timeout_calibrate_gyro_ms':
                self.timeout_calibrate_gyro = param.value / 1000.0
            elif param.name == 'timeout_trigger_ping_ms':
                self.timeout_trigger_ping = param.value / 1000.0
            elif param.name == 'timeout_set_protocol_ms':
                self.timeout_set_protocol = param.value / 1000.0
            elif param.name == 'timeout_get_config_ms':
                self.timeout_get_config = param.value / 1000.0
                
        if reconfigure and hasattr(self, 'serial') and self.serial.is_open:
            self.get_logger().info("Dynamic parameter change detected, reconfiguring DVL...")
            threading.Thread(target=self.send_configuration, args=(self.enable_on_activate,)).start()
            
        return SetParametersResult(successful=True)

    def read_loop(self):
        first_velocity = True
        while self.running and rclpy.ok():
            try:
                line = self.serial.readline().decode('utf-8', errors='replace').strip()
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)
                continue
                
            if not line:
                continue
                
            if line.startswith("wr?") or line.startswith("wrn") or line.startswith("wr!"):
                if line.startswith("wrn"):
                    self.get_logger().warn(f"DVL NAK received: {line}")
                elif line.startswith("wr!"):
                    self.get_logger().warn(f"DVL Checksum error received: {line}")
                else:
                    self.get_logger().debug(f"DVL malformed request error received (noise): {line}")
                
                if self.wait_for_ack:
                    self.command_success = False
                    self.command_event.set()
                continue
                
            if not verify_checksum(line):
                self.get_logger().debug(f"CRC Mismatch or malformed message: {line}")
                continue
                
            if line.startswith("wru"):
                data = parse_wru(line)
                if data and 0 <= data['id'] < 4:
                    b = DVLBeam()
                    b.id = data['id']
                    b.velocity = data['velocity']
                    b.distance = data['distance']
                    b.rssi = data['rssi']
                    b.nsd = data['nsd']
                    b.valid = (b.distance != -1.0)
                    self.current_beams[data['id']] = b
                    
            elif line.startswith("wrp"):
                data = parse_wrp(line)
                if data:
                    dr_msg = DVLDR()
                    dr_msg.header.stamp = self.get_clock().now().to_msg()
                    dr_msg.header.frame_id = self.frame
                    dr_msg.time = data['time_stamp']
                    dr_msg.position.x = data['x']
                    dr_msg.position.y = data['y']
                    dr_msg.position.z = data['z']
                    dr_msg.pos_std = data['pos_std']
                    dr_msg.roll = data['roll']
                    dr_msg.pitch = data['pitch']
                    dr_msg.yaw = data['yaw']
                    dr_msg.status = data['status']
                    dr_msg.format = "serial"
                    self.dr_pub.publish(dr_msg)
                    
                    self.odom_msg.header.stamp = dr_msg.header.stamp
                    self.odom_msg.header.frame_id = self.frame
                    self.odom_msg.pose.pose.position.x = data['x']
                    self.odom_msg.pose.pose.position.y = data['y']
                    self.odom_msg.pose.pose.position.z = data['z']
                    
                    # Convert degrees to radians and then to quaternion
                    r = math.radians(data['roll'])
                    p = math.radians(data['pitch'])
                    y = math.radians(data['yaw'])
                    
                    q = tf_transformations.quaternion_from_euler(r, p, y)
                
                    self.odom_msg.pose.pose.orientation.x = q[0]
                    self.odom_msg.pose.pose.orientation.y = q[1]
                    self.odom_msg.pose.pose.orientation.z = q[2]
                    self.odom_msg.pose.pose.orientation.w = q[3]
                    
                    self.odom_pub.publish(self.odom_msg)

            elif line.startswith("wrz"):
                if first_velocity:
                    self.get_logger().info("First velocity report (wrz) received! Data is streaming correctly.")
                    first_velocity = False
                data = parse_wrz(line)
                if data:
                    msg = DVL()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame
                    msg.time = data['time']
                    msg.velocity.x = data['vx']
                    msg.velocity.y = data['vy']
                    msg.velocity.z = data['vz']
                    if data['altitude'] >= 0.0 and data['valid']:
                        msg.altitude = data['altitude']
                    msg.velocity_valid = data['valid']
                    msg.fom = data['fom']
                    msg.status = data['status']
                    msg.time_of_validity = data['time_of_validity']
                    msg.time_of_transmission = data['time_of_transmission']
                    msg.form = "serial"
                    if len(data['covariance']) == 9:
                        msg.covariance = data['covariance']
                        
                        # Populate odometry twist
                        self.odom_msg.header.stamp = msg.header.stamp
                        self.odom_msg.header.frame_id = self.frame
                        self.odom_msg.twist.twist.linear.x = data['vx']
                        self.odom_msg.twist.twist.linear.y = data['vy']
                        self.odom_msg.twist.twist.linear.z = data['vz']
                        
                        # Safe list assignment for covariance
                        cov = list(self.odom_msg.twist.covariance)
                        for i in range(3):
                            for j in range(3):
                                cov[i*6 + j] = data['covariance'][i*3 + j]
                        self.odom_msg.twist.covariance = cov
                        
                    msg.beams = self.current_beams
                    self.pub.publish(msg)
                else:
                    self.get_logger().warn(f"Failed to parse wrz sentence: {line}")

            elif line.startswith("wrv"):
                data = parse_wrv(line)
                if data:
                    self.get_logger().info(f"DVL Connected. Part Number: {data['part_number']}, Firmware Version: {data['version']}")
                else:
                    self.get_logger().warn(f"Failed to parse wrv sentence: {line}")
                    
            elif line.startswith("wrw"):
                data = parse_wrw(line)
                if data:
                    self.get_logger().info(f"DVL Product Info - Name: {data['name']}, Version: {data['version']}, ChipID: {data['chip_id']}, IP: {data['ip_address']}")
                else:
                    self.get_logger().warn(f"Failed to parse wrw sentence: {line}")
                    
            elif line.startswith("wrc"):
                data = parse_wrc(line)
                if data:
                    self.wrc_data = data
                    self.get_logger().info(f"DVL Configuration Received: {data}")
                    self.wrc_event.set()
                self.get_logger().info(f"DVL Configuration ACK: {line}")
                if self.wait_for_ack:
                    self.command_success = True
                    self.command_event.set()

            elif line.startswith("wra"):
                self.get_logger().info(f"DVL Configuration ACK: {line}")
                if self.wait_for_ack:
                    self.command_success = True
                    self.command_event.set()
                
            else:
                # Log unhandled valid messages at DEBUG level so they don't spam the terminal
                self.get_logger().debug(f"Unhandled message type: {line}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join()
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DvlSerialPythonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
