# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
import tf2_ros
import serial
import math
import threading

HEAD1 = 0xAA
HEAD2 = 0x55
SEND_VELOCITY    = 0x11
SEND_PARAMS      = 0x12
SEND_COEFFICIENT = 0x13

# ---------------------------------------------------------------------------
# CALIBRATION
# ---------------------------------------------------------------------------
# ENCODER_SCALE: tune empirically.
#   Drive 1 meter, check odom.x. If reads 0.5 -> double the scale.
#
# CALIBRATION_SAMPLES: number of IMU samples to average for gyro bias.
#   100 samples at 50Hz = ~2 seconds. Robot MUST be stationary during this.
# ---------------------------------------------------------------------------
ENCODER_SCALE = 0.001 * 10
CALIBRATION_SAMPLES = 100

# YAW_TRIM: counteracts mechanical drift when driving straight.
# Positive = left correction (use if robot drifts right).
# Tune in steps of 0.05 until robot drives straight.
YAW_TRIM = 0.0145


def checksum(buf):
    return sum(buf) & 0xFF


def build_velocity_packet(x, y, yaw):
    x_int   = int(x   * 1000) & 0xFFFF
    y_int   = int(y   * 1000) & 0xFFFF
    yaw_int = int(yaw * 1000) & 0xFFFF
    tmp = [
        HEAD1, HEAD2, 0x0B, SEND_VELOCITY,
        (x_int   >> 8) & 0xFF, x_int   & 0xFF,
        (y_int   >> 8) & 0xFF, y_int   & 0xFF,
        (yaw_int >> 8) & 0xFF, yaw_int & 0xFF,
    ]
    tmp.append(checksum(tmp))
    return bytes(tmp)


def int16(high, low):
    val = (high << 8) | low
    if val > 32767:
        val -= 65536
    return val


class JetRacerDriver(Node):
    def __init__(self):
        super().__init__('jetracer_driver')

        # Serial port (override with -p port:=/dev/ttyACMx if needed)
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.get_logger().info('Opened base controller on {}'.format(port))

        # Commanded velocity (from /cmd_vel)
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Python-computed odom state (no longer used - EKF owns pose)
        self.computed_yaw = 0.0      # kept for compatibility, unused

        # Gyro bias calibration state
        self.gz_bias = 0.0
        self.gz_samples = []
        self.gz_calibrated = False

        # Publishers
        self.odom_pub  = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub   = self.create_publisher(Imu, 'imu', 10)
        self.lvel_pub  = self.create_publisher(Int32, 'motor/lvel', 10)
        self.rvel_pub  = self.create_publisher(Int32, 'motor/rvel', 10)
        self.lset_pub  = self.create_publisher(Int32, 'motor/lset', 10)
        self.rset_pub  = self.create_publisher(Int32, 'motor/rset', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Timer to send velocity at 50Hz
        self.timer = self.create_timer(0.02, self.send_velocity)

        # Serial receive thread
        self.recv_thread = threading.Thread(
            target=self.serial_receive_task, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(
            'JetRacer driver starting. Keep robot STATIONARY for ~2 seconds '
            'while gyro bias calibrates...')

    def cmd_vel_callback(self, msg):
        self.x   = msg.linear.x
        self.y   = msg.linear.y
        self.yaw = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def send_velocity(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
        # Apply yaw trim only when moving forward to counteract mechanical drift
        trim = YAW_TRIM if self.x > 0.0 else 0.0
        pkt = build_velocity_packet(self.x, self.y, self.yaw + trim)
        self.ser.write(pkt)

    def serial_receive_task(self):
        (HEAD1_S, HEAD2_S, SIZE_S, DATA_S, CSUM_S) = range(5)
        state = HEAD1_S
        data  = bytearray(50)
        frame_size = 0
        last_time = self.get_clock().now()

        while True:
            try:
                if state == HEAD1_S:
                    b = self.ser.read(1)
                    if not b:
                        continue
                    data[0] = b[0]
                    state = HEAD2_S if data[0] == HEAD1 else HEAD1_S

                elif state == HEAD2_S:
                    b = self.ser.read(1)
                    if not b:
                        continue
                    data[1] = b[0]
                    state = SIZE_S if data[1] == HEAD2 else HEAD1_S

                elif state == SIZE_S:
                    b = self.ser.read(1)
                    if not b:
                        continue
                    data[2] = b[0]
                    frame_size = data[2]
                    state = DATA_S

                elif state == DATA_S:
                    chunk = self.ser.read(frame_size - 4)
                    if len(chunk) < frame_size - 4:
                        state = HEAD1_S
                        continue
                    for i, v in enumerate(chunk):
                        data[3 + i] = v
                    state = CSUM_S

                elif state == CSUM_S:
                    b = self.ser.read(1)
                    if not b:
                        continue
                    data[frame_size - 1] = b[0]
                    calc = checksum(data[:frame_size - 1])
                    if data[frame_size - 1] == calc:
                        now = self.get_clock().now()
                        self.handle_frame(data, now, last_time)
                        last_time = now
                    state = HEAD1_S

            except Exception as e:
                self.get_logger().warn('Serial error: {}'.format(e))
                state = HEAD1_S

    def handle_frame(self, data, now, last_time):
        # ── PARSE IMU ─────────────────────────────────────────────
        gx = int16(data[4],  data[5])  / 32768.0 * 2000 / 180 * math.pi
        gy = int16(data[6],  data[7])  / 32768.0 * 2000 / 180 * math.pi
        gz = int16(data[8],  data[9])  / 32768.0 * 2000 / 180 * math.pi
        ax = int16(data[10], data[11]) / 32768.0 * 2 * 9.8
        ay = int16(data[12], data[13]) / 32768.0 * 2 * 9.8
        az = int16(data[14], data[15]) / 32768.0 * 2 * 9.8

        # ── GYRO BIAS CALIBRATION ─────────────────────────────────
        # During first ~2 seconds, average gz to find bias.
        # Robot MUST be stationary during this period.
        if not self.gz_calibrated:
            self.gz_samples.append(gz)
            if len(self.gz_samples) >= CALIBRATION_SAMPLES:
                self.gz_bias = sum(self.gz_samples) / len(self.gz_samples)
                self.gz_calibrated = True
                self.get_logger().info(
                    'Gyro calibrated. Bias: {:.6f} rad/s. Ready to drive.'
                    .format(self.gz_bias))
            return   # skip publishing until calibrated

        # Apply bias correction
        gz_corrected = gz - self.gz_bias

        # ── TIME STEP ─────────────────────────────────────────────
        # Use fixed dt matching firmware sample rate (50Hz) rather than
        # measured serial-frame timing, which includes USB jitter.
        dt = 0.02

        # NOTE: We no longer integrate yaw here. The EKF integrates the
        # gyro yaw rate itself from /imu. Integrating here too would be
        # double-integration and the two estimates would fight.

        # ── PUBLISH IMU (using bias-corrected gz, computed yaw) ───
        imu = Imu()
        imu.header.stamp    = now.to_msg()
        imu.header.frame_id = 'base_imu_link'
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz_corrected
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        # Orientation not provided - let EKF integrate yaw from gyro
        imu.orientation_covariance[0]   = -1.0  # convention: orientation not provided
        imu.angular_velocity_covariance = [float(x) for x in [0.01,0,0, 0,0.01,0, 0,0,0.01]]
        imu.linear_acceleration_covariance = [float(x) for x in [0.01,0,0, 0,0.01,0, 0,0,0.01]]
        self.imu_pub.publish(imu)

        # ── MOTOR FEEDBACK ────────────────────────────────────────
        lvel_raw = int16(data[34], data[35])
        rvel_raw = int16(data[36], data[37])
        lset_raw = int16(data[38], data[39])
        rset_raw = int16(data[40], data[41])

        # ── WHEEL VELOCITY (no pose integration here) ─────────────
        v_left  = lvel_raw * ENCODER_SCALE
        v_right = rvel_raw * ENCODER_SCALE
        v_linear = (v_left + v_right) / 2.0

        # TF odom->base_footprint is published by EKF node, not here.

        # ── Odometry message (VELOCITIES ONLY) ────────────────────
        # We publish twist (vx, vyaw) and let the EKF integrate it into
        # a pose. Pose fields are left at zero with huge covariance so
        # the EKF ignores them and relies on the velocities + IMU.
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        # Pose left at identity, marked as untrusted (1e6) so EKF ignores it
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance  = [float(x) for x in [1e6,0,0,0,0,0, 0,1e6,0,0,0,0,
                                  0,0,1e6,0,0,0,  0,0,0,1e6,0,0,
                                  0,0,0,0,1e6,0,  0,0,0,0,0,1e6]]

        # Twist is what the EKF actually uses from this source
        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.angular.z = gz_corrected
        odom.twist.covariance = [float(x) for x in [1e-3,0,0,0,0,0, 0,1e6,0,0,0,0,
                                  0,0,1e6,0,0,0,  0,0,0,1e6,0,0,
                                  0,0,0,0,1e6,0,  0,0,0,0,0,1e-2]]
        self.odom_pub.publish(odom)

        # ── MOTOR FEEDBACK PUBLISH ────────────────────────────────
        lv = Int32(); lv.data = lvel_raw
        rv = Int32(); rv.data = rvel_raw
        ls = Int32(); ls.data = lset_raw
        rs = Int32(); rs.data = rset_raw
        self.lvel_pub.publish(lv)
        self.rvel_pub.publish(rv)
        self.lset_pub.publish(ls)
        self.rset_pub.publish(rs)


def main(args=None):
    rclpy.init(args=args)
    node = JetRacerDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
