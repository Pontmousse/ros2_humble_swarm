import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


import os, cv2, av, time, numpy as np
from cv_bridge import CvBridge

from robomaster_msgs.msg import H264Packet
from sensor_msgs.msg import Image
from swarm_interfaces.msg import CoordXY, Markers

from swarm_aruco.aruco import ArucoMarker

class ArucoDetectorNode(Node):
    def __init__(self):
        # Start ArucoMarker class instance
        super().__init__('aruco_detector')  
        self.get_logger().info(f"Aruco_detector has been started.")
        self.markers = []
        self.markers_normals = []

        # Parameters
        self.declare_parameter("serial_number", 'sn')
        self.declare_parameter("aruco_type", 'DICT_5X5_100')
        self.declare_parameter("aruco_size", 2.5)  # in cm
        self.declare_parameter("aruco_scale", (1.5, 0.8))

        self.aruco_type = self.get_parameter("aruco_type").value
        self.aruco_size = self.get_parameter("aruco_size").value
        self.serial_number = self.get_parameter("serial_number").value
        self.aruco_scale = self.get_parameter("aruco_scale").value

        # # # # # # # # # # # # # # # # # # # # # # # # # # # 
        # For QoS
        self.declare_parameter('qos_depth', 10)
        self.declare_parameter('qos_reliability', 'RELIABLE')
        self.declare_parameter('qos_history', 'KEEP_LAST')

        qos_depth = self.get_parameter('qos_depth').value
        qos_reliability = self.get_parameter('qos_reliability').value
        qos_history = self.get_parameter('qos_history').value

        # Validate QoS settings       
        valid_reliabilities = ['RELIABLE', 'BEST_EFFORT']
        valid_histories = ['KEEP_LAST', 'KEEP_ALL']

        if qos_reliability not in valid_reliabilities:
            raise ValueError(f"Invalid qos_reliability: {qos_reliability}")
        if qos_history not in valid_histories:
            raise ValueError(f"Invalid qos_history: {qos_history}")

        # QoS Settings 
        self.qos_profile_reliable = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.qos_profile_best_effort = QoSProfile(
            depth=qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        # # # # # # # # # # # # # # # # # # # # # # # # # # # 


        # For timer
        self.declare_parameter("timer_frequency", 0.01)
        self.timer_frequency = self.get_parameter("timer_frequency").value

        # Detection-rate throttle. ArUco pose estimation (solvePnP) is the single
        # biggest CPU consumer on the Pi (~116% in profiling), running once per
        # incoming camera frame. Cap it to detect_max_hz: frames arriving faster
        # than this are dropped BEFORE cv_bridge conversion + detection, so we
        # skip the whole decode-image->gray->solvePnP cost on skipped frames.
        # <= 0 disables the throttle (process every frame).
        self.declare_parameter("detect_max_hz", 5.0)
        detect_max_hz = float(self.get_parameter("detect_max_hz").value)
        self.detect_min_interval = (1.0 / detect_max_hz) if detect_max_hz > 0.0 else 0.0
        self.last_detect_time = 0.0
        self.get_logger().info(
            f"Detection throttle: detect_max_hz={detect_max_hz} "
            f"(min interval {self.detect_min_interval*1000:.0f} ms)")

        self.aruco = ArucoMarker(sn = self.serial_number,
                                 aruco_type = self.aruco_type,
                                 aruco_marker_side_length = self.aruco_size,
                                 scale = self.aruco_scale)

        # For the calibration process
        self.declare_parameter("calibrate_camera", 0)
        self.declare_parameter("strict_calibration", 1)
        self.last_photo_time = time.time()
        self.countdown_logged = set()
        self.image_interval = 6  # seconds

        self.calibrate = False
        if self.get_parameter("calibrate_camera").value == 1:
            self.calibrate = True

        self.num_images = 20
        self.image_count = 0     
        if self.calibrate:
            # camera_calibration() globs '*.jpg', so photos left over from a
            # previous session would be silently mixed into the new solve --
            # which is how a fresh calibration can inherit the exact bias it
            # was meant to remove. Clear them first.
            self._clear_old_photos()
            self.get_logger().warn(f"Calibration started. Be ready to place (9x6) chessboard for taking photos.")
        else:
            # Reject implausible intrinsics at startup rather than silently
            # producing mis-scaled poses. Set strict_calibration:=0 to run with
            # a known-bad calibration (it only warns), e.g. while working
            # through re-calibrating the fleet.
            K, _ = self.aruco.load_calibration(
                strict=bool(self.get_parameter("strict_calibration").value))
            self.get_logger().info(
                f"Calibration loaded for {self.serial_number}: "
                f"fx={K[0][0]:.1f} fy={K[1][1]:.1f} "
                f"cx={K[0][2]:.1f} cy={K[1][2]:.1f}")

        # use H.264 image format, compressed, lighter in bandwidth and faster refresh rate and same quality
        self.use_h264 = False
        self.get_logger().info(f"Using H.264 video stream: {self.use_h264}")

        # Subscribe to video stream
        if self.use_h264 :
            self.decoder = av.codec.CodecContext.create('h264', 'r')
            self.create_subscription(H264Packet, 'camera/image_h264', self.h264_image_callback, self.qos_profile_best_effort)
        else:
            self.bridge = CvBridge()
            self.create_subscription(Image, 'camera/image_color', self.color_image_callback, self.qos_profile_best_effort)
        
        # Publish detected aruco markers
        self.marker_pub = self.create_publisher(Markers, 'markers_unf', self.qos_profile_reliable)
        self.create_timer(self.timer_frequency, self.publish_markers)


    ########################################################################################

    # def h264_image_callback(self, msg: H264Packet):
    #     try:
    #         packets = self.decoder.parse(msg.data)
    #         for packet in packets:
    #             frames = self.decoder.decode(packet)
    #             for frame in frames:
    #                 img = frame.to_ndarray(format='bgr24')

    #                 #######################################
    #                 if self.calibrate:
    #                     self.take_photo(img, self.use_h264)
    #                     if self.image_count == self.num_images:
    #                         self.aruco.camera_calibration()
    #                         self.aruco.load_calibration()
    #                         self.get_logger().info(f"Calibration complete. Files loaded.")
    #                         self.calibrate = False
    #                 else:
    #                     self.detect_arucos(img)
    #                 #######################################

    #                 if cv2.waitKey(1) & 0xFF == ord('q'):
    #                     rclpy.shutdown()
    #     except av.OSError as e:
    #         self.get_logger().warn(f"H264 decode error: {e}")

    ##############################################
    ##############################################

    def h264_image_callback(self, msg: H264Packet):
        # Throttle detection (same as color path). Note: H.264 must still be
        # parsed/decoded to advance the stream; we only gate the per-frame
        # ArUco detection work via detect_arucos below.
        now = time.time()
        throttle_detect = not (self.detect_min_interval > 0.0 and
                               (now - self.last_detect_time) < self.detect_min_interval)
        if throttle_detect:
            self.last_detect_time = now
        try:
            # Parse the incoming data into packets.
            # msg.data arrives as array.array('B'); parse() expects bytes.
            packets = self.decoder.parse(bytes(msg.data))
            for packet in packets:
                try:
                    frames = self.decoder.decode(packet)
                    for frame in frames:
                        img = frame.to_ndarray(format='bgr24')

                        #######################################
                        # Calibration and ArUco detection logic
                        if self.calibrate:
                            self.take_photo(img, self.use_h264)
                            # See color_image_callback for why this is `>=`
                            # and why self.calibrate is cleared up front.
                            if self.image_count >= self.num_images:
                                self.calibrate = False
                                try:
                                    self.aruco.camera_calibration()
                                    self.aruco.load_calibration(
                                        strict=bool(self.get_parameter(
                                            "strict_calibration").value))
                                    self.get_logger().info("Calibration complete. Files loaded.")
                                except Exception as cal_err:
                                    self.get_logger().error(
                                        f"CALIBRATION FAILED: {cal_err}")
                                    self.get_logger().error(
                                        "Photo capture stopped. Fix the issue "
                                        "and relaunch with CALIBRATE_CAMERA=1.")
                        elif throttle_detect:
                            self.detect_arucos(img)
                        #######################################

                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            rclpy.shutdown()

                except av.error.InvalidDataError as decode_err:
                    self.get_logger().warn(f"Invalid data during decode: {decode_err}")
                    continue

        except av.FFmpegError as parse_err:
            self.get_logger().warn(f"Error parsing packet: {parse_err}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected error in h264 callback: {e}")


    ########################################################################################

    def color_image_callback(self, msg: Image):
        # Throttle detection: drop frames arriving faster than detect_max_hz
        # BEFORE any conversion/detection work. publish_markers keeps republishing
        # the last stored markers on its own timer, so downstream rate is unaffected.
        now = time.time()
        if self.detect_min_interval > 0.0 and (now - self.last_detect_time) < self.detect_min_interval:
            return
        self.last_detect_time = now
        try:
            # Convert ROS Image to OpenCV format. The decoder now publishes
            # mono8 (single-channel gray); ArUco detection runs on gray anyway,
            # so we take it directly and skip a redundant BGR<->gray round-trip.
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            #######################################
            if self.calibrate:
                self.take_photo(img, self.use_h264)
                # `>=` not `==`: if the calibration attempt below fails, the
                # count keeps rising and an equality test can never fire again,
                # leaving the node snapping photos forever.
                if self.image_count >= self.num_images:
                    # Stop capturing BEFORE attempting the calibration, so a
                    # rejected calibration ends the session instead of looping.
                    self.calibrate = False
                    try:
                        self.aruco.camera_calibration()
                        self.aruco.load_calibration(
                            strict=bool(self.get_parameter(
                                "strict_calibration").value))
                        self.get_logger().info("Calibration complete. Files loaded.")
                    except Exception as cal_err:
                        # Must be reported here: the outer handler would label
                        # this "Failed to convert image", which it is not.
                        self.get_logger().error(
                            f"CALIBRATION FAILED: {cal_err}")
                        self.get_logger().error(
                            "Photo capture stopped. Fix the issue and relaunch "
                            "with CALIBRATE_CAMERA=1 to retry.")
            else:
                self.detect_arucos(img)
            #######################################

        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")

    ########################################################################################

    def _clear_old_photos(self):
        '''
        Move any existing calibration photos into a timestamped subfolder.

        camera_calibration() does glob('<sn>/*.jpg'), which is NOT recursive,
        so archiving into a subdirectory removes them from the solve while
        keeping them on disk -- deliberately not deleting, since some of these
        are symlinks into the git-tracked src/ tree.
        '''
        folder = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'calibration',
            self.serial_number
        )
        if not os.path.isdir(folder):
            self.get_logger().error(
                f"Calibration folder does not exist: {folder}. "
                f"cv2.imwrite will fail SILENTLY -- create it before continuing.")
            return

        stale = [f for f in os.listdir(folder) if f.endswith('.jpg')]
        if not stale:
            return

        archive = os.path.join(folder, f"archive_{int(time.time())}")
        os.makedirs(archive, exist_ok=True)
        for f in stale:
            try:
                os.rename(os.path.join(folder, f), os.path.join(archive, f))
            except OSError as err:
                self.get_logger().warn(f"Could not archive {f}: {err}")

        self.get_logger().warn(
            f"Archived {len(stale)} pre-existing photo(s) into "
            f"{os.path.basename(archive)}/ so they do not contaminate this "
            f"calibration.")


    def take_photo(self, img, use_h264):
        current_time = time.time()
        time_elapsed = current_time - self.last_photo_time
        countdown = self.image_interval - int(time_elapsed)

        # Log countdown once per second
        if 0 < countdown <= self.image_interval and countdown not in self.countdown_logged:
            self.get_logger().warn(f"{countdown}...")
            self.countdown_logged.add(countdown)

        if time_elapsed >= self.image_interval:
            tag = 'h264_' if use_h264 else ''
            self.image_count += 1

            name = f"{self.serial_number}/{tag}photo_{self.image_count}.jpg"
            path = os.path.join(
                get_package_share_directory('swarm_bringup'),
                'calibration',
                name
            )

            cv2.imwrite(path, img)
            self.get_logger().info(f"Photo {self.image_count}/{self.num_images} taken.")

            self.last_photo_time = current_time
            self.countdown_logged.clear()



    ########################################################################################

    def detect_arucos(self, img):
        self.aruco.pose_estimation(img)

        tvecs = self.aruco.tvecs
        rvecs = self.aruco.rvecs

        p = []  # marker positions
        q = []  # marker orientation normals

        for i in range(len(tvecs)):
            # Save marker position
            land = CoordXY()
            land.x = tvecs[i, 0]
            land.y = tvecs[i, 1]
            p.append(land)

            # Convert rotation vector to rotation matrix
            rvec = np.array(rvecs[i], dtype=np.float32)
            rot_matrix, _ = cv2.Rodrigues(rvec)

            # Get the marker's z-axis in camera frame
            z_axis = rot_matrix[:, 2]

            # Ensure z-marker is aligned with z of the camera
            if z_axis[2] < 0:
                z_axis *= -1

            # Project the marker z-axis to the robot/camera motion plane (x-z) and normalize
            projected_vec = np.array([z_axis[0], z_axis[2]])
            norm = np.linalg.norm(projected_vec)
            if norm > 1e-6:
                projected_vec /= norm

            # Save the normalized direction
            orien = CoordXY()
            orien.x = float(projected_vec[0])
            orien.y = float(projected_vec[1])
            q.append(orien)

        self.markers = p
        self.markers_normals = q


    ########################################################################################

    def publish_markers(self):
        msg = Markers()
        msg.markers = self.markers
        msg.markers_normals = self.markers_normals
        self.marker_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
