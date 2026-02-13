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

        self.aruco = ArucoMarker(sn = self.serial_number,
                                 aruco_type = self.aruco_type,
                                 aruco_marker_side_length = self.aruco_size,
                                 scale = self.aruco_scale)

        # For the calibration process
        self.declare_parameter("calibrate_camera", 0)
        self.last_photo_time = time.time()
        self.countdown_logged = set()
        self.image_interval = 6  # seconds

        self.calibrate = False
        if self.get_parameter("calibrate_camera").value == 1:
            self.calibrate = True

        self.num_images = 20
        self.image_count = 0     
        if self.calibrate:
            self.get_logger().warn(f"Calibration started. Be ready to place (9x6) chessboard for taking photos.")
        else:
            self.aruco.load_calibration()
            self.get_logger().info(f"Calibration files loaded.")

        # use H.264 image format, compressed, lighter in bandwidth and faster refresh rate and same quality
        self.use_h264 = True
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
        try:
            # Parse the incoming data into packets
            packets = self.decoder.parse(msg.data)
            for packet in packets:
                try:
                    frames = self.decoder.decode(packet)
                    for frame in frames:
                        img = frame.to_ndarray(format='bgr24')

                        #######################################
                        # Calibration and ArUco detection logic
                        if self.calibrate:
                            self.take_photo(img, self.use_h264)
                            if self.image_count == self.num_images:
                                self.aruco.camera_calibration()
                                self.aruco.load_calibration()
                                self.get_logger().info("Calibration complete. Files loaded.")
                                self.calibrate = False
                        else:
                            self.detect_arucos(img)
                        #######################################

                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            rclpy.shutdown()

                except av.error.InvalidDataError as decode_err:
                    self.get_logger().warn(f"Invalid data during decode: {decode_err}")
                    continue

        except av.AVError as parse_err:
            self.get_logger().warn(f"Error parsing packet: {parse_err}")


    ########################################################################################

    def color_image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV format
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            #######################################
            if self.calibrate:
                self.take_photo(img, self.use_h264)
                if self.image_count == self.num_images:
                    self.aruco.camera_calibration()
                    self.aruco.load_calibration()
                    self.get_logger().info(f"Calibration complete. Files loaded.")
                    self.calibrate = False
            else:
                self.detect_arucos(img)
            #######################################

        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")

    ########################################################################################

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
        frame = self.aruco.pose_estimation(img)
        cv2.imwrite("aruco_corner_frame.jpg", frame)

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
