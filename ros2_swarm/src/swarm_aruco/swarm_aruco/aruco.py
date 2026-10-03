import random, os, glob, cv2, numpy as np
from ament_index_python.packages import get_package_share_directory

__all__ = ["ArucoMarker"]


class ArucoMarker():
    '''
    This class includes multiple different operations related aruco markers:
    generate_marker - generate .png files containing specified aruco marker in the folder 'marker' in workspace
    take_photo - need to connect to a robot through wifi; then, use the camera to take photos for calibration purpose
    camera_calibration - using the photo in folder 'calibration/sn' to get the parameters for the camera and calibrate the image
    detect_initialize - obtain parameters from calibration file
    pose_estimate - given frames of image, find the relative position of aruco markers
    '''

    # Minimum number of photos with a successfully detected chessboard before
    # cv2.calibrateCamera is allowed to produce a calibration we keep.
    MIN_CALIBRATION_VIEWS = 8

    # Reference intrinsics the RoboMaster driver itself publishes on
    # camera/camera_info at 640x360 (robomaster_ros/config/
    # rm_camera_calibration_360.yaml). Same lens on every robot, so a
    # self-calibration that disagrees wildly is wrong, not "unit variation".
    REFERENCE_FX_AT_640 = 317.03

    # Accepted band around the reference, as a fraction. 0.25 lets genuine
    # unit-to-unit differences and focus spread through while catching the
    # ~0.5x failures.
    FX_TOLERANCE = 0.25

    # How far the principal point may sit from the image centre, as a fraction
    # of image width/height.
    PRINCIPAL_POINT_TOLERANCE = 0.15

    def __init__(self, sn, aruco_type="DICT_5X5_100", aruco_marker_side_length = 2.5, scale = (1.5, 0.80)):
        '''
            2.5 cm - > DICT_5X5_100  - >  scale = (1.5, 0.80)
            4.3 cm - > DICT_6X6_250  - >  scale = (1.0, 1.0)

            sn = the SN number of the robot detecting the markers
            aruco_type = the type of aruco marker used
            aruco_marker_side_length = the length of the printed aruco marker to be detected
            max_marker =  maximum number of markers a camera can detect
        '''
        self.aruco_type = aruco_type
        self.marker_size = int(aruco_type.split("_")[-1])
        self.max_marker = 25
        self.aruco_marker_side_length = aruco_marker_side_length

        self.ARUCO_DICT =  {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
        "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
        "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
        "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
        "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
        "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
        "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
        "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
        "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
        "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
        "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
        "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
        "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
        "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        } 

        self.sn = sn
        self.marker_num = 0

        self.tvecs = None
        self.rvecs = None
        self.scalex = scale[0]
        self.scaley = scale[1]

        self.CameraMatrix = []
        self.DistortionMatrix = []

        # Build the ArUco detector once and reuse it. pose_estimation() used to
        # rebuild the dictionary, the DetectorParameters and the ArucoDetector
        # object on every single frame (per detector node) -- pure per-frame
        # overhead. The dictionary is fixed for the node's lifetime.
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_type]),
            cv2.aruco.DetectorParameters())



    def generate_markers(self, num = 10):
        '''
        generate .png files containing specified aruco marker in the folder 'marker' in workspace
        num = number of markers generated
        '''

        arucoDict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_type])

        # create/load directory to store the marker
        offset = 0
        os.makedirs("markers", exist_ok=True)
        try:
            os.mkdir("markers/" + self.aruco_type)
        except FileExistsError:
            offset = len(os.listdir('markers/' + self.aruco_type))

        # generate marker
        for id in range(offset, num+offset):
            tag = np.zeros((self.marker_size, self.marker_size, 1), dtype='uint8')
            tag_name = 'markers/' + self.aruco_type + '/'+ self.aruco_type + '_' + str(id) + '.png'
            cv2.aruco.generateImageMarker(arucoDict, id, self.marker_size, tag, 1) # last argument denotes the padding around marker
            cv2.imwrite(tag_name, tag)
        
        print(f"{self.aruco_type} Markers Generated!")


    def camera_calibration(self, chessboardSize = (9,6), frameSize = (640, 360), size_of_chessboard_squares_mm = 23):
        '''
        using the photo in folder 'calibration/sn' to get the parameters for the camera and calibrate the image
        chessboardSize = size of the chessboard
        frameSize = frame size of the photos taken by the camera
        ''' 

        print("Calibration Starts...")

    ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

        objp = objp * size_of_chessboard_squares_mm

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.


        path = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'calibration',
            self.sn+'/*.jpg'
        )
        images = glob.glob(path)

        for image in images:
            img = cv2.imread(image)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, chessboardSize, None)

            # If found, add object points, image points (after refining them)
            if ret == True:

                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                # BUGFIX: this used to append `corners` (the raw integer-ish
                # detections) while discarding the sub-pixel refined `corners2`
                # computed on the line above. cornerSubPix was pure wasted work
                # and the solver was fed ~0.5 px-noisy observations, which
                # biases the focal length far more than the principal point --
                # exactly the fx-too-small / cx-plausible signature seen on
                # several robots.
                imgpoints.append(corners2)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
                cv2.imwrite('img_corners.jpg', img)
                cv2.waitKey(1000)

        cv2.destroyAllWindows()

        # calibrateCamera happily returns a garbage focal length from a handful
        # of near-identical views instead of failing, so refuse to write a
        # calibration that cannot possibly be well conditioned.
        if len(images) == 0:
            raise RuntimeError(
                f"No .jpg photos found under calibration/{self.sn}. The "
                f"take_photo step writes to the *install* share directory and "
                f"cv2.imwrite fails silently if it does not exist.")
        if len(objpoints) < self.MIN_CALIBRATION_VIEWS:
            raise RuntimeError(
                f"Only {len(objpoints)} of {len(images)} photos had a "
                f"detectable {chessboardSize[0]}x{chessboardSize[1]} chessboard "
                f"(need >= {self.MIN_CALIBRATION_VIEWS}). Refusing to write a "
                f"calibration -- too few views makes focal length and distance "
                f"degenerate. Re-shoot with the board filling more of the frame.")

        print(f"Using {len(objpoints)}/{len(images)} photos with a detected board.")

    ############## CALIBRATION #######################################################
        ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

        print(f"Reprojection error (cv2.calibrateCamera ret): {ret:.4f} px")

        # Validate BEFORE writing, so a bad run cannot overwrite a good stored
        # calibration. The yaml on disk is left untouched if this raises.
        try:
            self._sanity_check_intrinsics(
                cameraMatrix, frameSize, context="freshly computed")
        except RuntimeError as err:
            raise RuntimeError(
                f"{err}\n\nThe existing calibration file was NOT overwritten. "
                f"Re-shoot the photos with more varied board poses.") from err

        filename = self.sn+'/'+self.sn + "_cam_cal.yaml"
        path = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'calibration',
            filename
        )
        file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
        file.write(name='K', val=cameraMatrix) # cameraMatrix
        file.write(name='D', val=dist) # DistorsionMatrix
        file.release()

    ############## UNDISTORTION #####################################################

        # Randomly select one image from the set of calibration images
        folder = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'calibration',
            self.sn
        )

        files = os.listdir(folder)
        images = [f for f in files if f.endswith('.jpg')]

        if images:
            random_file = random.choice(images)
            path = os.path.join(folder, random_file)
            img = cv2.imread(path)
        else:
            img = None

        # The undistortion preview below is diagnostic only -- the calibration
        # has already been written. Previously `img = None` fell straight into
        # img.shape and raised AttributeError, discarding a perfectly good
        # calibration run at the very last step.
        if img is None:
            print("Skipping undistortion preview: no readable .jpg available.")
            return

        # Extract new Camera matrix
        h,  w = img.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

        # Undistort
        dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        # cv2.imshow("undistored picture", dst)

        # Undistort with Remapping
        mapx, mapy = cv2.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
        dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        # cv2.imshow("undistored mapping picture", dst)

        # Reprojection Error
        #
        # BUGFIX: this used to divide the L2 norm by len(imgpoints2) (= N, the
        # number of corners per view). cv2.norm(..., NORM_L2) already returns
        # sqrt(sum of squared errors) over all N points, so RMS requires
        # dividing by sqrt(N), not N. The old figure was understated by a
        # factor of sqrt(54) ~= 7.35 -- which is why the cheat sheet's
        # "under 1.0 is good" rule was in practice accepting anything under
        # ~7 px RMS, and why every stored calibration looked acceptable while
        # producing visibly wrong focal lengths.
        total_sq_error = 0.0
        total_points = 0

        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            norm = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)
            total_sq_error += norm * norm
            total_points += len(imgpoints2)

        rms_error = np.sqrt(total_sq_error / total_points)

        print("Calibration Complete!")
        print(f"RMS reprojection error: {rms_error:.4f} px "
              f"(cv2.calibrateCamera reported {ret:.4f})")
        if rms_error > 1.0:
            print(f"WARNING: {rms_error:.2f} px is high -- a good calibration is "
                  f"under ~1.0 px. The intrinsics passed the plausibility check "
                  f"but are probably still imprecise. Consider re-shooting with "
                  f"the board closer, flatter and more varied in angle.")

    

    def _sanity_check_intrinsics(self, K, frameSize, context="loaded"):
        '''
        Raise if a camera matrix is physically implausible for this hardware.

        A bad calibration does not announce itself: pose_estimation() will keep
        returning tvecs, just scaled wrong, and the error grows with distance.
        Catching it here turns a silent multi-hour debugging session into one
        loud message at startup.

        K          = 3x3 camera matrix
        frameSize  = (width, height) the calibration applies to
        context    = wording for the error message
        '''
        if K is None:
            raise RuntimeError(
                f"[{self.sn}] camera matrix is None ({context}). The yaml is "
                f"missing its 'K' node, or the file could not be parsed.")

        K = np.asarray(K, dtype=float)
        if K.shape != (3, 3):
            raise RuntimeError(
                f"[{self.sn}] camera matrix has shape {K.shape}, expected (3, 3).")

        width, height = frameSize
        fx, fy = K[0][0], K[1][1]
        cx, cy = K[0][2], K[1][2]

        problems = []

        # fx and fy scale linearly with image width, so normalise before
        # comparing against the 640-px reference.
        expected_fx = self.REFERENCE_FX_AT_640 * (width / 640.0)
        lo = expected_fx * (1.0 - self.FX_TOLERANCE)
        hi = expected_fx * (1.0 + self.FX_TOLERANCE)
        if not (lo <= fx <= hi):
            problems.append(
                f"fx={fx:.1f} is outside [{lo:.1f}, {hi:.1f}] "
                f"(driver reports ~{expected_fx:.1f} at {width}x{height}). "
                f"Distance estimates will be off by ~{expected_fx / fx:.2f}x.")

        # Square pixels: fx and fy must agree closely on this sensor.
        if fx > 0 and abs(fx - fy) / fx > 0.10:
            problems.append(
                f"fx={fx:.1f} and fy={fy:.1f} differ by "
                f"{100 * abs(fx - fy) / fx:.0f}% (expected <10%).")

        # Principal point belongs near the image centre.
        if abs(cx - width / 2.0) > self.PRINCIPAL_POINT_TOLERANCE * width:
            problems.append(
                f"cx={cx:.1f} is far from the image centre {width / 2.0:.1f}.")
        if abs(cy - height / 2.0) > self.PRINCIPAL_POINT_TOLERANCE * height:
            problems.append(
                f"cy={cy:.1f} is far from the image centre {height / 2.0:.1f}.")

        if problems:
            raise RuntimeError(
                f"[{self.sn}] implausible {context} intrinsics:\n  - "
                + "\n  - ".join(problems)
                + f"\nRe-run calibration (see scripts/camera-calibration-cheatsheet.md) "
                  f"or inspect with scripts/check_intrinsics.py.")

        return True


    def load_calibration(self, frameSize=(640, 360), strict=True):
        '''
        initialize the detection of aruco markers: load the calibration parameters

        frameSize = (width, height) of the stream these intrinsics will be used
                    on, used to sanity-check the values.
        strict    = raise on implausible intrinsics. Set False to downgrade to
                    a printed warning (e.g. while deliberately testing a known
                    bad calibration).
        '''
        filename = self.sn + '/' + self.sn + "_cam_cal.yaml"
        filepath = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'calibration',
            filename
        )

        # Fail loudly and early. Previously a missing file sailed through: the
        # FileStorage read returned an empty node, .mat() returned None, and the
        # first frame blew up deep inside estimatePoseSingleMarkers where the
        # detector node's broad `except` swallowed it into a single warn line.
        if not os.path.isfile(filepath):
            raise FileNotFoundError(
                f"[{self.sn}] no calibration file at {filepath}. Check the "
                f"serial_number parameter matches a folder under "
                f"swarm_bringup/calibration, and that swarm_bringup has been "
                f"rebuilt (this reads the install share directory, not src/).")

        cv_file = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
        try:
            self.CameraMatrix = cv_file.getNode('K').mat() # cameraMatrix
            self.DistortionMatrix = cv_file.getNode('D').mat() # DistorsionMatrix
        finally:
            cv_file.release()

        if self.DistortionMatrix is None:
            raise RuntimeError(
                f"[{self.sn}] calibration file {filepath} has no 'D' node.")

        try:
            self._sanity_check_intrinsics(self.CameraMatrix, frameSize)
        except RuntimeError:
            if strict:
                raise
            print(f"WARNING: [{self.sn}] intrinsics failed the sanity check and "
                  f"are being used anyway (strict=False).")

        return self.CameraMatrix, self.DistortionMatrix



    def pose_estimation(self, frame):
        '''
        given frames of image, find the relative position of aruco markers
        frame = a frame of the video taken in numpy array format
        '''
        tvec = []

        # Accept either a single-channel gray frame (the hardware decoder now
        # publishes mono8) or a legacy 3-channel BGR frame. Only convert when
        # we are actually handed colour -- the detector needs gray regardless.
        if frame.ndim == 2:
            gray = frame
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        if len(corners) > 0:

            self.marker_num = len(ids)
            self.tvecs = np.zeros([self.marker_num,2])
            self.rvecs = np.zeros([self.marker_num,3])

            for i in range(0, len(ids)):

                rvec, tvec, markerPoints = estimatePoseSingleMarkers(corners[i],
                        self.aruco_marker_side_length, self.CameraMatrix, self.DistortionMatrix)

                # NOTE: drawDetectedMarkers()/drawFrameAxes() removed -- they
                # annotated `frame` for a debug display that is never shown
                # (the detector node discards the returned frame), so they were
                # pure per-marker CPU waste and also require a 3-channel image
                # (incompatible with the new mono8 input).
                rvec = np.array(rvec, dtype=np.float32)
                tvec = np.array(tvec, dtype=np.float32)

                self.tvecs[i,0] = tvec[0][2] * self.scalex
                self.tvecs[i,1] = -1 * tvec[0][0] * self.scaley # Room configuration for y-axis


                self.rvecs[i,0] = rvec[0][0]
                self.rvecs[i,1] = rvec[0][1]
                self.rvecs[i,2] = rvec[0][2]
                # print(f"Marker {ids[i]}: Position = ({self.tvecs[i][0]:.2f}, {self.tvecs[i][1]:.2f}, {self.tvecs[i][2]:.2f}) meters")
        else:
            self.tvecs = np.zeros([0,2])
            self.rvecs = np.zeros([0,3])

        return frame

def estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
    
    # Ensure mtx is a 3x3 float32 NumPy array
    mtx = np.array(mtx, dtype=np.float32)
    distortion = np.array(distortion, dtype=np.float32)
    
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash

if __name__ == "__main__":
    # take photos of the chessboard and calibrate the camera 
    sn = "159CKC50070ECX"
    aruco = ArucoMarker(aruco_type="DICT_5X5_100", sn=sn)
    aruco.take_photo()
    aruco.camera_calibration()