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
                imgpoints.append(corners)

                # Draw and display the corners
                cv2.drawChessboardCorners(img, chessboardSize, corners2, ret)
                cv2.imwrite('img_corners.jpg', img)
                cv2.waitKey(1000)

        cv2.destroyAllWindows()

    ############## CALIBRATION #######################################################
        ret, cameraMatrix, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

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
        mean_error = 0

        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error

        print( "Calibration Complete!\ntotal error: {}".format(mean_error/len(objpoints)) )

    

    def load_calibration(self):
        '''
        initialize the detection of aruco markers: load the calibration parameters
        '''
        filename = self.sn + '/' + self.sn + "_cam_cal.yaml"
        filepath = os.path.join(
            get_package_share_directory('swarm_bringup'),
            'calibration',
            filename
        )
        cv_file = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ) 
        self.CameraMatrix = cv_file.getNode('K').mat() # cameraMatrix
        self.DistortionMatrix = cv_file.getNode('D').mat() # DistorsionMatrix
        cv_file.release()

        return self.CameraMatrix, self.DistortionMatrix



    def pose_estimation(self, frame):
        '''
        given frames of image, find the relative position of aruco markers
        frame = a frame of the video taken in numpy array format
        '''
        tvec = []
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_type])
        parameters = cv2.aruco.DetectorParameters()

        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)

        if len(corners) > 0:

            self.marker_num = len(ids)
            self.tvecs = np.zeros([self.marker_num,2])
            self.rvecs = np.zeros([self.marker_num,3])

            for i in range(0, len(ids)):

                rvec, tvec, markerPoints = estimatePoseSingleMarkers(corners[i], 
                        self.aruco_marker_side_length, self.CameraMatrix, self.DistortionMatrix)

                cv2.aruco.drawDetectedMarkers(frame, corners)

                rvec = np.array(rvec, dtype=np.float32)
                tvec = np.array(tvec, dtype=np.float32)
                cv2.drawFrameAxes(frame, self.CameraMatrix, self.DistortionMatrix, rvec, tvec, 3)

                
                
                
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