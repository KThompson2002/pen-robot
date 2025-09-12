import pyrealsense2 as rs
import numpy as np
import cv2



lower_purple = np.array([110,65,50])
upper_purple = np.array([130,255,200])

class Stream():
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))

    def __enter__(self):
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.profile = self.pipeline.stop()

    def record(self, video):
        self.config.enable_record_tofile(video)
    
    def set_scale(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # print("Depth Scale is: " , depth_scale)

    def align_self(self):
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def play_Back(self, video):
        self.config.enable_device_from_file(video)

    def capture_frame(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        clipping_distance = 2 / self.depth_scale

        # Get aligned frames
        self.aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        # if not aligned_depth_frame or not color_frame:
        #     continue

        depth_image = np.asanyarray(self.aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        self.bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        return self.bg_removed

    def convert_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        return hsv
    
    def threshold_purple(self, frame):
        mask = cv2.inRange(frame, lower_purple, upper_purple)
        return cv2.bitwise_and(self.bg_removed,self.bg_removed, mask= mask), mask

    def find_pen(self, mask):
        # imgray = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
        ret, thresh = cv2.threshold(mask, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh, 1, 2)
        if len(contours) < 1:
            return (-1, -1, -1), np.array([0, 0])
        elif len(contours) >= 1:
            # ind = 0 
            # max_ = 0
            # for i in contours:
            #     area = cv2.contourArea(i)
            #     if (area > max_):
            #         ind = i
            #         max_ = area
            #     i += 1

            # # print(ind)

            # cnt = contours[ind]

            cnt = max(contours, key=cv2.contourArea)
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                return (-1, -1, -1), np.array([0, 0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            prof = self.profile.get_stream(rs.stream.color)
            intr = prof.as_video_stream_profile().get_intrinsics()
            depth = self.aligned_depth_frame.get_distance(cx, cy)
            return rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth), np.array([cx, cy])
        # else:
        #     M = cv2.moments(contours)
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        #     prof = self.profile.get_stream(rs.stream.color)
        #     intr = prof.as_video_stream_profile().get_intrinsics()
        #     depth = self.aligned_depth_frame.get_distance(cx, cy)
        #     return rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth), np.array([cx, cy])

    # def clean_img(self, img):
        

    

    # def identify_pen(self, img):

