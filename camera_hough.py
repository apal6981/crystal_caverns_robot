
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2


class Camera:
    def __init__(self, display=False):
        self.display = display
        self.record = False

        vid_name = "realsense_record"
        self.video_name_c = str(vid_name + "_color.avi")
        self.video_name_d = str(vid_name + "_depth.avi")
        self.fps = 3
        self.writer_c = None
        self.writer_d = None
            # Create a pipeline
        print("start pipeline")
        self.pipeline = rs.pipeline()
        self.template = cv2.imread('mask_2_template.png',0)
        self.prev_lx = 480
        self.current_corner = 4
        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        print("config")
        self.config = rs.config()
        self.frame_num = 0

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print("Wrapper stuff")

        self.found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        
        print("Enable stream")
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.profile = self.pipeline.start(self.config)
        s = self.profile.get_device().query_sensors()[1]
        s.set_option(rs.option.exposure, 60)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        print("Align")
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.colorizer = rs.colorizer()

        # Flags
        self.num_frames_found = 0
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        print("Gathering frames")
        color_frame = aligned_frames.get_color_frame()
        depth_color_frame = self.colorizer.colorize(aligned_depth_frame)
       
        color_frame = aligned_frames.get_color_frame()
        depth_color_frame = self.colorizer.colorize(aligned_depth_frame)

        color_frame = aligned_frames.get_color_frame()
        depth_color_frame = self.colorizer.colorize(aligned_depth_frame)
        print("frames have been gathered")

    def find_circles(self, frame): #, x, y, radius):    
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=1, \
            # minDist=100, param1=200, param2=10, maxRadius=70, minRadius=30)
            minDist=100, param1=100, param2=15, maxRadius=50, minRadius=15)

        return circles

    def draw_circles(self, circles, frame, x_offset, y_offset):

        output = frame.copy()   
        # ensure at least some circles were found
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, radius) in circles:
                cv2.circle(output, (x+x_offset, y+y_offset), radius, (0, 255, 0), 5)
        return output


    def find_ball(self):
    # Streaming loop
        self.frame_num += 1
        ball_x_loc = 0
        stop_flag = False
        num_circles = 0
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        depth_color_frame = self.colorizer.colorize(aligned_depth_frame)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())        

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Images gathered, ready to be processed

        if self.writer_c is None:
            self.writer_c = cv2.VideoWriter(self.video_name_c, cv2.VideoWriter_fourcc(*'MJPG'), self.fps, (color_image.shape[1], color_image.shape[0]), True)
            self.writer_d = cv2.VideoWriter(self.video_name_d, cv2.VideoWriter_fourcc(*'MJPG'), self.fps, (color_image.shape[1], color_image.shape[0]), True)

        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Erode, dilate, and threshold color image to find white lines/ ball
        gray_color = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        erode_kernel = np.ones((5, 5))
        kernel = np.ones((5, 5), np.uint8)
        _, gray_color = cv2.threshold(gray_color, 50, 255, cv2.THRESH_BINARY)
        gray_color = cv2.erode(gray_color, erode_kernel, iterations=1)
        gray_color = cv2.dilate(gray_color, kernel, iterations=1)

        circles_color = self.find_circles(gray_color)
        circle_image = self.draw_circles(circles_color, color_image, 0, 0)
        self.writer_c.write(gray_color)
        self.writer_d.write(circle_image)
        
        ##### Template
        # w, h = self.template.shape[::-1]
        # method = cv2.TM_CCOEFF
        # res = cv2.matchTemplate(gray_color,self.template,method)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        # top_left = max_loc
        # if top_left[0] < 20 and self.prev_lx > 240:
        #     self.current_corner -= 1
        #     if self.current_corner == -1:
        #         self.current_corner = 7
        #     print("viewing corner #", self.current_corner)
        # self.prev_lx = top_left[0]
        # if self.display == True:
        #     bottom_right = (top_left[0] + w, top_left[1] + h)
        #     cv2.rectangle(gray_color,top_left, bottom_right, 255, 2)
        #     cv2.imshow("template", gray_color)

        dist_array = []
        total_dist = 0
        if circles_color is not None:
            x_mid = circles_color[0][0][0]
            y_mid = circles_color[0][0][1]
        else:
            x_mid = 320
            y_mid = 240
        for i in range(3):
            dist = aligned_depth_frame.get_distance(int(x_mid), int((y_mid-i*3)))
            if dist < 0.05:
                continue
            dist_array.append(dist)
        
        if dist_array == []:
            total_dist = 5
            print("Dist array is 0!")
        else:
            total_dist = np.average(np.array(dist_array))
        print("Frame num", self.frame_num)
        if self.frame_num <= 8:
            total_dist = 5
        
        if circles_color is not None:
            self.num_frames_found += 1
            if self.num_frames_found >= 1:   # Num frams ball must be found before detection is raised
                return True, (x_mid/480), total_dist  # Raise flag
            return False, x_mid/480, total_dist
        else:
            self.num_frames_found = 0
            return False, x_mid/480, total_dist
        
    def find_corner(self):
    # Streaming loop
        ball_x_loc = 0
        stop_flag = False
        num_circles = 0
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        depth_color_frame = self.colorizer.colorize(aligned_depth_frame)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())        

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Images gathered, ready to be processed

        # Time to resize images
        height, width, layers = color_image.shape
        new_h = 240
        new_w = 480
        color_image = cv2.resize(color_image, (new_w, new_h))
        depth_image = cv2.resize(depth_image, (new_w, new_h))

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Equalize depth image
        depth_gray = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2GRAY)
        depth_gray = cv2.equalizeHist(depth_gray)

        # Erode and Dilate gray depth image
        erode_kernel = np.ones((21, 21))
        kernel = np.ones((11, 11), np.uint8)
        depth_gray = cv2.erode(depth_gray, erode_kernel, iterations=1)
        depth_gray = cv2.dilate(depth_gray, kernel, iterations=1)

        # Erode, dilate, and threshold color image to find white lines/ ball
        gray_color = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        erode_kernel = np.ones((5, 5))
        kernel = np.ones((5, 5), np.uint8)
        _, gray_color = cv2.threshold(gray_color, 220, 255, cv2.THRESH_BINARY)
        gray_color = cv2.erode(gray_color, erode_kernel, iterations=1)
        gray_color = cv2.dilate(gray_color, kernel, iterations=1)

        # Find max values across all columns, then find highest 255 value
        gray_color_temp = gray_color[50:-17, 100:-100]
        white_vals = np.amax(gray_color_temp, axis=1)
        y_idx = (white_vals!=0).argmax()
        x_idx = (gray_color_temp[y_idx]!=0).argmax()

        ##### Template
        w, h = self.template.shape[::-1]
        method = cv2.TM_CCOEFF
        res = cv2.matchTemplate(gray_color,template,method)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        if top_left[0] < 20 and self.prev_lx > 240:
            self.current_corner -= 1
            if self.current_corner == -1:
                self.current_corner = 7
            print(frame_num, "viewing corner #", self.current_corner)
        self.prev_lx = top_left[0]

        if self.display == True:
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(gray_color,top_left, bottom_right, 255, 2)
            cv2.imshow("template", gray_color)

        center = (top_right[0] - top_left[0])/2 + top_left[0]

        return True, center, aligned_depth_frame.get_distance(int(center*1.33), int(top_left[1]-15)*1.5) # Raise flag

        if self.display == True:
            cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
            cv2.imshow('Depth', depth_colormap) #depth_gray)

            cv2.namedWindow('white', cv2.WINDOW_NORMAL)
            cv2.imshow('white', gray_color)
            cv2.waitKey(1)

    def drive_to_ball(self):
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        
        dist_array = []
        total_dist = 0
        for i in range(3):
            dist = aligned_depth_frame.get_distance(int(320), int(150-(i*4)))
            if dist < 0.05:
                continue
            dist_array.append(dist)
        
        if dist_array == []:
            total_dist = 0.4
            print("Dist array is 0!")
        else:
            total_dist = np.average(np.array(dist_array))

        return total_dist

    def stop_pipeline(self):
        pipeline.stop()

    