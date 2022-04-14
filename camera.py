
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
            # Create a pipeline
        print("start pipeline")
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        print("config")
        self.config = rs.config()

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

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        print("Align")
        self.align_to = rs.stream.color
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

    def find_circles(self, frame): #, x, y, radius):    
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=1, \
            # minDist=100, param1=200, param2=10, maxRadius=70, minRadius=30)
            minDist=100, param1=100, param2=10, maxRadius=30, minRadius=15)

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
        # cv2.circle(color_image, (x_idx, y_idx), 15, (0, 0, 255), -1)

        # draw bounding box/ ROI
        x_spacing = 60
        y_spacing = 40

        y_idx += 50
        x_idx += 100
        xL = x_idx-x_spacing
        xR = x_idx+x_spacing
        yT = y_idx - 4
        yB = y_idx + y_spacing

        # Check edge cases
        if xL < 0:
            xL = 0
        elif xR > gray_color.shape[1]:
            xR = gray_color.shape[1]
        if yT < 0:
            yT = 0
        elif yB > gray_color.shape[0]:
            yB = gray_color.shape[0]

        # cv2.rectangle(color_image, (xL, yT), (xR, yB), (0,0,255), 5)
        # cv2.rectangle(depth_colormap, (xL, yT), (xR, yB), (0,0,255), 5)

        roi_color = gray_color[yT:yB, xL:xR]
        roi_depth = depth_gray[yT:yB, xL:xR]
        roi_edges = cv2.Canny(roi_color, 10, 255)

        # cv2.imshow("roi_depth", roi_depth)
        # cv2.imshow("roi_color", roi_color)
        # cv2.imshow("roi_edges", roi_edges)

        # Time to look for circles / try different approaches
        circles_color = find_circles(roi_color)
        circles_depth = find_circles(roi_depth)
        circles_edges = find_circles(roi_edges)

        if circles_color is not None:
            num_circles += 1

        if circles_edges is not None:
            num_circles += 1

        if self.display == True:
            color_image_circle = draw_circles(circles_color, color_image, xL, yT)
            canny_image_circle = draw_circles(circles_edges, color_image, xL, yT)

        # Color Contours
        ###############################################
        # Get the contour and check its area
        contours = cv2.findContours(roi_color, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        max_area_c = 0
        top_cnt = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area_c:
                max_area_c = area
                top_cnt = cnt
        # find the centroid of the contour
        # if top_cnt != []:
            # print("Max area:", max_area)
        color_image_contour = color_image

        if max_area_c > 700 and max_area_c < 1000:
            num_circles += 1
            M = cv2.moments(top_cnt)

            new_size = np.sqrt(max_area_c / np.pi)

            # draw the circle and bounding box
            if self.display == True:
                color_image_contour = cv2.circle(color_image,
                                (xL+int(M['m10'] / M['m00']), yT+int(M['m01'] / M['m00'])), int(new_size), (0, 255, 0), -1)
        ###############################################

        # Depth Contours
        ###############################################
        # Get the contour and check its area
        contours = cv2.findContours(roi_depth, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]
        max_area_d = 0
        top_cnt = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area_d:
                max_area_d = area
                top_cnt = cnt
        # find the centroid of the contour
        # if top_cnt != []:
        #     print("Max area:", max_area)
        if max_area_d > 600 and max_area_d < 1000:
            stop_flag = True
            # num_circles += 1
            M = cv2.moments(top_cnt)

            new_size = np.sqrt(max_area_d / np.pi)

            if display == True:
                depth_colormap = cv2.circle(depth_colormap,
                                (xL+int(M['m10'] / M['m00']), yT+int(M['m01'] / M['m00'])), int(new_size), (0, 255, 0), -1)


        if num_circles >= 1 and stop_flag == True:
            self.num_frames_found += 1
            if self.num_frames_found >= 2:   # Num frams ball must be found before detection is raised
                return True, (xR-xL)/2, depth_frame.get_distance(int((xR-xL)/2), int(yB)) # Raise flag
        else:
            self.num_frames_found = 0
            return False, 0, 0


        if self.display == True:
            cv2.namedWindow('Hough Circle Color', cv2.WINDOW_NORMAL)
            cv2.imshow('Hough Circle Color', color_image_circle)

            cv2.namedWindow('canny_image_circle', cv2.WINDOW_NORMAL)
            cv2.imshow('canny_image_circle', canny_image_circle)
            canny_image_circle

            cv2.namedWindow('Color Contour', cv2.WINDOW_NORMAL)
            cv2.imshow('Color Contour', color_image_contour)

            cv2.namedWindow('Depth', cv2.WINDOW_NORMAL)
            cv2.imshow('Depth', depth_colormap) #depth_gray)

            # cv2.namedWindow('Depth_hist', cv2.WINDOW_NORMAL)
            # cv2.imshow('Depth_hist', depth_gray)

            cv2.namedWindow('white', cv2.WINDOW_NORMAL)
            cv2.imshow('white', gray_color)
            cv2.waitKey(1)
        
        def stop_pipeline(self):
            pipeline.stop()