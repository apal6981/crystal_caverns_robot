
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##         Align Depth to Color and Record         ##
#####################################################

# First import the library
# import pyrealsense2 as rs
import pyrealsense2.pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

display = False

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

vid_name = "realsense_record"
video_name_c = str(vid_name + "_color.avi")
video_name_d = str(vid_name + "_depth.avi")
fps = 15
writer_c = None
writer_d = None
# Streaming loop
try:
    colorizer = rs.colorizer()
    frame_num = 0
    while True:
        stop_flag = False
        frame_num += 1
        num_circles = 0
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()


        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_color_frame = colorizer.colorize(aligned_depth_frame)
        depth_color_image = np.asanyarray(depth_color_frame.get_data())        

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        if display == True:
            cv2.namedWindow('Color', cv2.WINDOW_NORMAL)
            cv2.imshow('Color', color_image)

            cv2.namedWindow('depth_colormap', cv2.WINDOW_NORMAL)
            cv2.imshow('depth_colormap', depth_colormap)
            cv2.waitKey(1)
        
        if writer_c is None:
            writer_c = cv2.VideoWriter(video_name_c, cv2.VideoWriter_fourcc(*'MJPG'), fps, (color_image.shape[1], color_image.shape[0]), True)
            writer_d = cv2.VideoWriter(video_name_d, cv2.VideoWriter_fourcc(*'MJPG'), fps, (depth_colormap.shape[1], depth_colormap.shape[0]), True)

        writer_c.write(color_image)
        writer_d.write(depth_colormap)
finally:
    pipeline.stop()
