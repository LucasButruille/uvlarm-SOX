
import pyrealsense2 as rs
import numpy as np
import math
import cv2,time,sys

pipeline = rs.pipeline()
config = rs.config()
colorizer = rs.colorizer()

# fps plus bas (30)
config.enable_stream(rs.stream.depth, 840, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 840, 480, rs.format.bgr8, 30)

pipeline.start(config)

align_to = rs.stream.depth
align = rs.align(align_to)

color_info=(0, 0, 255)
rayon=10

count=1
refTime= time.process_time()
freq= 60

try:
    while True:
        # This call waits until a new coherent set of frames is available on a device
        frames = pipeline.wait_for_frames()
        
        #Aligning color frame to depth frame
        aligned_frames =  align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not aligned_color_frame: continue

        # Two ways to colorized the depth map
        # first : using colorizer of pyrealsense                
        colorized_depth = colorizer.colorize(depth_frame)
        depth_colormap = np.asanyarray(colorized_depth.get_data())
        
        # second : using opencv by applying colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_image = np.asanyarray(depth_frame.get_data())
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Get the intrinsic parameters
        color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics

        color_image = np.asanyarray(aligned_color_frame.get_data())

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        #Use pixel value of  depth-aligned color image to get 3D axes
        x, y = int(color_colormap_dim[1]/2), int(color_colormap_dim[0]/2)
        depth = depth_frame.get_distance(x, y)
        dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
        distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
        
        #print("Distance from camera to pixel:", distance)
        #print("Z-depth from camera surface to pixel surface:", depth)

        #Show images
        images = np.hstack((color_image, depth_colormap)) # supose that depth_colormap_dim == color_colormap_dim (640x480) otherwize: resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        cv2.circle(images, (int(x), int(y)), int(rayon), color_info, 2)
        cv2.circle(images, (int(x+color_colormap_dim[1]), int(y)), int(rayon), color_info, 2)
        
        # Affichage distance au pixel (x,y)
        cv2.putText(images, "D="+str(round(distance,2)), (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
        cv2.putText(images, "D="+str(round(distance,2)), (int(x+color_colormap_dim[1])+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_NORMAL)

        # Resize the Window
        cv2.resizeWindow('RealSense', 960, 720)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # Frequency:
        if count == 10 :
            newTime= time.process_time()
            freq= 10/((newTime-refTime))
            refTime= newTime
            count= 0
        count+= 1

except Exception as e:
    print(e)
    pass

finally:
    pipeline.stop()

