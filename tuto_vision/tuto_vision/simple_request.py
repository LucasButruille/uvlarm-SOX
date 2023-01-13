#!/usr/bin/env python3

###############################################
##              Simple Request               ##
###############################################

import pyrealsense2 as rs

def main() :
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    print( f"Connect: {device_product_line}" )
    for s in device.sensors:
        print( "Name:" + s.get_info(rs.camera_info.name) )
