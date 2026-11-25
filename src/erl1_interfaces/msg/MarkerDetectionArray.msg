std_msgs/Header header    # timestamp + camera frame of the image
uint32 image_width        # needed to compute cx = image_width / 2
uint32 image_height
MarkerDetection[] markers # 0, 1 or more markers seen in this frame