uint8 PCD_XYZ = 0
uint8 PCD_RGBXYZ = 1
uint8 PCD_IRXYZ = 2
bool stopped
bool discontinuity
uint16 rcv_fps
PlayTime play_time
FrameImage depth
FrameImage ir
FrameImage raw1
FrameImage raw2
FrameImage raw3
FrameImage raw4
sensor_msgs/PointCloud2 pcd
uint8 pcd_kind
