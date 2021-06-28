# Reconstruction

# Dataset - KITTI Raw
2011_09_26_drive_0048 (0.1 GB)
Length: 28 frames (00:02 minutes)
Image resolution: 1392 x 512 pixels
Labels: 7 Cars, 1 Vans, 0 Trucks, 0 Pedestrians, 0 Sitters, 0 Cyclists, 0 Trams, 0 Misc

# 存在的问题
* 如果只对rectified的图像求R和T, 目前求出来的T的scale似乎不太正确, 是特征点的问题? 还是我算法的问题?
* KITTI数据集中的K (calibration matrix) 在经过rectified后, 是否发生了变化? 
* RecoverRT.cpp 中 assert(count_success == 1) 没有准确报错
* 在哪里放置left_rgb_camera_matrix比较合适？考虑定义虚拟相机类？


# ground truth t?
array([ 0.53267121, -0.00526146,  0.00782809])

# Note
* 改变数据集时, 目前需要修改undistort.cpp和FindEssentialMatrix.cpp中的相机参数


# 手动标注
/home/dekai/datasets/2011_09_26_drive_0113/2011_09_26_drive_0113_sync/2011_09_26/2011_09_26_drive_0113_sync/image_02/data/0000000000.png


    
