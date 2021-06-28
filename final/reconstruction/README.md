# Reconstruction
特征点检测法 -> 八点法获取R和t -> 确定t的scale -> rectify -> dense matching 

# Dataset 
- KITTI_TEST
  - ./test_images/kitti_test_01
  - 这个数据集中包含左右两张图像， 是KITTI中rectified的两张图像(rectified KITTI - 2011_09_26_drive_0113_sync - 0.png)
- MATLAB_TEST
  - ./test_images/matlab
  - 这个数据集中包含左右两张rectified的图像，来源于CV2的exe6
- KITTI_2011_09_26_drive_0048
  - KITTI官网Raw Data
  - 2011_09_26_drive_0048 (0.1 GB)
  - Length: 28 frames (00:02 minutes)
  - Image resolution: 1392 x 512 pixels
  - Labels: 7 Cars, 1 Vans, 0 Trucks, 0 Pedestrians, 0 Sitters, 0 Cyclists, 0 Trams, 0 Misc
- KITTI_2011_09_26_drive_0113


# 存在的问题
* 目前即使通过手动给点，计算得到的t的scale也不太正确。根据KITTI数据集给定的标定参数，t应该为 [ 0.53267121, -0.00526146,  0.00782809] (考虑到rectify前后optical center位置不变， 即 T02-T03), 但目前的计算结果和这个有较大的差异；
* 在MATLAB_TEST中， 答案中 t = [-0.4427, -0.0166, 0.8965], gamma = 0.0081, 即 t*gamma = [-0.00358587, -0.00013446,  0.00726165]. 本程序得到的结果为 [-0.00887002165298002, -0.0002242175976263845, 0.0108172731965549], 这个结果差异似乎还不是很大。

# 已验证
- KITTI数据集中的K (calibration matrix) 在经过rectified后, 发生了变化 (kitti_kit.cpp)


# Note
* 目前undistort.cpp中的相机内参和畸变参数为直接赋值，如果引入其他的需要去畸变的数据集需要注意。



    
