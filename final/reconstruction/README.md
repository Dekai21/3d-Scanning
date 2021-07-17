# Reconstruction
- 相机外参的计算
  - 八点法 + 全局法: 采用ORB, SIFT
  - Bundle Adjustment: 采用ORB, SIFT
  - 先在undistort的左图上手动选点, 然后根据指定的r和t计算出右图上对应的点, 采用八点法看看能不能恢复出正确的r和t
  - 采用bundle adjustment来看看能不能恢复相应的点, 虽然我觉得点的数量不足可能会造成困难
  - 验证orb结合八点法和bundle adjustment的表现
  - 估计orb+八点法表现很差, 所以分析一下原因

- Rectify
  - to do: 但是感觉即使用bundle adjustment得到的R和t精度也不够
- Dense matching
  - block matching
  - semi global matching
  - 基于学习
- 深度图质量评估
  - to do 
- 可视化: 输出点云 + mesh

# Dataset 
- KITTI_TEST
  - ./test_images/kitti_test_01
  - 这个数据集中包含左右两张图像， 是KITTI中rectified的两张图像(rectified KITTI - 2011_09_26_drive_0113_sync - 0.png)
  - 增加: 2011_09_26_drive_0048_sync - 1.png
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


# 已验证
- KITTI数据集中的K (calibration matrix) 在经过rectified后, 发生了变化 (kitti_kit.cpp)
- 八点法的鲁棒性较差，得到的R和t精度不行
- 计算R和t需要很多特征点 (200+?)


# Note
* 目前undistort.cpp中的相机内参和畸变参数为直接赋值，如果引入其他的需要去畸变的数据集需要注意。

# 环境
- OpenCV 3.2
- Eigen 3
- g2o @ 9b41a4e

# 实验结果
- KITTI rectified image (left rgb)
  ![left_rgb](./results/left.png)
- R和t的估算
  ![R_T](./results/Screenshot%20from%202021-06-30%2017-16-09.png)
- disparity map from sgm
  ![left_disp](./results/left_disp.png)
- depth
  ![depth](./results/depth_vis_decimeter.jpg)
- point cloud
  ![pc1](./results/kitti_test_pointcloud00.png)
  ![pc2](./results/kitti_test_pointcloud01.png)
- depth maps from block matching and semi-global matching
  - block matching
    - ![depth_bm](./results/depth_vis_decimeter_bm.jpg)
  - semi-global matching
    - ![depth_sgbm](./results/depth_vis_decimeter_sgbm.jpg)
    - 注: 相比上图, 这里的depth map中的255均被替换为0
  
  

  <!-- - 八点法
  
    - R和t:
  $$
  \left[
  \begin{matrix}
      0.9998050317301672 & -0.01973988720867603 & 0.0004851597964776189 \\
       0.01972332024661993 & 0.9995362024675716 & 0.02320281438005439 \\
      -0.000942955719347105 & -0.02318872160544275 & 0.99973066074059
  \end{matrix}
  \right]
  $$

  $$
  \left[
  \begin{matrix}
      -0.001032302587118445 \\
       7.174486476482287e-05 \\
      0.0009532134213666677
  \end{matrix}
  \right]
  $$


   - BundleAdjustment
     - R和t
    $$
  \left[
  \begin{matrix}
      0.746854 & 0.00590651  & -0.664961 \\
      -0.0455006 &  0.998071 & -0.0422388 \\
      0.663429 & 0.0618024 &   0.745682 
  \end{matrix}
  \right]
  $$

  $$
  \left[
  \begin{matrix}
      0.517458\\
      0.0317785\\
      0.340954
  \end{matrix}
  \right]
  $$

  - Ground truth
     - R和t
    $$
    \left[
    \begin{matrix}
        1 & 0  & 0 \\
        0 &  1 & 0 \\
        0 & 0 &   1 
    \end{matrix}
    \right]
  $$

  $$
    \left[
    \begin{matrix}
      0.5327119287764802\\
      -0.002752896950590371\\
      1.597899999998976e-05
    \end{matrix}
    \right]
  $$ -->


<!-- MATLAB TEST
>>> import numpy as np
>>> a = np.array([-0.4427, -0.0166, 0.8965])
>>> a
array([-0.4427, -0.0166,  0.8965])
>>> np.linalg.norm(a)
0.9999855498955972
>>> gamma = 0.0081
>>> gamma * a
array([-0.00358587, -0.00013446,  0.00726165]) -->


先测试在八点法的基础上，bundle adjustment能否使结果改善
评估r和t的代码
将得到的r和t用于rectify

计算R和t的ground truth
计算R对应的欧拉角
由八点法得到R和t
使用bundle adjustment进行优化

/usr/include/opencv2/xfeatures2d.hpp