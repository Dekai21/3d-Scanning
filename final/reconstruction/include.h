#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

struct transformation{
    cv::Mat R;
    cv::Mat t;
};

// // K_02
// cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.597910e+02, 0.000000e+00, 6.960217e+02, 
//                                                             0.000000e+00, 9.569251e+02, 2.241806e+02, 
//                                                             0.000000e+00, 0.000000e+00, 1.000000e+00);

// // K_03
// cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.037596e+02, 0.000000e+00, 6.957519e+02, 
//                                                             0.000000e+00, 9.019653e+02, 2.242509e+02, 
//                                                             0.000000e+00, 0.000000e+00, 1.000000e+00);

int OrbDetector (Mat img_1, Mat img_2, 
                 vector<KeyPoint>& keypoints_left, vector<KeyPoint>& keypoints_right,
                 size_t num_keypoints);


void Undistort(Mat distorted_left_image, Mat distorted_right_image, 
               Mat& undistorted_left_image,  Mat& undistorted_right_image);

Mat FindEssentialMatrix(Mat fundamental_mat);

struct transformation RecoverRT(Mat R1, Mat R2, Mat T, vector<KeyPoint> keypoints_left, vector<KeyPoint> keypoints_right);
