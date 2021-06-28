#include "include.h"
#define DEBUG_PRINT 1

using namespace std;
using namespace cv;

// // K_02
// cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.597910e+02, 0.000000e+00, 6.960217e+02, 
//                                                             0.000000e+00, 9.569251e+02, 2.241806e+02, 
//                                                             0.000000e+00, 0.000000e+00, 1.000000e+00);

// // K_03
// cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.037596e+02, 0.000000e+00, 6.957519e+02, 
//                                                             0.000000e+00, 9.019653e+02, 2.242509e+02, 
//                                                             0.000000e+00, 0.000000e+00, 1.000000e+00);

// // Calibration Parameters
// // K_02
// cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 555.80785, 0.000000e+00, 330.17268, 
//                                                             0.000000e+00, 557.34962, 244.70437, 
//                                                             0.000000e+00, 0.000000e+00, 1.000000e+00);

// // K_03
// cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 558.11555, 0.000000e+00, 314.72552, 
//                                                             0.000000e+00, 559.49020, 232.64247, 
                                                            // 0.000000e+00, 0.000000e+00, 1.000000e+00);


// rectified camera parameters
// K_02
cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 721.5377, 0, 609.5593, 0, 721.5377, 172.854, 0, 0, 1);

// K_03
cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 721.5377, 0, 609.5593, 0, 721.5377, 172.854, 0, 0, 1);

// // matlab
// // K_02
// cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 844.310547, 0, 243.413315, 0, 1202.508301, 281.529236, 0, 0, 1);

// // K_03
// cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 852.721008, 0, 252.021805, 0, 1215.657349, 288.587189, 0, 0, 1);


Mat FindEssentialMatrix(Mat fundamental_mat){
    if(DEBUG_PRINT){
        cout<<"fundamental_mat.size(): "<<fundamental_mat.size()<<endl;
    }
    return right_rgb_camera_matrix.t() * fundamental_mat * left_rgb_camera_matrix;

}