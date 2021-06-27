#include "include.h"
#define DEBUG_PRINT 1

using namespace std;
using namespace cv;

// K_02
cv::Mat left_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.597910e+02, 0.000000e+00, 6.960217e+02, 
                                                            0.000000e+00, 9.569251e+02, 2.241806e+02, 
                                                            0.000000e+00, 0.000000e+00, 1.000000e+00);

// K_03
cv::Mat right_rgb_camera_matrix = (cv::Mat_<double>(3,3) << 9.037596e+02, 0.000000e+00, 6.957519e+02, 
                                                            0.000000e+00, 9.019653e+02, 2.242509e+02, 
                                                            0.000000e+00, 0.000000e+00, 1.000000e+00);

Mat FindEssentialMatrix(Mat fundamental_mat){
    if(DEBUG_PRINT){
        cout<<"fundamental_mat.size(): "<<fundamental_mat.size()<<endl;
    }
    return right_rgb_camera_matrix.t() * fundamental_mat * left_rgb_camera_matrix;

}