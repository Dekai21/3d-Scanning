#include"include.h"
#define DEBUG_PRINT 0
#define IMAGE_SHOW 1

using namespace cv;
using namespace std;

int main(int argc, char*argv[]){


    // tuyang 801
    ir_distortion_coeffs = (cv::Mat_<double>(1,12)<<0.147751, 0.402135, -0.000574, -0.000963,
                                                    0.134576, 0.403372, 0.385723, 0.254154,
                                                    0.001764, -0.000431, 0.000827, -0.000298);
    
    ir2_distortion_coeffs = (cv::Mat_<double>(1,12) << -0.406042, -0.268743, -0.001174, -0.001153,
                                                       -0.029927, -0.150741, -0.416733, -0.076622,
                                                       0.002466, 0.000227, 0.001892, -0.000645);

    return 0; 
}