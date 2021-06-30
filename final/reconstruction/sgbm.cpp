#include "include.h"
#define DEBUG_PRINT 1
#define IMAGE_SHOW 1

using namespace std;
using namespace cv;

// 参考： https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html

int main(int argc, char*argv[]){

    struct Dataset dataset;
    dataset.name = KITTI_TEST;  // 选择所需要的数据集
    // dataset.name = MATLAB_TEST;
    dataset.rectified = 1;
    dataset.distort = 0;
    dataset.given_points = 1; 

    // 对kitti数据集中的img2和img3文件夹进行遍历
    String dir_path = GetDirPath(dataset);
    vector<String> left_image_paths, right_image_paths;
    getFilesList(dir_path, left_image_paths, right_image_paths);

    for(int i = 0; i<left_image_paths.size(); i++){
        // 参数设置， 后期可以改为从json文件读入
        String left = left_image_paths[i];
        String right = right_image_paths[i];

        // 读取图像
        Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );
        if(IMAGE_SHOW){
            imshow("img_1", img_1);
            waitKey(0);
        }

        int max_disp = 80; // 决定最近距离
        int wsize = 9;
        Mat left_disp;
        Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(5, max_disp, wsize);
        left_matcher->setP1(1*wsize*wsize);  
        left_matcher->setP2(96*wsize*wsize);  
        // left_matcher->setPreFilterCap(70);
        left_matcher->setMode(StereoSGBM::MODE_SGBM);  // MODE_HH4 performs the best in the experiment until now 
        // left_matcher->setDisp12MaxDiff(consistency); 
        // left_matcher->setUniquenessRatio(20);
        // left_matcher->setSpeckleRange(1);
        // left_matcher->setSpeckleWindowSize(2000);           
        // matching_time = (double)getTickCount();
        left_matcher-> compute(img_1, img_2, left_disp); 
        left_disp = left_disp / 6;

        cv::imshow("left_disparity", left_disp);
        cv::imwrite("left_disp.png", left_disp);
        cv::waitKey(0);

    }

    return 0;
}