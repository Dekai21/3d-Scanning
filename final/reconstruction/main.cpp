#include "include.h"
#define DEBUG_PRINT 0

using namespace std;
using namespace cv;


void ConvertKeypointsVector(std::vector<KeyPoint>src, std::vector<cv::Point2f>& dst){
    Point2f point;
    for(int i = 0; i<src.size(); i++){
        point.x = src[i].pt.x;
        point.y = src[i].pt.y;
        dst.push_back(point);
    }
}

void tmp(){
    Mat x1 = Mat::ones(cv::Size(3,1), CV_64FC1);
    Mat x2_hat = Mat::ones(cv::Size(3,3), CV_64FC1);
    Mat tmp = x1 * x2_hat;
    cout<<"tmp: "<<tmp<<endl;
}

int main(int argc, char*argv[]){
    // tmp();

    // 参数设置， 后期可以改为从文件读入
    String left = "/home/dekai/datasets/2011_09_26_drive_0048_extract/2011_09_26/2011_09_26_drive_0048_extract/image_02/data/0000000009.png";
    String right = "/home/dekai/datasets/2011_09_26_drive_0048_extract/2011_09_26/2011_09_26_drive_0048_extract/image_03/data/0000000009.png";
    vector<KeyPoint> keypoints_left; 
    vector<KeyPoint> keypoints_right;
    size_t num_keypoints = 80;

    // 读取图像，并对图像做undistortion操作
    Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );
    Mat undistort_1, undistort_2;
    Undistort(img_1, img_2, undistort_1, undistort_2);
    img_1 = undistort_1;
    img_2 = undistort_2; // swallow copy or deep copy?
    
    // 读取相似度高的key points
    OrbDetector(img_1, img_2, keypoints_left, keypoints_right, num_keypoints);
    // if(DEBUG_PRINT) cout<<"keypoints_left.size(): "<<keypoints_left.size()<<endl;

    // 根据这些key points采用八点法得到本质矩阵
    std::vector<cv::Point2f> keypoints_left_, keypoints_right_;
    ConvertKeypointsVector(keypoints_left, keypoints_left_);
    ConvertKeypointsVector(keypoints_right, keypoints_right_);
    Mat fundamental_mat = findFundamentalMat(keypoints_left_, keypoints_right_);
    Mat essential_mat = FindEssentialMatrix(fundamental_mat);

    // 由本质矩阵恢复出R和T
    Mat R1, R2, T;
    decomposeEssentialMat(essential_mat, R1, R2, T);

    // 选择正确的R和T
    struct transformation transformation = RecoverRT(R1, R2, T, keypoints_left, keypoints_right);
    

    // 根据R和T对图像进行rectify

    // 对rectify后的图像进行disparity计算

    return 0;
}
