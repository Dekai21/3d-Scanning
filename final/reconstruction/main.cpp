#include "include.h"
#define DEBUG_PRINT 1
#define IMAGE_SHOW 0

using namespace std;
using namespace cv;

// 将vector中的KeyPoint转化为Point2f
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

// 从dirpath目录下分别读取左右图像的路径，并存入left_image_paths和right_image_paths中
void getFilesList(String dirpath, vector<String> &left_image_paths, vector<String> &right_image_paths){
    // String left_dir = dirpath + "/image_02";
    // String right_dir = dirpath + "/image_03";
    struct dirent * entry;
    DIR *left_dir = opendir((dirpath + "/image_02/data/").c_str());
    DIR *right_dir = opendir((dirpath + "/image_03/data/").c_str());
    vector<String> left_images;
    vector<String> right_images;

    String name;
    while ((entry = readdir(left_dir)) != NULL){
        if(entry->d_type != DT_DIR){
            name = entry->d_name;
            left_images.push_back(name);
        }
    }

    while ((entry = readdir(right_dir)) != NULL){
        if(entry->d_type != DT_DIR){
            name = entry->d_name;
            right_images.push_back(name);
        }
    }

    sort(left_images.begin(), left_images.end());
    sort(right_images.begin(), right_images.end());

    assert(left_images.size() == right_images.size());

    if(DEBUG_PRINT){
        cout<<"\n left images: "<<endl;
        for(auto left_image:left_images) cout<<left_image<<endl;
        cout<<"\n right images: "<<endl;
        for(auto right_image:right_images) cout<<right_image<<endl;

    }
    for(int i = 0; i<left_images.size(); i++){
        assert(left_images[i] == right_images[i]);
        left_image_paths.push_back(dirpath + "/image_02/data/" + left_images[i]);
        right_image_paths.push_back(dirpath + "/image_03/data/" + right_images[i]);
    }

}

// 手动给定特征点坐标
void GetPoints(vector<cv::Point2f>& keypoints_left, vector<cv::Point2f>& keypoints_right){
    // // matlab
    // float x1[12] = {10.0000, 92.0000, 8.0000, 92.0000, 289.0000, 354.0000, 289.0000, 353.0000, 69.0000, 294.0000, 44.0000, 336.0000};
    // float y1[12] = {232.0000, 230.0000, 334.0000, 333.0000, 230.0000, 278.0000, 340.0000, 332.0000, 90.0000, 149.0000, 475.0000, 433.0000};
    // float x2[12] = {123.0000, 203.0000, 123.0000, 202.0000, 397.0000, 472.0000, 398.0000, 472.0000, 182.0000, 401.0000, 148.0000, 447.0000};
    // float y2[12] = {239.0000, 237.0000, 338.0000, 338.0000, 236.0000, 286.0000, 348.0000, 341.0000,  99.0000, 153.0000, 471.0000, 445.0000};

    // rectified KITTI - 2011_09_26_drive_0113_sync - 0.png
    float x1[12] = {290.0, 385.0, 390.0, 426.0, 440.0, 428.0, 444.0, 962.0, 973.0, 216.0, 696.0, 132.0};
    float y1[12] = {111.0, 57.0, 186.0, 69.0, 98.0, 136.0, 165.0, 241.0, 55.0, 312.0, 207.0, 236.0};
    float x2[12] = {259.0, 370.0, 373.0, 410.0, 428.0, 413.0, 429.0, 893.0, 949.0, 126.0, 675.0, 114.0};
    float y2[12] = {111.0, 57.0, 187.0, 69.0, 98.0, 137.0, 165.0, 241.0, 57.0, 331.0, 209.0, 236.0};
    Point2f p;
    for(int i = 0; i<12; i++){
        p.x = x1[i];
        p.y = y1[i];
        keypoints_left.push_back(p);
        p.x = x2[i];
        p.y = y2[i];
        keypoints_right.push_back(p);
    }

}

int main(int argc, char*argv[]){
    // tmp();

    // 对kitti数据集中的img2和img3文件夹进行遍历
    // String dir_path = "/home/dekai/datasets/2011_09_26_drive_0048/2011_09_26_drive_0048_extract/2011_09_26/2011_09_26_drive_0048_extract";  // 畸变+未矫正
    // String dir_path = "/home/dekai/datasets/2011_09_26_drive_0048/2011_09_26_drive_0048_sync/2011_09_26/2011_09_26_drive_0048_sync";  // 去畸变+矫正
    // String dir_path = "/home/dekai/datasets/2011_09_26_drive_0113/2011_09_26_drive_0113_extract/2011_09_26/2011_09_26_drive_0113_extract";
    // String dir_path = "/home/dekai/datasets/2011_09_26_drive_0113/2011_09_26_drive_0113_sync/2011_09_26/2011_09_26_drive_0113_sync";
    // String dir_path = "/home/dekai/datasets/matlab";
    // String dir_path = "/home/dekai/datasets/Calibration_2";
    String dir_path = "/home/dekai/datasets/test_01";
    vector<String> left_image_paths, right_image_paths;
    getFilesList(dir_path, left_image_paths, right_image_paths);

    for(int i = 0; i<left_image_paths.size(); i++){
        // 参数设置， 后期可以改为从json文件读入
        // String left = "/home/dekai/datasets/2011_09_26_drive_0048_extract/2011_09_26/2011_09_26_drive_0048_extract/image_02/data/0000000009.png";
        // String right = "/home/dekai/datasets/2011_09_26_drive_0048_extract/2011_09_26/2011_09_26_drive_0048_extract/image_03/data/0000000009.png";
        String left = left_image_paths[i];
        String right = right_image_paths[i];
        vector<KeyPoint> keypoints_left; 
        vector<KeyPoint> keypoints_right;
        size_t num_keypoints = 40;

        // 读取图像
        Mat img_1 = imread ( left, CV_LOAD_IMAGE_COLOR );
        Mat img_2 = imread ( right, CV_LOAD_IMAGE_COLOR );
        if(IMAGE_SHOW){
            imshow("img_1", img_1);
            waitKey(0);
        }
        
        // // 如果读取的是undistorted和rectified的图像可以注销该部分
        // Mat undistort_1, undistort_2;
        // Undistort(img_1, img_2, undistort_1, undistort_2);
        // img_1 = undistort_1;
        // img_2 = undistort_2; // swallow copy or deep copy?
        
        // 读取相似度高的key points
        // OrbDetector(img_1, img_2, keypoints_left, keypoints_right, num_keypoints);
        // if(DEBUG_PRINT) cout<<"keypoints_left.size(): "<<keypoints_left.size()<<endl;

        // 根据这些key points采用八点法得到本质矩阵
        std::vector<cv::Point2f> keypoints_left_, keypoints_right_;
        ConvertKeypointsVector(keypoints_left, keypoints_left_);
        ConvertKeypointsVector(keypoints_right, keypoints_right_);
        GetPoints(keypoints_left_, keypoints_right_);
        // for(auto point:keypoints_left_){
        //     cv::circle(img_1, point, 5, cv::Scalar(0, 0, 255), 1, cv::LINE_8, 0);
        //     imshow("img_1", img_1);
        //     waitKey(0);
        // }
        Mat fundamental_mat = findFundamentalMat(keypoints_left_, keypoints_right_,  cv::FM_8POINT);
        // cout<<"fundamental_mat: " << fundamental_mat << endl;
        Mat essential_mat = FindEssentialMatrix(fundamental_mat);
        // cout<<"essential_mat: "<<essential_mat<<endl;

        // 由本质矩阵恢复出R和T
        Mat R1, R2, T;
        decomposeEssentialMat(essential_mat, R1, R2, T);

        // 选择正确的R和T
        struct transformation transformation = RecoverRT(R1, R2, T, keypoints_left_, keypoints_right_);
        if(DEBUG_PRINT){
            cout<<"R: "<<transformation.R<<endl;
            cout<<"t: "<<transformation.t<<endl<<endl;
        }
        

        // 根据R和T对图像进行rectify

        // 对rectify后的图像进行disparity计算
    }



    return 0;
}
