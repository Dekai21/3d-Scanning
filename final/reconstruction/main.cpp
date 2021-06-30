#include "include.h"
#define DEBUG_PRINT 1
#define IMAGE_SHOW 1

using namespace std;
using namespace cv;


int main(int argc, char*argv[]){

    struct Dataset dataset;
    dataset.name = KITTI_TEST;  // 选择所需要的数据集
    // dataset.name = MATLAB_TEST;
    dataset.rectified = 1;
    dataset.distort = 0;
    dataset.given_points = 0; 

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
        
        // // 如果读取的是undistorted和rectified的图像可以注销该部分
        if(dataset.distort == 1){
            Mat undistort_1, undistort_2;
            Undistort(img_1, img_2, undistort_1, undistort_2);
            img_1 = undistort_1;
            img_2 = undistort_2; // swallow copy or deep copy?
        }

        vector<Point2f> keypoints_left; 
        vector<Point2f> keypoints_right;
        size_t num_keypoints = 40;

        if(dataset.given_points == 1){
            GetPoints(keypoints_left, keypoints_right, dataset);

            // show given points
            for(auto point:keypoints_left){
                cv::circle(img_1, point, 5, cv::Scalar(0, 0, 255), 1, cv::LINE_8, 0);
                imshow("img_1", img_1);            
            }
            waitKey(0);
        }
        else{
            OrbDetector(img_1, img_2, keypoints_left, keypoints_right, num_keypoints);
            // if(DEBUG_PRINT) cout<<"keypoints_left.size(): "<<keypoints_left.size()<<endl;
        }        
                
        Mat fundamental_mat = findFundamentalMat(keypoints_left, keypoints_right,  cv::FM_8POINT);
        // cout<<"fundamental_mat: " << fundamental_mat << endl;
        Mat essential_mat = FindEssentialMatrix(fundamental_mat, dataset);
        // cout<<"essential_mat: "<<essential_mat<<endl;

        // 由本质矩阵恢复出R和T
        Mat R1, R2, T;
        decomposeEssentialMat(essential_mat, R1, R2, T);

        // 选择正确的R和T
        struct transformation transformation = RecoverRT(R1, R2, T, keypoints_left, keypoints_right, dataset);
        if(DEBUG_PRINT){
            cout<<"R: "<<transformation.R<<endl;
            cout<<"t: "<<transformation.t<<endl<<endl;
        }
        

        // 根据R和T对图像进行rectify

        // 对rectify后的图像进行disparity计算
    }



    return 0;
}
