// for std
#include <iostream>
// for opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>
// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>

#define IMAGE_SHOW 0
#define DEBUG_PRINT 0


using namespace std;
using namespace cv;

// 寻找两个图像中的对应点，像素坐标系
// 输入：img1, img2 两张图像
// 输出：points1, points2, 两组对应的2D点
int findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 );

// 相机内参
// double cx = 325.5;
// double cy = 253.5;
// double fx = 518.0;
// double fy = 519.0;

// kitti rectify 后的内参
double cx = 609.5593;
double cy = 172.854;
double fx = 721.5377;
double fy = 721.5377;

bool LessSort (DMatch a, DMatch b) { return (a.distance < b.distance); }

// 将vector中的KeyPoint转化为Point2f
void ConvertKeypointsVector(std::vector<KeyPoint>src, std::vector<cv::Point2f>& dst){
    Point2f point;
    for(int i = 0; i<src.size(); i++){
        point.x = src[i].pt.x;
        point.y = src[i].pt.y;
        dst.push_back(point);
    }
}


int  findCorrespondingPoints( const cv::Mat& img_1, const cv::Mat& img_2, 
                              vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 , int choose_match_num)
{

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    if(IMAGE_SHOW){
        // Mat outimg1;
        // drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        // imshow("ORB特征点",outimg1);
    }

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> matches;
    vector<DMatch> good_matches;
    //BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, matches );


    sort(matches.begin(), matches.end(), LessSort);

    if(DEBUG_PRINT){
        for(auto match : matches){
            cout<<match.distance<<endl;
        }
    }

    vector<KeyPoint> _keypoints_left, _keypoints_right;
    int num_keypoints = choose_match_num;
    cout<<"num_keypoints before filter: "<<matches.size()<<endl;
    // assert(matches.size() >= num_keypoints); 
    // if(matches.size() < num_keypoints) num_keypoints = matches.size();
    for( size_t m = 0; m < num_keypoints; m++ ){
        int i1 = matches[m].queryIdx;
        int i2 = matches[m].trainIdx;
        _keypoints_left.push_back ( keypoints_1[i1] );
        _keypoints_right.push_back ( keypoints_2[i2] );
        good_matches.push_back(matches[m]);
    }

    if(IMAGE_SHOW){
        Mat outimg2_left, img_goodmatch;
        // drawKeypoints( img_1, keypoints_left, outimg2_left, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
        // imshow("ORB特征点(前n个) left",outimg2_left);
        drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
        imshow ( "优化后匹配点对", img_goodmatch );
        waitKey(0);
    }

    ConvertKeypointsVector(_keypoints_left, points1);
    ConvertKeypointsVector(_keypoints_right, points2);

    
    return true;
}


// 使用方法： ./ba ${匹配点的数量}
// 选择 200+ 对匹配可以获得比较好的效果，默认为240对
// 目前仅仅支持在rectified后的KITTI图像上进行，如果更改数据集需要手动修改全局变量 cx, cy, fx, fy

int main( int argc, char** argv ){

    cv::Mat img1 = cv::imread( "../test_images/kitti_test_01/image_02/data/left.png" ); 
    cv::Mat img2 = cv::imread( "../test_images/kitti_test_01/image_03/data/right.png" ); 
    
    int choose_match_num;
    if(argc == 2)
        choose_match_num = atoi(argv[1]);
    else 
        choose_match_num = 240;

    cout<<"choose_match_num: "<<choose_match_num<<endl;

    vector<cv::Point2f> pts1, pts2;
    if ( findCorrespondingPoints( img1, img2, pts1, pts2, choose_match_num) == false )
    {
        cout<<"匹配点不够！"<<endl;
        return 0;
    }
    cout<<"找到了"<<pts1.size()<<"组对应特征点。"<<endl;

    g2o::SparseOptimizer optimizer;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;  // 这里是9还是6？
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    // g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);



    // 添加节点
    // 两个位姿节点
    for ( int i=0; i<2; i++ )
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if ( i == 0)
            v->setFixed( true ); // 第一个点固定为零
        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
    }
    // 很多个特征点的节点
    // 以第一帧为准
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        // 由于深度不知道，只能把深度设置为1了
        double z = 1;
        double x = ( pts1[i].x - cx ) * z / fx; 
        double y = ( pts1[i].y - cy ) * z / fy; 
        v->setMarginalized(true);
        v->setEstimate( Eigen::Vector3d(x,y,z) );
        optimizer.addVertex( v );
    }

    // 准备相机参数
    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );


    // 准备边
    // 第一帧
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    // 第二帧
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    cout<<"优化完毕"<<endl;

    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose="<<endl<<pose.matrix()<<endl;

    // 以及所有特征点的位置
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        // cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        // cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }
    
    // 估计inlier的个数
    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 )
        {
            // cout<<"error = "<<e->chi2()<<endl;
        }
        else 
        {
            inliers++;
        }
    }
    
    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
    optimizer.save("ba.g2o");

    // return 0;


    return 0; 
}


