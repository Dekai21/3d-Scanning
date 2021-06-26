#include "include.h"

using namespace std;
using namespace cv;

int main(int argc, char*argv[]){

    // 读取图像，并对图像做undistortion操作
    String left = "../images/left.png";
    String right = "../images/right.png";
    
    // 读取相似度高的key points
    OrbDetector(left, right);

    // 根据这些key points进行R和T的计算

    // 根据R和T对图像进行rectify

    // 对rectify后的图像进行disparity计算

    return 0;
}