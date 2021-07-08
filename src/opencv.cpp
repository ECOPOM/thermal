#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <fmt/core.h>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat function(){
    cv::Mat image(500, 500, CV_8U, 50);
    return image;
}

int main(int argc, char * argv[]) {
    cv::Mat image; // create an empty image
    image = cv::imread("/home/bresilla/data/code/univ/SHEET/thermal/temp/bresilla.png");

    fmt::print(fmt::format("This image is {}x{}", image.rows, image.cols));

    cv::namedWindow("ORIGINAL IMAGE");
    cv::imshow("ORIGINAL IMAGE", image);

    cv::Mat result;
    cv::flip(image, result, 0);

    cv::namedWindow("RESULT IMAGE");
    cv::imshow("RESULT IMAGE", result);

    cv::waitKey();

    return 1;
}
