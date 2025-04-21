#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_map/voronoi_path_generator.h>
#include <tuw_voronoi_map/thinning.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <string>

using namespace cv;

namespace voronoi_map
{

    VoronoiPathGenerator::VoronoiPathGenerator()
    {

    }

    /**
     * @brief 准备地图数据
     *
     * 该函数负责将输入地图转换为适合路径规划的格式，包括图像类型转换、阈值化处理、腐蚀操作等。
     *
     * @param _map 输入的地图数据，类型为Mat
     * @param _des 处理后的地图数据，类型为Mat
     * @param erodeSize 腐蚀操作的大小，默认为1
     */
    void VoronoiPathGenerator::prepareMap(const Mat& _map, Mat& _des, int erodeSize)
    {
        static Mat srcMap;
        _map.convertTo(srcMap, CV_8UC1); // 转换到8位单通道图像

        // 打印地图大小信息
        std::cout << "Map size: " << srcMap.cols << "x" << srcMap.rows << std::endl; //1280x720

        for(int i = 0; i < srcMap.cols * srcMap.rows; i++)
        {
            if((signed char)_map.data[i] < 0)  srcMap.data[i] = 100;
        }

        _des = srcMap;

        cv::bitwise_not(srcMap, srcMap); // 反转图像
        cv::threshold(srcMap, _des, 10, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);// 阈值化处理，将图像转换为二值图0黑,255白
        
        if(erodeSize <= 0){
            erodeSize = 1;
        }
        if(erodeSize > 0){
            // 创建椭圆结构元素，用于膨胀(腐蚀)操作，去除边缘毛刺
            cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                        cv::Size( 2*erodeSize + 1, 2*erodeSize+1 ),
                                        cv::Point( erodeSize, erodeSize ) );
    
            cv::erode(_des, _des, element);
        }
        
        //测试，添加障碍物
        //1.寻找宽阔地区 2.在宽阔地区添加障碍物
        float robot_size = 7; //机器人直径7cm
        float threshold = robot_size*7; // 添加障碍物与原有障碍物的距离，越大提取出路径离原有障碍物越远
        int area_width = 10; // 人为添加障碍物的大小，单位像素点数 像素点数*map_resolution=长度cm
        int area_height = 10; // 障碍物越小，得到路径越密集
        // threshold *5 area 2 ; *7 10
        cv::Mat distField;
        cv::distanceTransform(_des, distField, cv::DIST_L2, 3);
        // 测试距离场值
        // std::cout << "Distance at (100, 100): " << distField.at<float>(100, 100) << std::endl;

        for(int y = area_height/2; y < distField.rows-area_height/2; y++) {
            for(int x = area_width/2; x < distField.cols-area_width/2; x++) {                
                if((distField.at<float>(y,x) > threshold)&&(_des.at<uchar>(y,x) == 255)) {
                    // // 创建一个圆形区域
                    // cv::circle(_des, cv::Point(x, y), area_width/2, 0, -1); // 用黑色填充圆形区域
                    // 创建一个矩形区域
                    cv::Rect area(x-area_width/2, y+area_height/2, area_width, area_height);
                    cv::rectangle(_des, area, cv::Scalar(0), -1); // 用黑色填充区域
                    cv::distanceTransform(_des, distField, cv::DIST_L2, 3);
                    x += area_width/2+1; // 跳过已处理的区域
                    if (x >= distField.cols-area_width/2) break; // 如果x超出范围，则跳出内层循环
                }
            }
        }
        
    }


    void VoronoiPathGenerator::computeDistanceField(const cv::Mat& _map, cv::Mat& _distField)
    {
        //计算一个给定地图（二值图像）的距离场，其中每个像素的值表示该像素到最近的障碍物（前景像素）的欧几里得距离。
        cv::distanceTransform(_map, _distField, cv::DIST_L2, 3);
    }

    void VoronoiPathGenerator::computeVoronoiMap(const cv::Mat& _map, cv::Mat& _voronoiMap) {

        Mat srcMap = _map;
        srcMap.convertTo(_voronoiMap, CV_8UC1, 0.0);
        voronoi_map::greyscale_thinning(srcMap, _voronoiMap);
        cv::threshold(_voronoiMap, _voronoiMap, 1, 255, cv::THRESH_BINARY);
        voronoi_map::sceletonize(_voronoiMap, _voronoiMap);
       
    }

}    