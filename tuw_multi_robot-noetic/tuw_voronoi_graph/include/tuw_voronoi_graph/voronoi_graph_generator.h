/*
 * Copyright (c) 2017, <copyright holder> <email>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef VORONOI_GRAPH_GENERATOR_H
#define VORONOI_GRAPH_GENERATOR_H

#include <ros/ros.h>
#include <tuw_voronoi_graph/segment.h>
#include <tuw_voronoi_graph/segment_expander.h>
#include <opencv2/opencv.hpp>

namespace tuw_graph
{
    class VoronoiGraphGenerator
    {
        private:
            std::vector<cv::Point> tracePath(const cv::Mat& voronoiPath, cv::Point start, cv::Point prev, int maxLength);
            
            // 添加新的私有函数声明
            bool isValidPoint(const cv::Point& p, const cv::Mat& map);
            void simplifyPath(const std::vector<cv::Point>& input, 
                            std::vector<cv::Point>& output, 
                            double tolerance);
            double pointLineDistance(const cv::Point& p, 
                                   const cv::Point& a, 
                                   const cv::Point& b);
            bool isDeadEnd(const std::vector<cv::Point>& path,
                          const std::vector<cv::Point>& crossings,
                          float threshold);
            bool canConnect(const cv::Point& p1, 
                        const cv::Point& p2, 
                        const cv::Mat& map,
                        const cv::Mat& distMap);
            std::vector<cv::Point> generatePathPoints(const cv::Point& start,
                                                    const cv::Point& end,
                                                    const cv::Mat& distMap);
            
            // 添加方向数组作为类成员
            const int dx[8] = {-1,0,1,-1,1,-1,0,1};
            const int dy[8] = {-1,-1,-1,0,0,1,1,1};

        public:     
            VoronoiGraphGenerator();
            std::vector<Segment> calcSegments(cv::Mat &_map, 
                                            cv::Mat &_distField, 
                                            cv::Mat &_voronoiPath, 
                                            float * potential, 
                                            float _path_length, 
                                            float _optimizeCrossingPixels, 
                                            float _optimizeEndSegmentsPixel);
    };
}
#endif // PLANNER_NODE_H
