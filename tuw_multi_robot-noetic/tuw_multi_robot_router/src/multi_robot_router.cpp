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

#include <tuw_global_router/multi_robot_router.h>
#include <chrono>
#include <iostream>
#include <ros/ros.h>

std::vector<uint32_t> all_try_robot;

namespace multi_robot_router
{
//TODO Multithreaded

    MultiRobotRouter::MultiRobotRouter(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotDiameter) :  RouteGenerator(), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
    {
        setRobotNr(_nr_robots);
        robotDiameter_ = _robotDiameter;
        route_coordinator_ = &rct_;
    }
    
    MultiRobotRouter::MultiRobotRouter(const uint32_t _nr_robots) :  RouteGenerator(), priority_scheduler_(_nr_robots), speed_scheduler_(_nr_robots)
    {
        setRobotNr(_nr_robots);
        std::vector<uint32_t> robotDiameter(_nr_robots, 0);
        robotDiameter_ = robotDiameter;
        route_coordinator_ = &rct_;
    }


    void MultiRobotRouter::setCollisionResolver(const SegmentExpander::CollisionResolverType cRes)
    {
        cResType_ = cRes;
    }


    void MultiRobotRouter::setRobotNr(const uint32_t _nr_robots)
    {
        nr_robots_ = _nr_robots;
        priority_scheduler_.reset(_nr_robots);
    }

    void MultiRobotRouter::setRobotDiameter(const std::vector< uint32_t > &_diameter)
    {
        robotDiameter_.clear();
        robotDiameter_ = _diameter;
        min_diameter_ = std::numeric_limits<uint32_t>::max();

        for(const uint32_t d : robotDiameter_)
        {
            min_diameter_ = std::min(min_diameter_, d);
        }
    }

    void MultiRobotRouter::resetAttempt(const std::vector< Segment > &_graph)
    {
        route_coordinator_->reset(_graph, nr_robots_);
        priority_scheduler_.reset(nr_robots_);
        speed_scheduler_.reset(nr_robots_);
        robotCollisions_.clear();
        robotCollisions_.resize(nr_robots_);
        priorityScheduleAttempts_ = 0;
        speedScheduleAttempts_ = 0;
    }

    void MultiRobotRouter::setRobotPairs(const std::vector<RobotPair> &pairs) {
    robot_pairs_ = pairs;
    }

    const uint32_t MultiRobotRouter::getPriorityScheduleAttempts() const
    {
        return priorityScheduleAttempts_;
    }

    const uint32_t MultiRobotRouter::getSpeedScheduleAttempts() const
    {
        return speedScheduleAttempts_;
    }

    bool MultiRobotRouter::getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, std::vector<std::vector< Checkpoint>> &routingTable, const float &_timeLimit)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> tstart = std::chrono::high_resolution_clock::now();

        resetAttempt(_graph);
        route_coordinator_->setStartSegments(_startSegments);
        route_coordinator_->setGoalSegments(_goalSegments);

        maxIterationsSingleRobot_ =  _graph.size();

        srr.setCollisionResolver(cResType_);
        srr.initSearchGraph(_graph, min_diameter_);

        std::vector<std::vector<RouteVertex>> routeCandidates;
        routeCandidates.resize(nr_robots_);

        std::vector<uint32_t> priorityList = priority_scheduler_.getActualSchedule();
        std::vector<float> speedList = speed_scheduler_.getActualSpeeds();
        uint32_t firstSchedule = 0;
        bool found = false;
        uint32_t lastPlannedRobot;
        float duration = 0;
        Eigen::Vector2d temp;

        std::ofstream SegPos;
        SegPos.open("/home/zjw/catkin_ws/src/tuw_multi_robot_xin/tuw_multi_robot-noetic/tuw_multi_robot_router/vertices position.txt");
        for(int i = 0;i < _graph.size();i++)
        {
            temp = _graph[i].getStart();
            SegPos << temp(0) << " ";
            SegPos << temp(1) << std::endl;
        }

        std::ifstream goal_pos;
        goal_pos.open("/home/zjw/catkin_ws/src/tuw_multi_robot_xin/tuw_multi_robot-noetic/tuw_multi_robot_router/goal position.txt");
        std::vector<Eigen::Vector2d> goals_;      //goals position
        goals_.resize(_startSegments.size());
        for(int i = 0;i < goals_.size();i++)
        {
            for(int j = 0;j < 2;j++)
            {    
                goal_pos >> goals_[i](j);
            }
        }

        std::ifstream start_pos;
        start_pos.open("/home/zjw/catkin_ws/src/tuw_multi_robot_xin/tuw_multi_robot-noetic/tuw_multi_robot_router/start position.txt");
        std::vector<Eigen::Vector2d> start_;      //goals position
        start_.resize(_startSegments.size());
        for(int i = 0;i < start_.size();i++)
        {
            for(int j = 0;j < 2;j++)
            {    
                start_pos >> start_[i](j);
            }
        }
        
        std::cout << std::endl <<"_startSegments: ";
        for(int i = 0;i < _startSegments.size();i++)
            std::cout << _startSegments[i] << " ";
        std::cout << std::endl << "_goalSegments: ";
        for(int i = 0;i < _goalSegments.size();i++)
            std::cout << _goalSegments[i] << " ";
        std::cout << std::endl << std::endl;
                
        std::cout << "we try the original method!!! " << std::endl;
        do
        {
            int32_t firstRobot = -1;
            do
            {
                //Find first schedule to replan if speed rescheduling was active
                //(used for removing from path coordinator)
                if(firstRobot != -1)
                {
                    for(uint32_t i = 0; i < priorityList.size(); i++)
                    {
                        if(priorityList[i] == firstRobot)
                            firstSchedule = i;
                    }
                }
                
                found = planPaths(priorityList, speedList, _startSegments, _goalSegments, firstSchedule, routeCandidates, lastPlannedRobot);
                speedScheduleAttempts_++;
                std::chrono::time_point<std::chrono::high_resolution_clock>  tgoal = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::milliseconds>(tgoal - tstart).count();
                //////duration /= 1000;
                // if(!found)
                // {
                //     std::cout << std::endl;
                //     std::cout << "rescheduleSpeeds try: ";
                //     for(int i=0;i < speedList.size();i++)
                //         std::cout << speedList[i] << " ";
                //     std::cout << std::endl << std::endl;
                // }
                if(found)
                {
                    std::cout << std::endl;
                    std::cout << "found resolution" << std::endl;
                }
            }
            while(useSpeedRescheduler_ && duration < _timeLimit && !found && speed_scheduler_.rescheduleSpeeds(lastPlannedRobot, srr.getRobotCollisions(), speedList, firstRobot) );
            priorityScheduleAttempts_++;
            if(!found)
            {
                //std::cout << "=================================" << std::endl;
                //std::cout << "reschedulePriorities try: ";
                //for(int i=0;i < priorityList.size();i++)
                    //std::cout << priorityList[i] << " ";
                //std::cout << std::endl << std::endl;
                speed_scheduler_.reset(speedList.size());
                speedList = speed_scheduler_.getActualSpeeds();
            }
        }
        while(usePriorityRescheduler_ && duration < _timeLimit && !found && priority_scheduler_.reschedulePriorities(lastPlannedRobot, srr.getRobotCollisions(), priorityList, firstSchedule));
        std::cout << std::endl;
        std::cout << "Last pritorityList: ";
        for(int i=0;i < priorityList.size();i++)
            std::cout << priorityList[i] << " ";
        std::cout << std::endl;
        std::cout << "Last speedList: ";
        for(int i=0;i < speedList.size();i++)
            std::cout << speedList[i] << " ";
        std::cout << std::endl << std::endl;
        if(!found)
        {
            speed_scheduler_.reset(speedList.size());
            std::cout << "We try a new planning method!!! " << std::endl << std::endl;
            routeCandidates.clear();
            routeCandidates.resize(_startSegments.size());
            found = planPaths_push(_graph, goals_, start_, _goalSegments, _startSegments, routeCandidates, speedList);
        }
        routingTable = generatePath(routeCandidates, *route_coordinator_);                        
        return found;
    }

    bool MultiRobotRouter::planPaths(const std::vector<uint32_t> &_priorityList, const std::vector<float> &_speedList, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, const uint32_t _firstSchedule, std::vector<std::vector<RouteVertex>> &_routeCandidates, uint32_t &_robot)
    {
        bool found = false;

        //Remove only schedules (robots) which have to be replanned
        for(uint32_t i = _firstSchedule; i < nr_robots_; i++)
        {
            _robot = _priorityList[i];
            route_coordinator_->removeRobot(_robot);
        }

        //Find a plan for each robot with no plan
        for(uint32_t i = _firstSchedule; i < nr_robots_; i++)
        {
            found = false;
            _robot = _priorityList[i];

            //route_coordinator_->setActive(_robot);

            RouteCoordinatorWrapper rcWrapper(_robot, *route_coordinator_);
            //Worst case scenario: Search whole graph once + n * (move through whole graph to avoid other robot) -> graph.size() * (i+1) iterations
            if(!srr.getRouteCandidate(_startSegments[_robot], _goalSegments[_robot], rcWrapper, robotDiameter_[_robot], _speedList[_robot], _routeCandidates[_robot], maxIterationsSingleRobot_ * (i + 1)))
            {
                //ROS_INFO("Failed Robot");
                std::cout << "failed to get single robot's route" << std::endl;
                robotCollisions_[_robot] = srr.getRobotCollisions();
                robotCollisions_[_robot].resize(nr_robots_, 0);
                break;
            }

            robotCollisions_[_robot] = srr.getRobotCollisions();
            robotCollisions_[_robot].resize(nr_robots_, 0);

            if(!route_coordinator_->addRoute(_routeCandidates[_robot], robotDiameter_[_robot], _robot))
            {
                std::cout << "Failed coordinator" << std::endl;
                ROS_INFO("Failed coordinator");
                break;
            }

            // Check hose length restriction for paired robots
            for (const RobotPair &pair : robot_pairs_) {
                if (pair.robot1 == _robot || pair.robot2 == _robot) {
                    uint32_t other_robot = (pair.robot1 == _robot) ? pair.robot2 : pair.robot1;
                    if (!checkHoseLengthRestriction(_routeCandidates[_robot], _routeCandidates[other_robot], pair.max_hose_length)) {
                        std::cout << "Failed Hose Length Restriction" << std::endl;
                        return false;
                    }
                }
            }
            found = true;
        }

        return found;
    }

    bool MultiRobotRouter::checkHoseLengthRestriction(const std::vector<RouteVertex> &route1, const std::vector<RouteVertex> &route2, float max_hose_length) const {
        for (size_t i = 0; i < std::min(route1.size(), route2.size()); ++i) {
            Eigen::Vector2d pos1 = route1[i].getPosition();
            Eigen::Vector2d pos2 = route2[i].getPosition();
            float distance = (pos1 - pos2).norm();
            if (distance > max_hose_length) {
                return false;
            }
        }
        return true;
    }
    
    /**
     * @brief 为多个机器人规划路径并推送路径结果
     *
     * 根据给定的路径段、目标位置、起始位置、目标段和起始段等信息，为多个机器人规划路径，并将路径候选集推送到传入的容器中。
     *
     * @param _graph         路径段集合，类型为std::vector<Segment>
     * @param _goalpos       目标位置集合，类型为std::vector<Eigen::Vector2d>
     * @param _startpos      起始位置集合，类型为std::vector<Eigen::Vector2d>
     * @param _goalSegments  目标段ID集合，类型为std::vector<uint32_t>
     * @param _startSegments 起始段ID集合，类型为std::vector<uint32_t>
     * @param _routeCandidates 路径候选集容器，类型为std::vector<std::vector<RouteVertex>>
     * @param _speedList     机器人速度列表，类型为std::vector<float>
     *
     * @return 如果成功规划路径并推送结果，则返回true；否则返回false。
     */
    bool MultiRobotRouter::planPaths_push(const std::vector<Segment> &_graph, const std::vector<Eigen::Vector2d> &_goalpos, \
    const std::vector<Eigen::Vector2d> &_startpos, const std::vector<uint32_t> &_goalSegments, \
    const std::vector<uint32_t> &_startSegments, std::vector<std::vector<RouteVertex>> &_routeCandidates, \
    const std::vector<float> &_speedList)
    {    
        // std::vector<uint32_t> all_try_robot;
        Eigen::Vector2d temp1;
        Eigen::Vector2d temp2;
        Eigen::Vector2d temp3;        
        bool eq;
        uint32_t crossing;
        uint32_t crossing_end;
        double dis;
        double angle;
        std::vector<std::vector<Segment>> crossing_segment(3);   //0 is right, 1 is up, 2 is left
        std::vector<double> goal_distance(_goalpos.size());
        std::vector<double> start_distance(_goalpos.size());
        std::vector<double> robot_angle(_startpos.size());
        std::vector<std::vector<uint32_t>> all_robot;   //position information
        std::vector<double> temp_goal_distance;
        std::vector<double> temp;
        std::vector<uint32_t> robot_in_crossing(_goalpos.size());
        std::vector<std::vector<RouteVertex>> temp_candidate(_goalpos.size());
        uint32_t first;
        uint32_t first_robot_pos;
        uint32_t first_goal_pos;
        uint32_t robot_start;
        uint32_t robot_goal;
        uint32_t get;
        uint32_t cross_go;
        bool get_route = false;
        uint32_t add;
        uint32_t find;

        std::cout << "=========================================" << std::endl;
        std::cout << "robot in goal: ";
        for(int i = 0;i < all_try_robot.size();i++)
            std::cout << all_try_robot[i] << " ";
        std::cout << std::endl << "=========================================" << std::endl;
        
        //寻找交叉点
        std::vector<bool> goal_assigned(_goalpos.size(),false);
        for(int i = 0;i < _graph.size();i++)
        {
            eq = false;
            temp1 = _graph[i].getStart(); 
            for(int j = i+1;j < _graph.size();j++)
            {
                temp2 = _graph[j].getStart();
                for(int k = 0;k < 2;k++)
                {
                    if(temp1(k) == temp2(k))
                        eq = true;
                    else
                    {
                        eq = false;
                        break;
                    }
                }
                if(eq == true)
                {
                    crossing_end = j;
                    break;
                }
            }
            if(eq == true)
            {
                crossing = i;
                break;
            }
        }
        // 0-right, 1-up, 2-left
        crossing_segment[0].insert(crossing_segment[0].begin(), _graph.begin() + crossing + 1, _graph.begin() + crossing_end);
        crossing_segment[1].insert(crossing_segment[1].begin(), _graph.begin(), _graph.begin() + crossing - 1);
        crossing_segment[2].insert(crossing_segment[2].begin(), _graph.begin() + crossing_end + 1, _graph.end() - 2);
        //std::cout << std::endl;
        // std::cout << "crossing: " << crossing << std::endl;
        // std::cout << std::endl;
        // std::cout << "crossing_end: " << crossing_end << std::endl;

        temp2 = _graph[crossing].getStart();
        
        // 需要根据对应map.yaml文件修改，此处仅为T-intersection地图。
        double origin_x = -15.0;  
        double origin_y = -15.0;  
        double resolution = 0.05;
        

        // 转换交叉点坐标
        temp2(0) = temp2(0) * resolution + origin_x;  // x_物理
        temp2(1) = temp2(1) * resolution + origin_y;  // y_物理

        std::cout << "crossing point: ("<< temp2(0) << "," << temp2(1) << ")"<<std::endl;
        //check the distance between crossing and goal && check angle of robots
        // 计算交叉点到目标和起始点的距离，以及起始点和交叉点的角度
        for(int i = 0;i < _goalpos.size();i++)
            {
                temp1 = _goalpos[i];
                temp3 = _startpos[i];
                dis = (temp1(0)-temp2(0)) * (temp1(0)-temp2(0)) + (temp1(1)-temp2(1)) * (temp1(1)-temp2(1));
                angle = atan2((temp3(1)-temp2(1)),(temp3(0)-temp2(0))) * 180/PI;
                goal_distance[i] = sqrt(dis);
                robot_angle[i] = angle;        
                dis = (temp3(0)-temp2(0)) * (temp3(0)-temp2(0)) + (temp3(1)-temp2(1)) * (temp3(1)-temp2(1));
                start_distance[i] = sqrt(dis); 
            }
            std::vector<double>::iterator biggest = std::max_element(goal_distance.begin(), goal_distance.end());
            first = std::distance(goal_distance.begin(),biggest);     //firstly address
            std::cout << "the farest robot to goal: " << first << std::endl << std::endl;
            //std::cout << "the angle of robots: ";
            // for(int i = 0;i < robot_angle.size();i++)
            //     std::cout << robot_angle[i] << " ";
            // std::cout << std::endl;

            //确定机器人相对于交叉点的位置
            for(int i = 0;i < robot_angle.size();i++)
            {
                std::cout << "robot_angle: " << robot_angle[i] << std::endl;
                if(abs(robot_angle[i]) < 20)
                {
                    robot_in_crossing[i] = 0; //right
                }
                else if(abs(robot_angle[i] - 90) < 20)
                {
                    robot_in_crossing[i] = 1; //up
                }
                else
                    robot_in_crossing[i] = 2; //left
            }
            for(int i = 0;i < 3;i++)
            {
                for(int j = 0;j < robot_angle.size();j++)
                {
                    if(robot_in_crossing[j] == i)
                    {
                        std::cout << "crossing" << i <<":"<< j << std::endl;
                    }
                }
                
            }
            
            all_robot.clear();
            all_robot.resize(_goalpos.size());
            // 找到同一个区域、起始点距离交叉点更近的机器人，添加到all_robot[i]，all_robot由近到远排序
            for(int i = 0;i < _goalpos.size();i++)      //find robots with same position in front of a robot 
            {
                for(int j = 0;j < _goalpos.size();j++)
                {
                    // if(i != j && std::abs(robot_angle[i] - robot_angle[j]) < 20 && start_distance[i] > start_distance[j])
                    if(i != j && robot_in_crossing[i]==robot_in_crossing[j] && start_distance[i] > start_distance[j])
                        all_robot[i].push_back(j);
                }

            }
            for(int i = 0;i < all_robot.size();i++)
            {       
                std::cout << "before " << i << ": ";
                for(int j = 0;j < all_robot[i].size();j++)
                {
                    std::cout << all_robot[i][j] << " ";
                }
                std::cout << std::endl;
            }
            //  std::cout << std::endl;
            
            //std::cout << "the angle of goals: ";
            eq = false;
            temp_goal_distance = goal_distance;
            for(int i = 0;i < _goalpos.size();i++)
            {
                temp1 = _goalpos[first];            
                angle = atan2(temp1(1) - temp2(1), temp1(0) - temp2(0)) * 180/PI;   // 
                //std::cout << angle << " "; 
                //目标和机器人在同一区域       
                if(abs(robot_angle[first] - angle) < 20)       //goal and robot in the same place
                {
                    temp_goal_distance[first] = 0;
                    if(temp_goal_distance.size() - 1 == i)
                    {
                        std::cout << std::endl;
                        std::cout << "no first goal to try!!!" << std::endl << std::endl;
                        return false;
                    }
                    std::vector<double>::iterator biggest = std::max_element(temp_goal_distance.begin(), temp_goal_distance.end());
                    first = std::distance(temp_goal_distance.begin(),biggest);
                }
                else //不在
                {
                    for(int j = 0;j < all_try_robot.size();j++)
                    {
                        eq = false;
                        if(first == all_try_robot[j])
                        {
                            eq = true;
                            temp_goal_distance[first] = 0;
                            std::vector<double>::iterator biggest = std::max_element(temp_goal_distance.begin(), temp_goal_distance.end());
                            first = std::distance(temp_goal_distance.begin(),biggest);
                            break;
                        }
                    }
                    if(eq == false)
                        break;
                }
            }
            std::cout << std::endl;
            std::cout << "first: " << first << std::endl;

            all_try_robot.push_back(first);            
            
            // for(int i = 0;i < 3;i++)
            // {
            //     std::cout << "the position of crossing " << i << ": ";
            //     for(int j = 0;j < crossing_segment[i].size();j++)
            //         std::cout << crossing_segment[i][j].getSegmentId() << " ";
            //     std::cout << std::endl;
            // }

            robot_goal = _goalSegments[first];
            robot_start = _startSegments[first];
            for(int i = 0;i < 3;i++)      //find robot and goal position in the crossing
            {
                for(int j = 0;j < crossing_segment[i].size();j++)
                {
                    if(robot_goal == crossing_segment[i][j].getSegmentId())
                    {
                        first_goal_pos = i;
                    }
                    if(robot_start == crossing_segment[i][j].getSegmentId())
                    {
                        first_robot_pos = i;
                    }
                }

            }

            std::cout << "first_robot_pos: " << first_robot_pos << std::endl;        
            std::cout << "first_goal_pos: " << first_goal_pos << std::endl << std::endl;

            std::vector<uint32_t> robot_in_goal;
            robot_in_goal.clear();
            //检查目标区域是否有其他机器人
            for(int i = 0;i < _goalSegments.size();i++)     //check robots whose position is same with goal 
            {
                if(i != first)
                {
                    eq = false;
                    robot_start = _startSegments[i];
                    for(int j = 0;j < 3;j++)
                    {
                        for(int k = 0;k < crossing_segment[j].size();k++)
                        {
                            if(robot_start == crossing_segment[j][k].getSegmentId() && j == first_goal_pos)
                            {
                                robot_in_goal.push_back(i);
                                eq = true;
                                break;
                            }

                        }
                        if(eq == true)
                            break;
                    }
                }
            }        

            std::cout << "robot position in goal: ";
            for(int i = 0;i < robot_in_goal.size();i++)
                std::cout << robot_in_goal[i] << " ";
            std::cout << std::endl;

            temp.assign(start_distance.begin(),start_distance.end());
            std::sort(temp.begin(),temp.end());
            std::vector<uint32_t> robot_in_goal_position;       //robot in goal position distance from near to far
            robot_in_goal_position.clear();
            for(int i = 0;i < temp.size();i++)
            {
                for(int j = 0;j < robot_in_goal.size();j++)
                {
                    if(start_distance[robot_in_goal[j]] == temp[i])
                    {
                        robot_in_goal_position.push_back(robot_in_goal[j]);
                    }
                }
                if(robot_in_goal_position.size() == robot_in_goal.size())
                    break;
            }

            std::cout << "robots distance in goal from near to far: ";
            for(int i = 0;i < robot_in_goal_position.size();i++)
                std::cout << robot_in_goal_position[i] << " ";
            std::cout << std::endl;
            
            std::vector<uint32_t>::iterator it;
            for(int i = 0;i < all_try_robot.size();i++)
            {
                it = std::find(robot_in_goal.begin(), robot_in_goal.end(), all_try_robot[i]);
                // std::cout << "find the robot" << std;
                if(it != robot_in_goal.end())
                    robot_in_goal.erase(it);
                it = std::find(robot_in_goal_position.begin(), robot_in_goal_position.end(), all_try_robot[i]);
                if(it != robot_in_goal_position.end())
                    robot_in_goal_position.erase(it);
            }

            std::vector<uint32_t> robot_in_start_position;
            robot_in_start_position.clear();
            // 挡住启动机器人的机器人，从远到近排序
            for(int i = 0;i < temp.size();i++)
            {
                for(int j = 0;j < all_robot[first].size();j++)
                {
                    if(start_distance[all_robot[first][j]] == temp[i])
                    {
                        robot_in_start_position.push_back(all_robot[first][j]);
                    }
                }
                if(robot_in_start_position.size() == all_robot[first].size())
                    break;
            }

            std::cout << "robot in front of the start robot from far to near: ";
            for(int i = 0;i < robot_in_start_position.size();i++)
                std::cout << robot_in_start_position[i] << " ";
            std::cout << std::endl << std::endl;

            // 非起始点非目标点的空闲区域
            for(int i = 0;i < 3;i++)
            {
                if(i != first_robot_pos && i != first_goal_pos)
                {
                    cross_go = i;
                }
            }
            std::cout << "crossing to go: " << cross_go << std::endl << std::endl;

            //get path 清除路径协调器中的所有机器人信息
            for(uint32_t i = 0; i < _startSegments.size(); i++)
            {
                route_coordinator_->removeRobot(i); 
            }

            //std::cout << crossing_segment[cross_go][crossing_segment[cross_go].size() - 2 - add + 0].getSegmentId() << std::endl;
            add = robot_in_start_position.size() + robot_in_goal_position.size();
            for(int i = 0;i < all_try_robot.size();i++)
            {
                get = all_try_robot[i];
                RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                get_route = srr.getRouteCandidate(_startSegments[get], _startSegments[get], rcWrapper, \
                    robotDiameter_[get],  _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (i + 1));

                robotCollisions_[get] = srr.getRobotCollisions();
                robotCollisions_[get].resize(nr_robots_, 0);

                get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

            }
            if(cross_go == 1) 
            {    
                //挡住启动机器人的机器人
                for(int i = 0;i < robot_in_start_position.size();i++)     //for robot in front of start robot
                {
                    get = robot_in_start_position[i];
                    RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                    get_route = srr.getRouteCandidate(_startSegments[get], \
                    crossing_segment[cross_go][crossing_segment[cross_go].size() - 1 - add + i].getSegmentId(), rcWrapper, \
                    robotDiameter_[get],  _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (i + 1));

                    robotCollisions_[get] = srr.getRobotCollisions();
                    robotCollisions_[get].resize(nr_robots_, 0);

                    get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                }
                //挡住目标点的机器人
                for(int i = 0;i < robot_in_goal_position.size();i++)    //for robot has the same place with goal
                {
                    get = robot_in_goal_position[i];
                    RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                    get_route = srr.getRouteCandidate(_startSegments[get], crossing_segment[cross_go][crossing_segment[cross_go].size() \
                      - 1 - add + robot_in_start_position.size() + i].getSegmentId(), rcWrapper, \
                    robotDiameter_[get],  _speedList[get], temp_candidate[get], \
                    maxIterationsSingleRobot_ * (i + 1 + robot_in_start_position.size()));

                    robotCollisions_[get] = srr.getRobotCollisions();
                    robotCollisions_[get].resize(nr_robots_, 0);

                    get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                }

            }
            else
            {
                for(int i = 0;i < robot_in_start_position.size();i++)     //for robot in front of start robot
                {
                    get = robot_in_start_position[i];
                    RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                    get_route = srr.getRouteCandidate(_startSegments[get], \
                    crossing_segment[cross_go][add - i].getSegmentId(), rcWrapper, \
                    robotDiameter_[get],  _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (i + 1));

                    robotCollisions_[get] = srr.getRobotCollisions();
                    robotCollisions_[get].resize(nr_robots_, 0);

                    get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                }

                for(int i = 0;i < robot_in_goal_position.size();i++)    //for robot has the same place with goal
                {
                    get = robot_in_goal_position[i];
                    RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                    get_route = srr.getRouteCandidate(_startSegments[get], crossing_segment[cross_go][add - robot_in_start_position.size() - i].getSegmentId(), \
                    rcWrapper, robotDiameter_[get],  _speedList[get], temp_candidate[get], \
                    maxIterationsSingleRobot_ * (i + 1 + robot_in_start_position.size()));

                    robotCollisions_[get] = srr.getRobotCollisions();
                    robotCollisions_[get].resize(nr_robots_, 0);

                    get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                }
            }

            // std::cout << "first: " << first << std::endl;
            get = first;
            RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);        //for start robot

            get_route = srr.getRouteCandidate(_startSegments[get], _goalSegments[get], rcWrapper, robotDiameter_[get], \
                _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (1 + add));

            robotCollisions_[get] = srr.getRobotCollisions();
            robotCollisions_[get].resize(nr_robots_, 0);

            get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

            std::vector<uint32_t> left_robot;    //get remain robots
            for(int i = 0;i < _startSegments.size();i++)
            {
                left_robot.push_back(i);
            }
            std::vector<uint32_t>::iterator iter;
            for(int i = 0;i < robot_in_goal_position.size();i++)
            {
                iter = std::find(left_robot.begin(), left_robot.end(), robot_in_goal_position[i]);
                left_robot.erase(iter);
            }
            for(int i = 0;i < robot_in_start_position.size();i++)
            {
                iter = std::find(left_robot.begin(), left_robot.end(), robot_in_start_position[i]);
                left_robot.erase(iter);
            }
            iter = std::find(left_robot.begin(), left_robot.end(),first);
            left_robot.erase(iter);

            std::vector<uint32_t> robot_in_other;
            std::vector<uint32_t> robot_in_other_position;
            for(int i = 0;i < _goalSegments.size();i++)     //robot in crossing to go, distance from crossing center to themselves
            {
                if(robot_in_crossing[i] == cross_go)
                {
                    robot_in_other.push_back(i);
                }
            }
            for(int i = 0;i < temp.size();i++)
            {
                for(int j = 0;j < robot_in_other.size();j++)
                {
                    if(start_distance[robot_in_other[j]] == temp[i])
                    {
                        robot_in_other_position.push_back(robot_in_other[j]);
                    }
                }
                if(robot_in_other_position.size() == robot_in_other.size())
                    break;
            }

            std::cout << "robots in where we want to go from near to far: ";
            for(int i = 0;i < robot_in_other_position.size();i++)
                std::cout << robot_in_other_position[i] << " ";
            std::cout << std::endl;
            // for(int i = 0;i < robot_in_other_position.size();i++)
            //     std::cout << robot_in_crossing[robot_in_other_position[i]] << " ";
            // std::cout << std::endl << std::endl;

            for(int i = 0;i < left_robot.size();i++)         //for remain robots behind the start robot
            {
                get = left_robot[i];
                if(robot_in_crossing[get] != cross_go)            
                {
                    RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                    get_route = srr.getRouteCandidate(_startSegments[get], _startSegments[get], rcWrapper, \
                    robotDiameter_[get], _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (i + add + 1 + 1));

                    robotCollisions_[get] = srr.getRobotCollisions();
                    robotCollisions_[get].resize(nr_robots_, 0);
                
                    get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                }
            }

            for(int i = 0;i < robot_in_other_position.size();i++)   //robot in where we want to go
            {
                get = robot_in_other_position[robot_in_other_position.size() - 1 - i];     //the farest one
                for(int k = 0;k < crossing_segment[cross_go].size();k++)
                {
                    if(_startSegments[get] == crossing_segment[cross_go][k].getSegmentId())
                    {
                        find = k;
                        break;
                    }
                }
                if(cross_go == 1)
                {
                    
                    if(crossing_segment[cross_go].size() - find < add + robot_in_other_position.size())     //if the robot need to be pushed
                    {
                        RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                        get_route = srr.getRouteCandidate(_startSegments[get], \
                        crossing_segment[cross_go][crossing_segment[cross_go].size() - 1 - add - robot_in_other_position.size() + i].getSegmentId(),\
                            rcWrapper, robotDiameter_[get], _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (add + 3 + i));

                        robotCollisions_[get] = srr.getRobotCollisions();
                        robotCollisions_[get].resize(nr_robots_, 0);

                        get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                    }
                    else
                    {
                        RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                        get_route = srr.getRouteCandidate(_startSegments[get], _startSegments[get], rcWrapper, \
                        robotDiameter_[get], _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (add + 3 + i));

                        robotCollisions_[get] = srr.getRobotCollisions();
                        robotCollisions_[get].resize(nr_robots_, 0);
                    
                        get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                    }
                }
                else
                {
                    if(find + 1 < add + robot_in_other_position.size())
                    {
                        RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                        get_route = srr.getRouteCandidate(_startSegments[get], \
                        crossing_segment[cross_go][add + robot_in_other_position.size() - i].getSegmentId(),\
                            rcWrapper, robotDiameter_[get], _speedList[get], temp_candidate[get], \
                            maxIterationsSingleRobot_ * (add + 3 + i));

                        robotCollisions_[get] = srr.getRobotCollisions();
                        robotCollisions_[get].resize(nr_robots_, 0);

                        get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                    }
                    else
                    {
                        RouteCoordinatorWrapper rcWrapper(get, *route_coordinator_);

                        get_route = srr.getRouteCandidate(_startSegments[get], _startSegments[get], rcWrapper, \
                        robotDiameter_[get], _speedList[get], temp_candidate[get], maxIterationsSingleRobot_ * (add + 3 + i));

                        robotCollisions_[get] = srr.getRobotCollisions();
                        robotCollisions_[get].resize(nr_robots_, 0);
                    
                        get_route = route_coordinator_->addRoute(temp_candidate[get], robotDiameter_[get], get);

                    }
                }
            }

            for(int i = 0;i < temp_candidate.size();i++)
            {
                for(int j = 0;j < temp_candidate[i].size();j++)
                    _routeCandidates[i].emplace_back(temp_candidate[i][j]);
            }

        return get_route;
    }

    void MultiRobotRouter::setPriorityRescheduling(const bool _status)
    {
        usePriorityRescheduler_ = _status;
    }

    void MultiRobotRouter::setSpeedRescheduling(const bool _status)
    {
        useSpeedRescheduler_ = _status;
    }

}




