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

#ifndef MRR_H
#define MRR_H

#include <tuw_global_router/single_robot_router.h>
#include <tuw_global_router/srr_utils.h>
#include <tuw_global_router/priority_scheduler.h>
#include <tuw_global_router/route_coordinator_timed.h>
#include <tuw_global_router/route_coordinator.h>
#include <tuw_global_router/route_generator.h>
#include <tuw_global_router/speed_scheduler.h>
#include <tuw_global_router/segment_expander.h>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>

#define PI 3.1415926
//#include <tuw_global_router/router_node.h>


struct RobotPair {
    uint32_t robot1;
    uint32_t robot2;
    float max_hose_length;
};

//TODO replan
namespace multi_robot_router
{
/**
     * @brief standard multi robot router without multi threading (Works best in practice)
     */
class MultiRobotRouter : protected RouteGenerator
{
  public:
    /**
             * @brief constructor
             * @param _nr_robots the number of robots to plan
             * @param _robotDiameter a vector with size _nr_robots which contains every robots diameter 
             */
    MultiRobotRouter(const uint32_t _nr_robots, const std::vector<uint32_t> &_robotDiameter);
    /**
             * @brief constructor
             * @param _nr_robots the number of robots to plan
             */
    MultiRobotRouter(const uint32_t _nr_robots);
    /** 
             * @brief sets the number of robots used
             * @param _nr_robot the number of robots 
             */
    virtual void setRobotNr(const uint32_t _nr_robots);
    /**
             * @brief sets the robot diameter for every robot
             * @param _diameter vector with length _nr_robots to set all robo diameters
             */
    virtual void setRobotDiameter(const std::vector<uint32_t> &_diameter);
    /**
             * @brief computes the routing table according to the given start and goal _goalSegments
             * @param _graph the base graph used to plan a path 
             * @param _startSegments for each robot the id of the start segment
             * @param _goalSegments for each robot the id of the goal segment
             * @param _routingTable the reference to the found routing table
             * @param _timeLimit the (approximate) maximum time the planner is allowed to use 
             * @returns if a routing table is found
             */
    virtual bool getRoutingTable(const std::vector<Segment> &_graph, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, std::vector<std::vector<Checkpoint>> &_routingTable, const float &_timeLimit);
    /**
             * @brief returns the number of attemps to reschedul the priorities taken
             * @returns the number of attemps to reschedul the priorities taken
             */
    virtual const uint32_t getPriorityScheduleAttempts() const;
    /**
             * @brief returns the number of attemps to limit the maximum speed of a robot taken
             * @returns the number of attemps to limit the maximum speed of a robot taken
             */
    virtual const uint32_t getSpeedScheduleAttempts() const;
    /**
             * @brief sets the CollisionResolverType used to resolve occured robot collisions 
             * @param cRes the CollisionResolver typ  
             */
    virtual void setCollisionResolver(const SegmentExpander::CollisionResolverType cRes);
    /**
             * @brief enables or disables priority rescheduling 
             */
    virtual void setPriorityRescheduling(const bool _status);
    /**
             * @brief enables or disables speed rescheduling 
             */
    virtual void setSpeedRescheduling(const bool _status);
    virtual void setRobotPairs(const std::vector<RobotPair> &pairs);
    virtual bool checkHoseLengthRestriction(const std::vector<RouteVertex> &route1, const std::vector<RouteVertex> &route2, float max_hose_length) const;


  private:
    void resetAttempt(const std::vector<Segment> &_graph);
    // attemps to find a routing table according to a given priority and speed schedule
    bool planPaths(const std::vector<uint32_t> &_priorityList, const std::vector<float> &_speedList, const std::vector<uint32_t> &_startSegments, const std::vector<uint32_t> &_goalSegments, const uint32_t _firstSchedule, std::vector<std::vector<RouteVertex>> &_routeCandidates, uint32_t &_robot);
    //try new method to get routing table
    bool planPaths_push(const std::vector<Segment> &_graph, const std::vector<Eigen::Vector2d> &_goalpos, const std::vector<Eigen::Vector2d> &_startpos, const std::vector<uint32_t> &_goalSegments, const std::vector<uint32_t> &_startSegments, std::vector<std::vector<RouteVertex>> &_routeCandidates, const std::vector<float> &_speedList);

    PriorityScheduler priority_scheduler_;
    SpeedScheduler speed_scheduler_;
    RouteCoordinator *route_coordinator_;
    RouteCoordinatorTimed rct_;
    bool useSpeedRescheduler_ = true;
    bool usePriorityRescheduler_ = true;

    uint32_t nr_robots_;
    uint32_t min_diameter_;
    std::vector<uint32_t> robotDiameter_;
    std::vector<std::vector<uint32_t>> robotCollisions_;
    uint32_t priorityScheduleAttempts_;
    uint32_t speedScheduleAttempts_;
    SegmentExpander::CollisionResolverType cResType_;
    SingleRobotRouter srr;
    uint32_t maxIterationsSingleRobot_;
    std::vector<RobotPair> robot_pairs_; // Add this line
};



} // namespace multi_robot_router
#endif
