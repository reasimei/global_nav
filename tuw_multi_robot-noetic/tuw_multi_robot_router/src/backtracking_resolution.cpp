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

#include <tuw_global_router/backtracking_resolution.h>
#include <iostream>
#define TIMEOVERLAP (1)

namespace multi_robot_router
{
BacktrackingResolution::BacktrackingResolution(uint32_t _timeoverlap) : timeoverlap_(_timeoverlap)
{
}

BacktrackingResolution::BacktrackingResolution() : BacktrackingResolution(TIMEOVERLAP)
{
}

/*
BacktrackingResolution::resetSession函数：
（1）赋予机器人直径等参数以初值
（2）重置冲撞的情况以及voronoi路径下子区域的划分
（3）将路径节点前置与后置取消
该段程序在本人扩展后无需更改
*/
void BacktrackingResolution::resetSession(const RouteCoordinatorWrapper *_route_querry, const PotentialCalculator *_pCalc, const uint32_t _robotDiameter)
{
    route_querry_ = _route_querry;
    pCalc_ = _pCalc;
    robotDiameter_ = _robotDiameter;
    generatedSubgraphs_.clear();
    encounteredCollisions_.clear();
    resolutionAttemp_ = 0;
    avoidStartPredecessorDone_ = false;
    avoidStartSuccessorDone_ = false;
}


/*
BacktrackingResolution::addCollision函数：
每当机器人遇到的路径冲突总数小于等于原有冲突数时，均添加机器人遇到的路径冲突总数至原有冲突数+1
该段程序在本人扩展后需更改
*/
void BacktrackingResolution::addCollision(const uint32_t robot)
{
    if (encounteredCollisions_.size() <= robot)
        encounteredCollisions_.resize(robot + 1, 0);

    encounteredCollisions_[robot]++;
}


/*
void BacktrackingResolution::saveCollision函数：

*/
void BacktrackingResolution::saveCollision(const uint32_t _coll)
{
    addCollision(_coll);
}

const std::vector<uint32_t> &BacktrackingResolution::getRobotCollisions() const
{
    return encounteredCollisions_;  //返回冲撞可能数
}

//将所有的顶点包装
std::vector<std::reference_wrapper<Vertex>> BacktrackingResolution::resolve(Vertex &_current, Vertex &_next, int32_t _collision)
{
    //std::vector<std::vector<std::unique_ptr<Vertex>>> generatedSubgraphs_  generatedSubgraphs_为指向Vertex类指针对应的向量，即调用当前路径段与路径点形成的向量
    generatedSubgraphs_.emplace_back(); 
    foundSolutions_.clear();  //初始状态下，没有路径解，即初始化路径解

    //Triggered when a robot blocks a vertex
    addCollision(_collision);  //一台机器人封堵住了一个路径节点使得其它机器人无法通过时，增加相应的冲突路径点，即存在冲突时BTA被触发

    //find the potential when the collision robots leave the segment
    //依据机器人占用路径节点的时间，设置机器人在该节点的离开势能，此部分为路径协调器工作
    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(_collision, _next.getSegment().getSegmentId());  
    

    //There is no solution if the colliding robot never leaves the segment.
    //Therefore, do nothing when leavePotential is smaller zero
    //如果碰撞机器人RC一直无法离开当前路径段，则路径协调器无法产生可行解，只有离开势能大于零时，才具有可行解，此时进行路径回溯
    if (leavePotential >= 0)
    {
        trackBack(_current, _next, _collision, leavePotential);
    }

    resolutionAttemp_++;  //每在一台机器人上运行BTA后，该坐标加１，得以区分
    std::cout << std::endl;
    std::cout << "this is BacktrackingResolution::resolve" << std::endl;
    return foundSolutions_;
}

void BacktrackingResolution::trackBack(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //Free the next Vertex for new expansion
    //初始化下一节点的势能与碰撞势能
    _next.potential = -1;
    _next.collision = -1;

    //Return if potential is blocked forever
    //自由势能恒小于零时，则该路径节点被持续阻挡，无法用BTA算法解决，返回空值
    if (_freePotential < 0)
        return;

    //If backtracking is not possible (we are on the start vertex) try to wait there
    //Additionally Backtracking beond wait Segments is not allowed to be able to solve
    //multi robot scenarios (avoiding n robots in a row)
    //Anyway backtracking beond wait Segments makes no sense, we will only find allready
    //found solutions...
    //
    //如果当前节点va没有前一顶点（即va是起始顶点）或者当前就在等待（即va已经找到了避免冲突的点）时，直接设置该点为可行解即等待顶点
    if (_current.predecessor_ == NULL || _current.isWaitSegment) 
    {
        //设置机器人碰撞势能为-1，即继续等待
        int32_t collision = -1;
        //搜索路径段为机器人走过的路径段时
        if (route_querry_->checkSegment(_current, 0, _freePotential + 2 * timeoverlap_, robotDiameter_, collision))
        {
            //初始化势能和碰撞信息
            _next.potential = -1;
            _next.collision = -1;
            //_next.successor_ = NULL; //Tell expander to expand the vertex normally
            //将当前节点设置潜在势能与冲撞势能
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;
            //Tell expander to only expand to next
            //仅仅扩展至下一节点
            current_n.successor_ = &_next;


            foundSolutions_.push_back(current_n);
        }
        else
        {
            if (_collision != collision)
                addCollision(collision);
        }
    }
    //we are somewhere on the path　如果当前节点va有前一顶点（即va不是起始顶点）且当前未在等待（即va与机器人未发生冲突）时，则开始回溯
    else
    {
        int32_t collision = -1;
        bool vertexFree = route_querry_->checkSegment(*_current.predecessor_, _current.predecessor_->potential - timeoverlap_, _freePotential + 2 * timeoverlap_, robotDiameter_, collision);
        //如果为未占用或存在冲突的路径节点
        if (vertexFree || collision != -1)
        {
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &next_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            //初始化下一节点新元素的势能与冲突数
            next_n.potential = -1;
            next_n.collision = -1;
            //将当前节点的上一节点后插入产生的路径点，并在此时读取产生的路径点这一容器中的末尾路径点（即新插入的路径点），作为当前节点，此过程可以完成回溯
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(*(_current.predecessor_)));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;

            //Set References
            current_n.successor_ = &next_n; //Tell expander to only expand to "new next" (with new Potential)
            next_n.predecessor_ = &current_n;
            next_n.successor_ = &_next; //Tell expander to only expand to next (we are not allowed to leave the backtracked path)

            //回溯到第一个节点未占用的顶点时，结束回溯，将该点加入可行解中
            if (vertexFree)
            {
                foundSolutions_.push_back(current_n);
            }
            //如果冲突未解决则继续回溯
            else if (collision != -1)
            {
                if (collision != -1 && collision != _collision)
                    addCollision(collision);
                //路径点依然具有冲突势能时，继续执行trackBack的回溯过程，直至找到vertexFree的点，再结束回溯
                float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, current_n.getSegment().getSegmentId());
                trackBack(current_n, next_n, collision, leavePotential);

                //Continue Resolving (Avoid Segment, Avoid Crossing, ...)
            }
        }
    }
    std::cout << std::endl;
    std::cout << "BacktrackingResolution::trackBack" << std::endl;
}
} // namespace multi_robot_router
