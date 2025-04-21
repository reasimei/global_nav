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

#include <tuw_global_router/avoidance_resolution.h>
#include <iostream>
#define TIMEOVERLAP (1)

namespace multi_robot_router
{
AvoidanceResolution::AvoidanceResolution(uint32_t _timeoverlap) : timeoverlap_(_timeoverlap)
{
}

AvoidanceResolution::AvoidanceResolution() : AvoidanceResolution(TIMEOVERLAP)
{
}

/*
MultipleRobotsResolution::resetSession函数：
（1）赋予机器人直径等参数以初值
（2）重置冲撞的情况以及voronoi路径下子区域的划分
（3）将路径节点前置与后置取消
*/
void AvoidanceResolution::resetSession(const RouteCoordinatorWrapper *_route_querry, const PotentialCalculator *_pCalc, const uint32_t _robotDiameter)
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
MultipleRobotsResolution::addCollision函数：
每当机器人遇到的路径冲突总数小于等于原有冲突数时，均添加机器人遇到的路径冲突总数至原有冲突数+1
*/
void AvoidanceResolution::addCollision(const uint32_t robot)
{
    if (encounteredCollisions_.size() <= robot)
        encounteredCollisions_.resize(robot + 1, 0);

    encounteredCollisions_[robot]++;
}

void AvoidanceResolution::saveCollision(const uint32_t _coll)
{
    addCollision(_coll);
}

const std::vector<uint32_t> &AvoidanceResolution::getRobotCollisions() const
{
    return encounteredCollisions_;  //返回机器人遇到的路径冲突总数
}

// std::vector<std::reference_wrapper<Vertex>> MultipleRobotsResolution::resolve(Vertex &_current, Vertex &_next, int32_t _collision)
// {
//     //第三次调用SRRP时，如果冲突序列中存在的机器人数量等于３
//     if (robot = 3)
        //区分优先级最高与第二高的两个机器人中哪个是本体机器人，哪个是受体机器人
        

        //如果优先级第三的机器人与优先级最高的机器人所组成的冲突类型（例如：BTA）＝优先级第二的机器人与优先级最高的机器人所组成的冲突类型


            //对于优先级第二与第三的机器人执行FB算法


// }

// void MultipleRobotsResolution::trackBack(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
// {
            //如果resolve的FB算法执行后不能返回有效的routing_table


                //如果优先级第二的机器人与优先级最高的机器人所组成的冲突类型＝AVRCA

                    
                    //如果优先级第三的机器人的路径不为优先级第二的机器人的路径的子集，且优先级第二的机器人的路径不为优先级第三的机器人的路径的子集


                        //对于优先级第二与第三的机器人执行BO算法


                    //end


                //否则（即如果优先级第二的机器人与优先级最高的机器人所组成的冲突类型不为AVRCA)


                    //如果优先级第三的机器人的路径为优先级第二的机器人的路径的子集，或优先级第二的机器人的路径为优先级第三的机器人的路径的子集


                        //对于优先级第二与第三的机器人执行+AVRCA算法

                        
                    //end

                    
                //end


            //end


        //end


// }        


//reslove对于不同的机器人分别执行，即每加入一个机器人，执行一次SRRP的resolve，不同的resolve通过resolutionAttemp_这一坐标区分
std::vector<std::reference_wrapper<Vertex>> AvoidanceResolution::resolve(Vertex &_current, Vertex &_next, int32_t _collision)
{
    //std::vector<std::vector<std::unique_ptr<Vertex>>> generatedSubgraphs_  generatedSubgraphs_为指向Vertex类指针对应的向量，即当前路径段与路径点形成的向量
    generatedSubgraphs_.emplace_back();
    foundSolutions_.clear();
    //一台机器人封堵住了一个路径节点使得其它机器人无法通过时，增加相应的冲突路径点，即存在冲突时BTA被触发


    //Triggered when a robot blocks a vertex
    addCollision(_collision);

    //find the potential when the collision robots leaves the segment
    //依据机器人占用路径节点的时间，设置机器人在该节点的离开势能，此部分为路径协调器工作
    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(_collision, _next.getSegment().getSegmentId());

    //There is no solution if the colliding robot never leaves the segment.
    //Therefore, do nothing when leavePotential is smaller zero

    //离开势能大于０，即需要进行规避措施时，先不回溯，直接执行AVRGA，然后再进行回溯，此回溯包含了AVRSA和AVRCA两种算法
    if (leavePotential >= 0)
    {
        int32_t collision = -1;
        Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        Vertex &next_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        avoidGoal(_current, _next, _collision, leavePotential);
        forwardandbackward(current_n, next_n, collision, leavePotential);
        backatopposite(current_n, next_n, collision, leavePotential);
        plusavrca(current_n, next_n, collision, leavePotential);
        trackBack(_current, _next, _collision, leavePotential);
    }

    resolutionAttemp_++;
    return foundSolutions_;
}

void AvoidanceResolution::trackBack(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //Free the next Vertex for new expansion
    _next.potential = -1;
    _next.collision = -1;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //If backtracking is not possible (we are on the start vertex) try to wait there
    //Additionally Backtracking beond wait Segments is not allowed to be able to solve
    //multi robot scenarios (avoiding n robots in a row)
    //目前需要由n台变成２台，再扩展到n台
    //Anyway backtracking beond wait Segments makes no sense, we will only find allready
    //found solutions...

    //如果当前节点的上一节点为空（即目前在起始顶点），或当前节点为需要等待的节点解
    if (_current.predecessor_ == NULL || _current.isWaitSegment)
    {
        int32_t collision = -1;
        //const RouteCoordinatorWrapper *route_querry_; checkSegment的作用为检查搜索路径段是否为各机器人规划路径段
        if (route_querry_->checkSegment(_current, 0, _freePotential + 2 * timeoverlap_, robotDiameter_, collision))
        {
            _next.potential = -1;
            _next.collision = -1;
            //_next.successor_ = NULL; //Tell expander to expand the vertex normally

            //后四步为更新当前节点势能惯用手段，前两步为通过不断增加当前节点的方式，获得需要搜索的路径序列
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;
            //此时仅依次扩展下一个节点
            current_n.successor_ = &_next; //Tell expander to only expand to next

            //current_n即为当前可行的节点解，将其作为尾部插入至foundSolutions_这个所有可行解序列中
            foundSolutions_.push_back(current_n);
        }
        else
        {
            if (_collision != collision)
                addCollision(collision);
            int32_t collision = -1;
            float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, _current.getSegment().getSegmentId());
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            Vertex &next_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            //如果当前节点没有前一顶点，即当前节点为起始点，则运行AVRSA算法
            if (_current.predecessor_ == NULL)
                {
                avoidStart(_current, _next, collision, leavePotential);
                forwardandbackward(current_n, next_n, collision, leavePotential);
                backatopposite(current_n, next_n, collision, leavePotential);
                plusavrca(current_n, next_n, collision, leavePotential);
                }
            else if (_current.isWaitSegment)
                moveSegment(*_current.predecessor_, _current, collision, leavePotential);
        }



    }
    //如果当前节点并非起始顶点，且当前节点并非需要等待的节点解
    else //we are somewhere on the path
    {
        int32_t collision = -1;
        bool vertexFree = route_querry_->checkSegment(*_current.predecessor_, _current.predecessor_->potential - timeoverlap_, _freePotential + 2 * timeoverlap_, robotDiameter_, collision);

        //如果当前节点未占用或存在冲突势能
        if (vertexFree || collision != -1)
        {
            //将下一节点加入冲突场景的队列
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
            Vertex &next_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            next_n.potential = -1;
            next_n.collision = -1;

            //后四步为更新当前节点势能惯用手段
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(*(_current.predecessor_)));
            Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            current_n.potential = _freePotential + 2 * timeoverlap_;
            current_n.collision = _collision;

            //Set References
            current_n.successor_ = &next_n; //Tell expander to only expand to "new next" (with new Potential)
            next_n.predecessor_ = &current_n;
            next_n.successor_ = &_next; //Tell expander to only expand to next (we are not allowed to leave the backtracked path)

            //如果在逐步扩展节点后存在了未占用的节点，则将其加入可行解队列尾部
            if (vertexFree)
            {
                foundSolutions_.push_back(current_n);
            }
            //如果在逐步扩展节点后未存在未占用的节点，且具有冲突势能，则将其加入新的碰撞冲突点
            else if (collision != -1)
            {
                if (collision != -1 && collision != _collision)
                    addCollision(collision);
                //此时并未找到未占用节点，故需先回溯找到可以等待的路径节点，然后启动AVRCA策略进行路口的规避
                float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, current_n.getSegment().getSegmentId());
                trackBack(current_n, next_n, collision, leavePotential);
                avoid(current_n, next_n, collision, leavePotential);
                forwardandbackward(current_n, next_n, collision, leavePotential);
                backatopposite(current_n, next_n, collision, leavePotential);
                plusavrca(current_n, next_n, collision, leavePotential);

                //Continue Resolving (Avoid Segment, Avoid Crossing, ...)
            }
        }
    }
        //第三次调用SRRP时，如果冲突序列中存在的机器人数量等于３
  //  if (robot = 3)
        //区分优先级最高与第二高的两个机器人中哪个是本体机器人，哪个是受体机器人
        

        //如果优先级第三的机器人与优先级最高的机器人所组成的冲突类型（例如：BTA）＝优先级第二的机器人与优先级最高的机器人所组成的冲突类型


            //对于优先级第二与第三的机器人执行FB算法


            //如果resolve的FB算法执行后不能返回有效的routing_table


                //如果优先级第二的机器人与优先级最高的机器人所组成的冲突类型＝AVRCA

                    
                    //如果优先级第三的机器人的路径不为优先级第二的机器人的路径的子集，且优先级第二的机器人的路径不为优先级第三的机器人的路径的子集


                        //对于优先级第二与第三的机器人执行BO算法


                    //end


                //否则（即如果优先级第二的机器人与优先级最高的机器人所组成的冲突类型不为AVRCA)


                    //如果优先级第三的机器人的路径为优先级第二的机器人的路径的子集，或优先级第二的机器人的路径为优先级第三的机器人的路径的子集


                        //对于优先级第二与第三的机器人执行+AVRCA算法

                        
                    //end

                    
                //end


            //end


        //end



}




void AvoidanceResolution::avoid(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    if (_current.predecessor_ == NULL)
        return;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;
    //寻找路口
    //We need to find if _next is a successor or a predecessor of current and use this crossing for avoidance
    //初始化路口是否找到的标志位，并定义Vertex类下的路口向量
    bool crossingFound = false;
    std::vector<std::reference_wrapper<Vertex>> crossing;

    //如果当前节点存在向前回溯的节点（即当前节点不为起始顶点），则将Va的前一顶点预设为可能的crossing
    if (_current.crossingPredecessor)
    {
        crossing = _current.getPlanningPredecessors();

        //把可能的crossing中的每个元素赋值给顶点类型的v中，即遍历各顶点是否为路口的形式，此时v为所有非起始顶点Va的可能前一顶点
        for (const Vertex &v : crossing)
        {
            //如果Va前一顶点v中的路径段ID号与Va下一节点的路径段ID号相同，则找到路口（因为路口会出现节点共同占用同一路径段的情况）
            if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }

    //如果没有找到路口并且在当前节点下存在后回溯的顶点（即当前节点不为终止顶点，此方法与当前节点不为起始顶点即向前回溯的搜索结合，可完成对于整个路径段的搜索），则将向后回溯的点预设为可能的crossing
    if (!crossingFound && _current.crossingSuccessor)
    {
        crossing = _current.getPlanningSuccessors();

        for (const Vertex &v : crossing)
        {
            if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }
    //开始执行AVRCA算法
    if (crossingFound)
    {
        for (const Vertex &waitSeg : crossing)
        {
            if (waitSeg.getSegment().getSegmentId() != _next.getSegment().getSegmentId())
            {
                int32_t collision = -1;
                bool vertexFree = route_querry_->checkSegment(waitSeg, _current.predecessor_->potential + pCalc_->CalculatePotential(_current) - timeoverlap_, std::max<float>(_freePotential, _current.potential + pCalc_->CalculatePotential(waitSeg)) + timeoverlap_, robotDiameter_, collision);

                //如果当前节点未占用或存在冲突势能，则从产生的路径点中读取末尾节点作为当前节点，并读取等待节点作为当前节点的后回溯节点
                if (vertexFree || collision != -1)
                {
                    //Copy the current Vertex and set smallest possible potential (everithing else stays the same)
                    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
                    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                    current_n.potential = current_n.predecessor_->potential + pCalc_->CalculatePotential(current_n);

                    //Copy the wait vertex insert it in the path and assign the right potential
                    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
                    Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                    wait_seg_n.potential = std::max<float>(_freePotential, current_n.potential + pCalc_->CalculatePotential(wait_seg_n)); //Set the potential as max of the next segment is free, and the minimum Vertex potential the robot can achive
                    wait_seg_n.collision = -1;
                    wait_seg_n.isWaitSegment = true;

                    wait_seg_n.successor_ = &_next;
                    wait_seg_n.predecessor_ = &current_n;

                    current_n.successor_ = &wait_seg_n;
                    //Reset _next to be ready for expansion (should be allready done in trackback, but safety first :D)
                    _next.potential = -1;
                    _next.collision = -1;

                    //如果当前节点未占用，则将等待节点输出为可行解
                    if (vertexFree)
                    {
                        foundSolutions_.push_back(wait_seg_n);
                    }
                    else if (collision != -1)
                    {
                        if (collision != _collision)
                            addCollision(collision);

                        float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_seg_n.getSegment().getSegmentId());
                        moveSegment(current_n, wait_seg_n, collision, leavePotential);
                    }
                }
            }
        }
    }
}

void AvoidanceResolution::moveSegment(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential) //Quick fix depth (Better depth queue with best first)
{
    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    std::queue<queueElement> empty;
    std::swap(queue_, empty);

    queueElement newEle;
    newEle.next = &_next;
    newEle.current = &_current;
    newEle.potential = _freePotential;
    newEle.collision = _collision;

    queue_.push(newEle);

    while (!queue_.empty())
    {

        queueElement q = queue_.front();
        queue_.pop();

        //We need to find the neighborhood where _current is not present
        std::vector<std::reference_wrapper<Vertex>> crossing;
        crossing = q.next->getPlanningPredecessors();

        for (const Vertex &v : crossing)
        {
            if (v.getSegment().getSegmentId() == q.current->getSegment().getSegmentId())
            {
                crossing = q.next->getPlanningSuccessors();
                break;
            }
        }

        int32_t collision = -1;

        //Check if we have enough time to move through the segment
        if (route_querry_->checkSegment(*(q.next), q.current->potential - timeoverlap_, q.current->potential + pCalc_->CalculatePotential(*(q.next)) + timeoverlap_, robotDiameter_, collision))
        {
            for (const Vertex &waitSeg : crossing)
            {
                if (expandSegment(waitSeg, *(q.current), *(q.next), q.collision, q.potential))
                    return;
            }
        }
        else if (collision != -1)
        {
            if (collision != q.collision)
                addCollision(collision);
        }
    }
}

bool AvoidanceResolution::expandSegment(const Vertex &cSeg, Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //Return if potential is blocked forever
    if (_freePotential < 0)
        return false;

    int32_t collision;
    float moveFwdTime = _current.potential + pCalc_->CalculatePotential(_next) + timeoverlap_;
    float waitTime = std::max<float>(moveFwdTime + pCalc_->CalculatePotential(cSeg) + timeoverlap_, _freePotential + 2 * timeoverlap_);
    bool vertexIsFree = route_querry_->checkSegment(cSeg, moveFwdTime - timeoverlap_, waitTime, robotDiameter_, collision);

    if (vertexIsFree || collision != -1)
    {
        //We need one Vertex for moving forward (next), one for waiting (waitSeg) and one for moving backward (next).
        //(We have to copy all three Vertex because every Vertex in the crossing is considered (backtracking...))

        //Copy Vertex used for moving away
        generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_next));
        Vertex &move_fwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        move_fwd_n.potential = moveFwdTime;
        move_fwd_n.isWaitSegment = false;
        move_fwd_n.collision = -1;

        //Copy the Vertex for waiting
        generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(cSeg));
        Vertex &wait_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        wait_n.potential = waitTime;
        wait_n.isWaitSegment = true;
        wait_n.collision = -1;

        //Copy Vertex used for moving back
        generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_next));
        Vertex &move_bwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        move_bwd_n.potential = -1;
        move_bwd_n.isWaitSegment = false;
        move_bwd_n.collision = -1;

        move_fwd_n.successor_ = &wait_n;
        //pred allready set (curr)

        wait_n.successor_ = &move_bwd_n;
        wait_n.predecessor_ = &move_fwd_n;

        move_bwd_n.predecessor_ = &wait_n;
        move_bwd_n.successor_ = _next.successor_;

        if (vertexIsFree)
        {
            foundSolutions_.push_back(wait_n);
            return true;
        }
        else if (collision != -1)
        {
            if (collision != _collision)
                addCollision(collision);

            float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_n.getSegment().getSegmentId());
            queueElement qe;
            qe.current = &move_fwd_n;
            qe.next = &wait_n;
            qe.collision = collision;
            qe.potential = leavePotential;
            queue_.push(qe);
            //addSegment(move_fwd_n, wait_n, collision, leavePotential);
            return false;
        }
    }

    return false;
}

void AvoidanceResolution::avoidStart(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //If we are not on Start we cant use this strategy :D
    //If we have allready used the avoid strategy we will not use it again
    //(caused due to multiple backtracking attemps)
    if (_current.predecessor_ != NULL)
        return;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //Select the side of the vertex which not contains _next to move avoid from the colliding robot
    bool foundPredecessorVertex = false;
    std::vector<std::reference_wrapper<Vertex>> crossing = _current.getPlanningPredecessors();

    for (const Vertex &v : crossing)
    {
        if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
        {
            crossing = _current.getPlanningSuccessors();
            foundPredecessorVertex = true;
            break;
        }
    }

    //Take care to only use avoid start once (else computation time issue if there are multiple backtracking attemps)
    if ((!foundPredecessorVertex && !avoidStartPredecessorDone_) || (foundPredecessorVertex && !avoidStartSuccessorDone_))
    {
        if (!foundPredecessorVertex)
            avoidStartPredecessorDone_ = true;
        else
            avoidStartSuccessorDone_ = true;

        for (const Vertex &waitSeg : crossing)
        {
            int32_t collision = -1;
            float waitTime = std::max<float>(_freePotential + 2 * timeoverlap_, pCalc_->CalculatePotential(_current) + pCalc_->CalculatePotential(waitSeg) + timeoverlap_);
            bool vertexIsFree = route_querry_->checkSegment(waitSeg, pCalc_->CalculatePotential(_current) - timeoverlap_, waitTime, robotDiameter_, collision);

            if (vertexIsFree || collision != -1)
            {
                //We need one Vertex for moving forward (current), one for waiting (waitSeg) and one for moving backward (current).
                //(We have to copy all three Vertex because every Vertex in the crossing is considered (backtracking...))

                //Copy Vertex used for moving away
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
                Vertex &move_fwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                move_fwd_n.potential = pCalc_->CalculatePotential(move_fwd_n);
                move_fwd_n.isWaitSegment = false;
                move_fwd_n.collision = -1;

                //Copy the Vertex for waiting
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
                Vertex &wait_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                wait_n.potential = waitTime;
                wait_n.isWaitSegment = true;
                wait_n.collision = -1;

                //Copy Vertex used for moving back
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
                Vertex &move_bwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                move_bwd_n.potential = -1;
                move_bwd_n.isWaitSegment = false;
                move_bwd_n.collision = -1;

                move_fwd_n.successor_ = &wait_n;
                move_fwd_n.predecessor_ = NULL; //We are on start

                wait_n.successor_ = &move_bwd_n;
                wait_n.predecessor_ = &move_fwd_n;

                move_bwd_n.predecessor_ = &wait_n;
                move_bwd_n.successor_ = _next.successor_;

                if (vertexIsFree)
                {
                    foundSolutions_.push_back(wait_n);
                }
                else if (collision != -1)
                {
                    if (collision != _collision)
                        addCollision(collision);

                    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_n.getSegment().getSegmentId());
                    moveSegment(move_fwd_n, wait_n, robotDiameter_, leavePotential);
                }
            }
        }
    }
}

void AvoidanceResolution::avoidGoal(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    //_next is goal Segment
    //_current tells us the direction

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //If we are not on Goal we cant use this strategy :D
    if (_next.getSegment().getSegmentId() != route_querry_->getEnd())
        return;

    //Select the side of the vertex which not contains _current to move avoid from the colliding robot
    std::vector<std::reference_wrapper<Vertex>> crossing = _next.getPlanningPredecessors();

    for (const Vertex &v : crossing)
    {
        if (v.getSegment().getSegmentId() == _current.getSegment().getSegmentId())
        {
            crossing = _current.getPlanningSuccessors();
            break;
        }
    }

    //Check if we can move through goal
    int32_t collision = -1;

    if (route_querry_->checkSegment(_next, _current.potential - timeoverlap_, _current.potential + pCalc_->CalculatePotential(_next), robotDiameter_, collision, true))
    {
        for (const Vertex &waitSeg : crossing)
        {
            int32_t collision = -1;
            float waitTime = std::max<float>(_freePotential + 2 * timeoverlap_, _current.potential + pCalc_->CalculatePotential(_next) + pCalc_->CalculatePotential(waitSeg) + 2 * timeoverlap_);
            bool vertexIsFree = route_querry_->checkSegment(waitSeg, _current.potential + pCalc_->CalculatePotential(_next) - timeoverlap_, waitTime, robotDiameter_, collision);

            if (vertexIsFree || collision != -1)
            {
                //We need one Vertex for moving forward (current), one for waiting (waitSeg) and one for moving backward (current).
                //(We have to copy all three Vertex because every Vertex in the crossing is considered (backtracking...))

                //Copy Vertex used for moving away
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_next));
                Vertex &move_fwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                move_fwd_n.potential = _current.potential + pCalc_->CalculatePotential(move_fwd_n);
                move_fwd_n.isWaitSegment = false;
                move_fwd_n.collision = -1;

                //Copy the Vertex for waiting
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
                Vertex &wait_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                wait_n.potential = waitTime;
                wait_n.isWaitSegment = true;
                wait_n.collision = -1;

                //Copy Vertex used for moving back
                generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_next));
                Vertex &move_bwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                move_bwd_n.potential = -1;
                move_bwd_n.isWaitSegment = false;
                move_bwd_n.collision = -1;

                move_fwd_n.successor_ = &wait_n;
                move_fwd_n.predecessor_ = &_current;

                wait_n.successor_ = &move_bwd_n;
                wait_n.predecessor_ = &move_fwd_n;

                move_bwd_n.predecessor_ = &wait_n;
                move_bwd_n.successor_ = NULL; //We are on the goal vertex

                if (vertexIsFree)
                {
                    foundSolutions_.push_back(wait_n);
                }
                else if (collision != -1)
                {
                    if (collision != _collision)
                        addCollision(collision);

                    float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_n.getSegment().getSegmentId());
                    moveSegment(move_fwd_n, wait_n, robotDiameter_, leavePotential);
                }
            }
        }
    }
    else if (collision != -1)
    {
        if (collision != _collision)
            addCollision(collision);
    }
}
} // namespace multi_robot_router



void multi_robot_router::AvoidanceResolution::forwardandbackward(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
 //   bool vertexIsFree = route_querry_->checkSegment(waitSeg, _current.potential + pCalc_->CalculatePotential(_next) - timeoverlap_, waitTime, robotDiameter_, collision); 
    int32_t collision = -1;
    if (_current.predecessor_ == NULL)
        return;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //仅对于Cons机器人加入搜索路径序列


    //后四步为更新当前节点势能惯用手段，前两步为通过不断增加当前节点的方式，获得需要搜索的路径序列
    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
    current_n.potential = _freePotential + 2 * timeoverlap_;
    current_n.collision = _collision;
    //
    bool foundPredecessorVertex = false;
    std::vector<std::reference_wrapper<Vertex>> crossing = _current.getPlanningPredecessors();
    if (_current.crossingPredecessor)
    {
        for (const Vertex &v : foundSolutions_)
        {
            if (v.getSegment().getSegmentId() == _current.getSegment().getSegmentId())
            {
            crossing = _current.getPlanningPredecessors();
            foundPredecessorVertex = true;
            break;
            }
        }
        for (const Vertex &waitSeg : crossing)
        {
            generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
            Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
            wait_seg_n.potential = std::max<float>(_freePotential, current_n.potential + pCalc_->CalculatePotential(wait_seg_n)); //Set the potential as max of the next segment is free, and the minimum Vertex potential the robot can achive
            wait_seg_n.collision = -1;
            wait_seg_n.isWaitSegment = true;
            foundSolutions_.push_back(wait_seg_n);
        }
    }
    // if (vertexIsFree)
    // {
    //     foundSolutions_.push_back(wait_seg_n);
    // }
    else if (collision != -1)
    {
        if (collision != _collision)
            addCollision(collision);
        Vertex &wait_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_n.getSegment().getSegmentId());
        Vertex &move_fwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        moveSegment(move_fwd_n, wait_seg_n, robotDiameter_, leavePotential);
    }
}

void multi_robot_router::AvoidanceResolution::backatopposite(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    int32_t collision = -1;    
    if (_current.predecessor_ == NULL)
        return;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //仅对于Cons机器人加入搜索路径序列


    
    //后四步为更新当前节点势能惯用手段，前两步为通过不断增加当前节点的方式，获得需要搜索的路径序列
    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
    current_n.potential = _freePotential + 2 * timeoverlap_;
    current_n.collision = _collision;

    //寻找路口
    //We need to find if _next is a successor or a predecessor of current and use this crossing for avoidance
    //初始化路口是否找到的标志位，并定义节点类下的路口向量
    bool crossingFound = false;
    std::vector<std::reference_wrapper<Vertex>> crossing;

    //如果当前节点存在向前回溯的节点（即当前节点不为起始顶点），则将向前回溯的点预设为可能的crossing
    if (_current.crossingPredecessor)
    {
        crossing = _current.getPlanningPredecessors();

        //把可能的crossing中的每个元素因此赋值给顶点类型的v中，即遍历各顶点是否为路口的形式
        for (const Vertex &v : crossing)
        {
            //如果v中的路径段ID号与下一节点的路径段ID号相同，则找到路口（因为路口会出现节点共同占用同一路径段的情况）
            if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }

    //如果没有找到路口并且在当前节点下存在后回溯的顶点（即当前节点不为终止顶点，此方法与当前节点不为起始顶点即向前回溯的搜索结合，可完成对于整个路径段的搜索），则将向后回溯的点预设为可能的crossing
    if (!crossingFound && _current.crossingSuccessor)
    {
        crossing = _current.getPlanningSuccessors();

        for (const Vertex &v : crossing)
        {
            if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }
    //开始执行AVRCA算法
    if (crossingFound)
    {

    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
    current_n.potential = _freePotential + 2 * timeoverlap_;
    current_n.collision = _collision;
    //
    bool foundPredecessorVertex = false;
    std::vector<std::reference_wrapper<Vertex>> crossing = _current.getPlanningPredecessors();
    if (_current.crossingPredecessor)
    {
        for (const Vertex &v : crossing)
        {
            if (v.getSegment().getSegmentId() == _current.getSegment().getSegmentId())
            {
            crossing = _current.getPlanningPredecessors();
            foundPredecessorVertex = true;
            break;
            }
        }
        Vertex &waitSeg = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
        Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        wait_seg_n.potential = std::max<float>(_freePotential, current_n.potential + pCalc_->CalculatePotential(wait_seg_n)); //Set the potential as max of the next segment is free, and the minimum Vertex potential the robot can achive
        wait_seg_n.collision = -1;
        wait_seg_n.isWaitSegment = true;
        foundSolutions_.push_back(wait_seg_n);
    }
    // if (vertexIsFree)
    // {
    //     foundSolutions_.push_back(wait_seg_n);
    // }
    else if (collision != -1)
    {
        if (collision != _collision)
            addCollision(collision);
        Vertex &wait_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_n.getSegment().getSegmentId());
        Vertex &move_fwd_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
        moveSegment(move_fwd_n, wait_seg_n, robotDiameter_, leavePotential);
    }

    }

}    

void multi_robot_router::AvoidanceResolution::plusavrca(Vertex &_current, Vertex &_next, const int32_t _collision, const float _freePotential)
{
    int32_t collision = -1;
    //
    if (_current.predecessor_ == NULL)
        return;

    //Return if potential is blocked forever
    if (_freePotential < 0)
        return;

    //仅对于Cons机器人加入搜索路径序列


    
    //后四步为更新当前节点势能惯用手段，前两步为通过不断增加当前节点的方式，获得需要搜索的路径序列
    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
    current_n.potential = _freePotential + 2 * timeoverlap_;
    current_n.collision = _collision;

    //寻找路口
    //We need to find if _next is a successor or a predecessor of current and use this crossing for avoidance
    //初始化路口是否找到的标志位，并定义节点类下的路口向量
    bool crossingFound = false;
    std::vector<std::reference_wrapper<Vertex>> crossing;

    //如果当前节点存在向前回溯的节点（即当前节点不为起始顶点），则将向前回溯的点预设为可能的crossing
    if (_current.crossingPredecessor)
    {
        crossing = _current.getPlanningPredecessors();

        //把可能的crossing中的每个元素因此赋值给顶点类型的v中，即遍历各顶点是否为路口的形式
        for (const Vertex &v : crossing)
        {
            //如果v中的路径段ID号与下一节点的路径段ID号相同，则找到路口（因为路口会出现节点共同占用同一路径段的情况）
            if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }

    //如果没有找到路口并且在当前节点下存在后回溯的顶点（即当前节点不为终止顶点，此方法与当前节点不为起始顶点即向前回溯的搜索结合，可完成对于整个路径段的搜索），则将向后回溯的点预设为可能的crossing
    if (!crossingFound && _current.crossingSuccessor)
    {
        crossing = _current.getPlanningSuccessors();

        for (const Vertex &v : crossing)
        {
            if (v.getSegment().getSegmentId() == _next.getSegment().getSegmentId())
            {
                crossingFound = true;
                break;
            }
        }
    }
    //开始执行AVRCA算法
    if (crossingFound)
    {
        for (const Vertex &waitSeg : crossing)
        {
            if (waitSeg.getSegment().getSegmentId() != _next.getSegment().getSegmentId())
            {
                int32_t collision = -1;
                bool vertexFree = route_querry_->checkSegment(waitSeg, _current.predecessor_->potential + pCalc_->CalculatePotential(_current) - timeoverlap_, std::max<float>(_freePotential, _current.potential + pCalc_->CalculatePotential(waitSeg)) + timeoverlap_, robotDiameter_, collision);

                //如果当前节点未占用或存在冲突势能，则从产生的路径点中读取末尾节点作为当前节点，并读取等待节点作为当前节点的后回溯节点
                if (vertexFree || collision != -1)
                {
                    //Copy the current Vertex and set smallest possible potential (everithing else stays the same)
                    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(_current));
                    Vertex &current_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                    current_n.potential = current_n.predecessor_->potential + pCalc_->CalculatePotential(current_n);

                    //Copy the wait vertex insert it in the path and assign the right potential
                    generatedSubgraphs_[resolutionAttemp_].emplace_back(std::make_unique<Vertex>(waitSeg));
                    Vertex &wait_seg_n = *(generatedSubgraphs_[resolutionAttemp_].back().get());
                    wait_seg_n.potential = std::max<float>(_freePotential, current_n.potential + pCalc_->CalculatePotential(wait_seg_n)); //Set the potential as max of the next segment is free, and the minimum Vertex potential the robot can achive
                    wait_seg_n.collision = -1;
                    wait_seg_n.isWaitSegment = true;

                    wait_seg_n.successor_ = &_next;
                    wait_seg_n.predecessor_ = &current_n;

                    current_n.successor_ = &wait_seg_n;
                    //Reset _next to be ready for expansion (should be allready done in trackback, but safety first :D)
                    _next.potential = -1;
                    _next.collision = -1;
                    
                    //如果当前节点未占用，则将等待节点输出为可行解
                    if (vertexFree)
                    {
                        foundSolutions_.push_back(wait_seg_n);
                    }
                    else if (collision != -1)
                    {
                        if (collision != _collision)
                            addCollision(collision);

                        float leavePotential = route_querry_->findPotentialUntilRobotOnSegment(collision, wait_seg_n.getSegment().getSegmentId());
                        moveSegment(current_n, wait_seg_n, collision, leavePotential);
                    }
                }
            }
        }
    }
}
