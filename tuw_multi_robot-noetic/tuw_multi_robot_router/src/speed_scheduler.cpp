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

#include <tuw_global_router/speed_scheduler.h>
#include <climits>

namespace multi_robot_router
{

const std::vector<float> &SpeedScheduler::getActualSpeeds()
{
    return actualSpeedSchedule_;
}

SpeedScheduler::SpeedScheduler(const uint32_t _nrRobots)
{
    reset(_nrRobots);
}

bool SpeedScheduler::rescheduleSpeeds(const uint32_t _collidingRobot, const std::vector<uint32_t> &_collisions, std::vector<float> &_newSchedule, int32_t &_firstRobotToReplan)
{
    //Perform only one speed reduction

    //bool found = false;

    //while(!found)
    //{
    _firstRobotToReplan = -1;
    uint32_t collisions = 0;

    for (int i = 0; i < _collisions.size(); i++)
    {
        if (_collisions[i] > collisions && i != _collidingRobot)
        {
            _firstRobotToReplan = i;
            collisions = _collisions[i];
        }
    }

    if (_firstRobotToReplan != -1 && actualSpeedSchedule_[_firstRobotToReplan] == 1.0)
    {
        actualSpeedSchedule_[_firstRobotToReplan] *= 2.0;
        checkedSchedules_.emplace_back(actualSpeedSchedule_);
        _newSchedule = actualSpeedSchedule_;
        return true;
    }
    else
    {
        _firstRobotToReplan = -1;
        return false;
    }

    //}
    //return found;
}

void SpeedScheduler::reset(const uint32_t _nrRobots)
{
    checkedSchedules_.clear();
    actualSpeedSchedule_.clear();

    actualSpeedSchedule_.resize(_nrRobots, 1.0);
    checkedSchedules_.emplace_back(actualSpeedSchedule_);
}

} // namespace multi_robot_router
