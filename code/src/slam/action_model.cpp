#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>



ActionModel::ActionModel(void)
: alpha1_(0.55f)
, alpha2_(0.0015f)
, alpha3_(0.255f)
, alpha4_(0.0015f)
, initialized_(false)
{

}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_)
    {
        previousOdometry_ = odometry;
        initialized_ = true;
    }
    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = angle_diff(odometry.theta, previousOdometry_.theta);
    direction_ = 1.0;
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    trans_ = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    if (std::abs(trans_) < 0.001f)
    {
        rot1_ = 0.0f;
    }
    else if(std::abs(rot1_) > M_PI / 2.0)
    {
        rot1_ = angle_diff(M_PI, rot1_);
        direction_ = -1.0;
    }
    rot2_ = angle_diff(deltaTheta, rot1_);
    moved_ = (deltaX != 0.0f) || (deltaY != 0.0f) || (deltaTheta != 0.0f);
    rot1Std_ = alpha1_ * abs(rot1_);
    transStd_ = alpha3_ * abs(trans_);
    rot2Std_ = alpha1_ * abs(rot2_);
    previousOdometry_ = odometry;
    utime_ = odometry.utime;

    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    if (moved_)
    {
        float sampleRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
        float sampleTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
        float sampleRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

        particle_t newSample = sample;
        newSample.pose.x += direction_ * sampleTrans * std::cos(newSample.pose.theta + sampleRot1);
        newSample.pose.y += direction_ * sampleTrans * std::sin(newSample.pose.theta + sampleRot1);
        newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampleRot1 + sampleRot2);
        newSample.parent_pose = sample.pose;
        newSample.pose.utime = utime_;

        return newSample;
    }
    else
    {
        particle_t newSample = sample;
        newSample.pose.utime = utime_;

        return newSample;
    }
}
