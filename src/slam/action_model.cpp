#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    previousPose_.x = 0;
    previousPose_.y = 0;
    previousPose_.theta = 0;

    dx_ = 0;
    dy_ = 0;
    dtheta_ = 0;
    
    numberGenerator_ = std::mt19937(std::random_device()());
    
    xStd_ = 1;
    yStd_ = 1;
    thetaStd_ = 1;
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    double ds = dx_ * dx_ + dy_ * dy_;
    double alpha = atan2(dy_, dx_) - previousPose_.theta;
    double angle = dtheta_ - alpha;

    xStd_ = std::sqrt(k1_ * abs(alpha));
    yStd_ = std::sqrt(k2_ * ds);
    thetaStd_ = std::sqrt(k1_ * abs(angle));

    previousPose_ = odometry;
    utime_ = odometry.utime;
    
    if (ds < min_dist_ && abs(alpha) < min_theta_ && abs(angle) < min_theta_)
    {
        return false;
    }
    else
    {
        return true;    // Placeholder
    }
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    mbot_lcm_msgs::pose2D_t newPose = sample.pose;
    
    std::normal_distribution<double> e1(0, xStd_);
    std::normal_distribution<double> e2(0, yStd_);
    std::normal_distribution<double> e3(0, thetaStd_);

    double e_1 = e1(numberGenerator_);
    double e_2 = e2(numberGenerator_);
    double e_3 = e3(numberGenerator_);

    double ds = dx_ * dx_ + dy_ * dy_;
    double alpha = atan2(dy_, dx_) - newPose.theta;
    double angle = dtheta_ - alpha;

    newPose.x += (dx_+e_2)*cos(newPose.theta+alpha+e_1);
    newPose.y += (dy_+e_2)*sin(newPose.theta+alpha+e_1);
    newPose.theta += angle + e_1 + e_3;
    
    mbot_lcm_msgs::particle_t newSample = sample;
    newSample.pose = newPose;

    return newSample;
}
