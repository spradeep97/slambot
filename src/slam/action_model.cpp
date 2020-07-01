#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <common/time_util.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    is_initialized_ = false;
    k1_ = 0.001; //1.0 [3.0 , 0.01]
    k2_ = 0.00001; //0.5
    k3_ = 0.001; //0.001
    k4_ = 0.00001; //0.05
    std_dev1_ = 0.0;
    std_dev2_ = 0.0;
    std_dev3_ = 0.0;
}


// bool ActionModel::updateAction(const pose_xyt_t& odometry)
// {
//     ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
//     // Initialize prevPose_ if it does not already exist
//     // pose_xyt_t prevPose_;
//     if (!is_initialized_) {
//         prevPose_ = odometry;
//         is_initialized_ = true;
//     }
//
//     if (std::abs(prevPose_.x - odometry.x) < 1E-7 && std::abs(prevPose_.y - odometry.y) < 1E-7 && std::abs(angle_diff(prevPose_.theta, odometry.theta)) < 1E-7) {
//       prevPose_ = odometry;
//       return false;
//     }
//
//     //Calculate actions
//     double del_x, del_y, del_s, del_theta, alpha;
//     del_x     = odometry.x - prevPose_.x;
//     del_y     = odometry.y - prevPose_.y;
//     del_theta = angle_diff(odometry.theta, prevPose_.theta);
//     del_s     = std::sqrt(del_x * del_x + del_y * del_y);
//     alpha     = angle_diff(std::atan2(del_y, del_x) , prevPose_.theta);
//
//     action_[0] = wrap_to_pi(alpha);
//     action_[1] = del_s;
//     action_[2] = angle_diff(del_theta , alpha);
//
//     //Initialize the random generator
//
//     double std_dev1 = k1_ * abs(action_[0]);
//     double std_dev2 = k2_ * abs(action_[1]);
//     double std_dev3 = k3_ * abs(action_[2]);
//
//     // std::random_device rd;
//     // std::mt19937 gen(rd());
//     double mean = 0.0;
//     double mean_angle = 0.0;
//     N1.param( decltype(N1)::param_type{ mean_angle, std_dev1 } ) ;
//     N2.param( decltype(N2)::param_type{ mean, std_dev2 } ) ;
//     N3.param( decltype(N3)::param_type{ mean_angle, std_dev3 } ) ;
//
//     //Sample noise for action from normal distribution
//
//     //Update prevPose_
//     prevPose_ = odometry;
//
//     return true;
// }

bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    // Initialize prevPose_ if it does not already exist
    // pose_xyt_t prevPose_;
    if (!is_initialized_) {
        prevPose_ = odometry;
        is_initialized_ = true;
    }

    if (std::abs(prevPose_.x - odometry.x) < 1E-7 && std::abs(prevPose_.y - odometry.y) < 1E-7 && std::abs(angle_diff(prevPose_.theta, odometry.theta)) < 1E-7) {
      prevPose_ = odometry;
      return false;
    }

    //Calculate actions
    double del_x, del_y, del_s, del_theta, alpha;
    del_x     = odometry.x - prevPose_.x;
    del_y     = odometry.y - prevPose_.y;
    del_theta = angle_diff(odometry.theta, prevPose_.theta);
    // del_theta = odometry.theta - prevPose_.theta;
    del_s     = std::sqrt(del_x * del_x + del_y * del_y);
    alpha     = angle_diff(std::atan2(del_y, del_x) , prevPose_.theta);
    // alpha     = std::atan2(del_y, del_x) , prevPose_.theta;

    double c1 = k1_, c2 = k2_, c3 = k3_, c4 = k4_;

    if(del_s < 0.001) {
         c1 = 0.0;
         c3 = 0.0;
         c4 = 0.0;
    }

    direction_ = 1;

    if (std::abs(alpha) > 3) {
        alpha = 0.0;
        direction_ = -1;
    }


    action_[0] = alpha;
    action_[1] = direction_ * del_s;
    action_[2] = angle_diff(del_theta , alpha);
    //action_[2] = del_theta - alpha;

    //Initialize the random generator

    double std_dev1 = std::sqrt(c1 * action_[0] * action_[0] + c2 * action_[1] * action_[1]);
    double std_dev2 = std::sqrt(c3 * action_[1] * action_[1] + c4 * action_[0] * action_[0] + c4 * action_[2] * action_[2]);
    double std_dev3 = std::sqrt(c1 * action_[2] * action_[2] + c2 * action_[1] * action_[1]);

    // std::random_device rd;
    // std::mt19937 gen(rd());
    double mean = 0.0;
    double mean_angle = 0.0;
    N1.param( decltype(N1)::param_type{ mean_angle, std_dev1 } ) ;
    N2.param( decltype(N2)::param_type{ mean, std_dev2 } ) ;
    N3.param( decltype(N3)::param_type{ mean_angle, std_dev3 } ) ;

    //Sample noise for action from normal distribution

    //Update prevPose_
    prevPose_ = odometry;

    return true;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    std::random_device rd;
    std::mt19937 gen(rd());

    double eps1_ = N1(gen);
    double eps2_ = N2(gen);
    double eps3_ = N3(gen);

    particle_t new_sample = {0};
    pose_xyt_t new_pose = {0};

    new_pose.x = sample.pose.x + (action_[1] + eps2_) * cos(sample.pose.theta + action_[0] + eps1_);
    new_pose.y = sample.pose.y + (action_[1] + eps2_) * sin(sample.pose.theta + action_[0] + eps1_);
    new_pose.theta = wrap_to_pi(sample.pose.theta + action_[2] + action_[0] + eps1_ + eps3_);
    new_pose.utime = prevPose_.utime;

    new_sample.parent_pose = sample.pose;
    new_sample.pose = new_pose;
    new_sample.weight = sample.weight;
    return new_sample;
}
