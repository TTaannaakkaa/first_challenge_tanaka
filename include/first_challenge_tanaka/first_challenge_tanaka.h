#ifndef FIRST_CHALLENGE_TANAKA_H
#define FIRST_CHALLENGE_TANAKA_H

#include <ros/ros.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>
#include <math.h>

class FirstChallenge
{
    public:
        FirstChallenge();
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
        float scan();

        void run(float v,float o);

        int hz_;

        float init_angl_;
        float current_angle_;


        nav_msgs::Odometry odometry_;
        sensor_msgs::LaserScan laser_;
        roomba_500driver_meiji::RoombaCtrl cmd_vel_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber odom_sub_;
        ros::Subscriber laser_sub_;
        ros::Publisher cmd_vel_pub_;

};

#endif
