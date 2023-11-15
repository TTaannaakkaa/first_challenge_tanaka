#include "first_challenge_tanaka/first_challenge_tanaka.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    odom_sub_ = nh_.subscribe("/roomba/odometry", 1, &FirstChallenge::odometry_callback, this);
    laser_sub_ = nh_.subscribe("/scan", 1, &FirstChallenge::laser_callback, this);
    cmd_vel_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

void FirstChallenge::straight()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.30;
    cmd_vel_.cntl.angular.z = 0.0;

    cmd_vel_pub_.publish(cmd_vel_);
}

void FirstChallenge::turn()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.4;

    cmd_vel_pub_.publish(cmd_vel_);
}

void FirstChallenge::stop()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.0;

    cmd_vel_pub_.publish(cmd_vel_);
}

float FirstChallenge::scan()
{
    float sum, ave = 0;
    int n = 25;
    int front = laser_.ranges.size() / 2;
    for(int i= front - n;i<front + n;i++)
    {
        sum += laser_.ranges[i];
    }
    ave = sum / 2 / n;

    return ave;
}

void FirstChallenge::process()
{
    float yaw = 0;
    float yaw2 = 2.525;
    int count = 0;

    init_angl_ = tf2::getYaw(odometry_.pose.pose.orientation);

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        yaw = tf2::getYaw(odometry_.pose.pose.orientation);
//        printf("yaw = %lf\n",yaw);
        if(odometry_.pose.pose.position.x >= 1.0)
        {
            if(current_angle_ - init_angl_ >= -0.1 && count  > 50)
            {
                straight();
//                printf("d = %f\n",scan());
                if(scan() <= 0.50)
                {
                    stop();
                }
            }
//            printf("x2 = %f\n", odometry_.pose.pose.position.x);
            else
            {
                turn();
                current_angle_ = yaw;
                if(current_angle_ < 0.0)
                {
                    count += 1;
                }
            }
//            printf("current_angle_ = %f, count = %d\n",current_angle_,count);
//            printf("C - I = %f\n",current_angle_ - init_angl_);
        }
        else
        {
            straight();
//            printf("x1 = %f\n",odometry_.pose.pose.position.x);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge_tanaka");
    FirstChallenge first_challenge;
    first_challenge.process();
    return 0;
}
