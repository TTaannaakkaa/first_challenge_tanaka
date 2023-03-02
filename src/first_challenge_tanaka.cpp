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

void FirstChallenge::run(float v,float o)
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = v;
    cmd_vel_.cntl.angular.z = o;

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

// void GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw)
// {
//     tf2::Quaternion quat(q.x,q.y,q.z,q.w);
//     tf2::Matrix3x3(quad_t).getRPY(roll, pitch, yaw);
// }

void FirstChallenge::process()
{
    float roll, pitch, yaw = 0;
    float yaw2 = 2.525;
    float d = 0;
    int count = 0;
//    GetRPY(odometry_.pose.pose.orientation, roll, pitch, yaw);
    init_angl_ = tf2::getYaw(odometry_.pose.pose.orientation);

    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        ros::spinOnce();
        yaw = tf2::getYaw(odometry_.pose.pose.orientation);
//        printf("yaw = %lf\n",yaw);
//        GetRPY(odometry_callback_.pose.pose.orientation, roll, pitch, yaw);
        if(odometry_.pose.pose.position.x >= 1.0)
        {
            if(current_angle_ - init_angl_ >= -0.1 && count  > 50)
            {
                run(0.20,0.0);
//                printf("d = %f\n",scan());
                if(scan() <= 0.50)
                {
                    run(0.0,0.0);
                }
            }
//            printf("x2 = %f\n", odometry_.pose.pose.position.x);
            else
            {
                run(0.0,0.40);
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
            run(0.40,0);
//            printf("x1 = %f\n",odometry_.pose.pose.position.x);
        }

        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge_tanaka");
    FirstChallenge first_challenge;
    first_challenge.process();
    return 0;
}
