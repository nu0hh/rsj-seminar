#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

class rsj_robot_test_node
{
private:
    nav_msgs::Odometry odom;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_scan;
    ros::Publisher pub_twist;

    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        ROS_INFO("vel %f",
            msg->twist.twist.angular.z);
        odom = *msg;
    }

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        int iside,ifront;
        float dtheta;
        dtheta = (msg->angle_max-msg->angle_min)/msg->ranges.size();
        iside  = (-3.1415/2.-msg->angle_min)/dtheta;
        ifront = (0.-msg->angle_min)/dtheta;
	// front
        //if(msg->ranges[msg->ranges.size() / 2] < msg->range_min)
        if(msg->ranges[ifront] < msg->range_min)
        {
            ROS_INFO("front-range: measurement error");
        }
        else
        {
            ROS_INFO("front-range: %0.3f",
                //msg->ranges[msg->ranges.size() / 2]);
                msg->ranges[ifront]);
        }
	// side
        if(msg->ranges[iside] < msg->range_min)
        {
            ROS_INFO("side-range: measurement error");
        }
        else
        {
            ROS_INFO("side-range: %0.3f",
                msg->ranges[iside]);
	}
    }

public:
    rsj_robot_test_node()
    {
        ros::NodeHandle nh("~");
        pub_twist = nh.advertise<geometry_msgs::Twist>(
                    "/ypspur_ros/cmd_vel", 5);//send msg to topic
        sub_odom = nh.subscribe("/ypspur_ros/odom", 5,
                                &rsj_robot_test_node::cb_odom, this);
        sub_scan = nh.subscribe("/scan", 5,
                    &rsj_robot_test_node::cb_scan, this);
        odom.pose.pose.orientation.w = 1.0;
        odom.pose.pose.position.x = 0.0;
    }
    void mainloop()
    {
        ROS_INFO("Hello ROS World!");

        ros::Rate rate(10.0);
        while(ros::ok())
        {
            ros::spinOnce();

            geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
/*
            if(odom.pose.pose.position.x < 1.0) {
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = 0.0;
            }
            else if(tf::getYaw(odom.pose.pose.orientation) > -1.57)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.5;
            }
            else if (odom.pose.pose.position.x > 0.0)
            {
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = 0.0;
            }
            else if (odom.pose.pose.position.x <= 0.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
*/
            pub_twist.publish(cmd_vel);

            rate.sleep();
        }
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rsj_robot_test_node");//,,node name

    rsj_robot_test_node robot_test;//instance

    robot_test.mainloop();
}

