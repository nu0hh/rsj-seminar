#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class rsj_robot_test_node
{
private:
    nav_msgs::Odometry odom;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_scan;
    ros::Publisher pub_twist;
    sensor_msgs::LaserScan isens;

    // 壁との距離 [m]
    const float distanceSide = 0.5;
    const float LARGE_VALUE = 99999.;

    //angularvel
    float angularVel = 0.0;
    float linearVel = 0.0;

    // URGからの読み取り値
    float urgRight, urgLeft, urgFront;

    //角度の配列指定
    int ifront, iright, ileft;

    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        odom = *msg;
    }

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        float dtheta;
        //dtheta = (msg->angle_max - msg->angle_min)/msg->ranges.size();
        dtheta = msg->angle_increment;
        iright  = (-M_PI/2.-msg->angle_min)/dtheta;
        ileft  = (M_PI/2.-msg->angle_min)/dtheta;
        ifront = (0.-msg->angle_min)/dtheta;

        //msg->ranges.size();  726 -44 = 682

        // front
        urgFront = msg->ranges[ifront];//frontversion
        if ((urgFront < msg->range_min)|| // エラー値の場合
            (urgFront > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(urgFront)))       // 無限遠の場合
        {
        }
        else
        {
        }
        // side
        urgRight = msg->ranges[iright];//rightversion
        if ((urgRight < msg->range_min)|| // エラー値の場合
            (urgRight > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(urgRight)))       // 無限遠の場合
        {
        }
        else
        {
        }
        urgLeft = msg->ranges[ileft];//leftversion
        if ((urgLeft < msg->range_min)|| // エラー値の場合
            (urgLeft > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(urgLeft)))       // 無限遠の場合
        {
        }
        else
        {
        }

        //メッセージを保存
        isens = *msg;

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

        // 指定した壁からの距離(distanceSide)からのズレから角速度を計算する際の係数 [rad/s/m]
        float factorAngularVel = 0.5;

        //start time
        ros::Time start = ros::Time::now();

        ros::Rate rate(10.0);
        while(ros::ok())
        {
            ros::spinOnce();
            //passed time
            ros::Time now = ros::Time::now();

            geometry_msgs::Twist cmd_vel;

            //壁に一番近い距離
		float minDistance = 10;
		int minAngle;
		
            for (int i = ifront-100; i < ifront + 100; i++)//前方何度かを断続的にスキャン
            {
                float Distance = isens.ranges[i];//
                if ((Distance < isens.range_min)|| // エラー値の場合
                    (Distance > isens.range_max)|| // 測定範囲外の場合
                    (std::isnan(Distance)))       // 無限遠の場合
                {}
                else
                {
                    if(Distance < minDistance)
                    {
                        minDistance = Distance;
                        minAngle = i;
                    }
                }
            }

        int diff = ifront - minAngle;
        if (diff > 0)
		{
            angularVel = -0.5;
		}
        else if (diff < 0)
        {
            angularVel  = 0.5;
        }
        else
        {
            angularVel = 0.0;
        }


		linearVel = 0.0;


            if (urgFront != 0.0)//センサーが始まってから動き出す
            {
                ROS_INFO("----------------------------------");
                ROS_INFO("linear.x = %0.2f, angular.z = %0.2f", odom.twist.twist.linear.x, odom.twist.twist.angular.z);
                ROS_INFO("poxition.x = %0.2f, rad = %0.2f, (angle = %0.2f)",odom.pose.pose.position.x, tf::getYaw(odom.pose.pose.orientation), tf::getYaw(odom.pose.pose.orientation)*180/M_PI);
                ROS_INFO("right= %0.2f", urgRight);

                cmd_vel.linear.x = linearVel;
                cmd_vel.angular.z = angularVel;
            }

            if(urgFront < 0.2 || urgRight < 0.2 || urgLeft < 0.2)
            {
                //ROS_INFO("stopping:urg");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }

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

