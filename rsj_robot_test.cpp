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
    const float distanceSide = 0.3;
    const float LARGE_VALUE = 99999.;
    //angularvel
    float angularVel = 0.0;
    float linearVel = 0.0;

    // URGからの読み取り値
    float urgRight, urgLeft, urgFront;

    //from relative angle
    float myurgFront, myurgRight, myurgLeft;

    //tes
    float tes;

    //斜め１５よん方向
    float migiue, migisita, hidariue, hidarisita;

    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
//        ROS_INFO("positionX %f angularZ%f",
//            msg->pose.pose.position.x, msg->pose.pose.position.z);
        odom = *msg;
    }

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        //absolute angule--------------------------------
        int iright,ifront, ileft;
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
            //ROS_INFO("front-range: measurement error");
            //urgFront = msg->range_max;
        }
        else
        {
            //ROS_INFO("front-range: %0.3f",urgFront);
        }
        // side
        urgRight = msg->ranges[iright];//rightversion
        if ((urgRight < msg->range_min)|| // エラー値の場合
            (urgRight > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(urgRight)))       // 無限遠の場合
        {
            //ROS_INFO("right-range: measurement error");
            //urgSide = msg->range_max;
        }
        else
        {
            //ROS_INFO("side-range: %0.3f",urgSide);
        }
        urgLeft = msg->ranges[ileft];//leftversion
        if ((urgLeft < msg->range_min)|| // エラー値の場合
            (urgLeft > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(urgLeft)))       // 無限遠の場合
        {
            //ROS_INFO("left-range: measurement error");
            //urgSide = msg->range_max;
        }
        else
        {
            //ROS_INFO("side-range: %0.3f",urgSide);
        }

        //relative angule----------------------
        int myright, myfront, myleft;
        myright  = (-M_PI/2.-msg->angle_min + tf::getYaw(odom.pose.pose.orientation))/dtheta;
        myleft  = (M_PI/2.-msg->angle_min + tf::getYaw(odom.pose.pose.orientation))/dtheta;
        myfront = (0.-msg->angle_min + tf::getYaw(odom.pose.pose.orientation))/dtheta;

        tes = myright*dtheta;
        // front
        //kakonoisan myurgFront = cos(tf::getYaw(odom.pose.pose.orientation))*msg->ranges[ifront];//frontversion
        myurgFront = msg->ranges[myfront];//frontversion
        if ((myurgFront < msg->range_min)|| // エラー値の場合
            (myurgFront > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(myurgFront)))       // 無限遠の場合
        {
        //    ROS_INFO("myfront-range: measurement error");
            //urgFront = msg->range_max;
        }
        // side
        myurgRight = msg->ranges[myright];//rightversion
        if ((myurgRight < msg->range_min)|| // エラー値の場合
            (myurgRight > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(myurgRight)))       // 無限遠の場合
        {
        //    ROS_INFO("myright-range: measurement error");
            //urgSide = msg->range_max;
        }
        myurgLeft = msg->ranges[myleft];//leftversion
        if ((myurgLeft < msg->range_min)|| // エラー値の場合
            (myurgLeft > msg->range_max)|| // 測定範囲外の場合
            (std::isnan(myurgLeft)))       // 無限遠の場合
        {
         //   ROS_INFO("myleft-range: measurement error");
            //urgSide = msg->range_max;
        }

        //斜め
        int s;
        s = (-M_PI/2.-msg->angle_min+15*M_PI/180)/dtheta;
        migiue = msg->ranges[s];
        //int s;
        s = (-M_PI/2.-msg->angle_min-15*M_PI/180)/dtheta;
        migisita = msg->ranges[s];
        //int s;
        s = (M_PI/2.-msg->angle_min-15*M_PI/180)/dtheta;
        hidariue = msg->ranges[s];
        //int s;
        s = (M_PI/2.-msg->angle_min+15*M_PI/180)/dtheta;
        hidarisita = msg->ranges[s];

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
        float factorAngularVel = 0.7;
        //start time
        ros::Time start = ros::Time::now();
        //間隔設定
        int k = 0;

        ros::Rate rate(10.0);
        while(ros::ok())
        {
            ros::spinOnce();
            //passed time
            ros::Time now = ros::Time::now();

            geometry_msgs::Twist cmd_vel;

            //float mynum = cos(tf::getYaw(odom.pose.pose.orientation))*urgRight-distanceSide;
            float mynum = urgRight-distanceSide;
            float margin = 0.0;

            if (migiue - migisita > 0.001)//sitahanen
            {
                if (urgRight > 0.3)
                {
                    angularVel = -0.3;
                }
                else if (urgRight < 0.3)
                {
                    angularVel = 0.0;
                }
            }
            else
            {
                angularVel = 0.0;
            }


            if (urgFront != 0.0)//run when urg starts
            {/*
                //ROS_INFO("angle= %f, right= %f", odom.twist.twist.angular.z, urgRight);
                // ROS_INFO("front=%f right=%f angVel=%f",urgRight,myurgRight,angularVel);
                ROS_INFO("----------------------------------");
                ROS_INFO("linear.x = %0.2f, angular.z = %0.2f", odom.twist.twist.linear.x, odom.twist.twist.angular.z);
                ROS_INFO("poxition.x = %0.2f, rad = %0.2f, (angle = %0.2f)",odom.pose.pose.position.x, tf::getYaw(odom.pose.pose.orientation), tf::getYaw(odom.pose.pose.orientation)*180/M_PI);
                ROS_INFO("right= %0.2f abRight = %2.2f", urgRight, myurgRight);
                ROS_INFO("rad= %0.2f abrad = %2.2f",tf::getYaw(odom.pose.pose.orientation) , tes);
                ROS_INFO("angle= %0.2f abangle = %2.2f", tf::getYaw(odom.pose.pose.orientation)*180/M_PI, -90.0+tf::getYaw(odom.pose.pose.orientation)*180/M_PI);
                ROS_INFO("distance= %0.2f", mynum);
*/
                cmd_vel.linear.x = 0.3;
                cmd_vel.angular.z = angularVel;
            }

            if(urgFront < 0.2 || urgRight < 0.2 || urgLeft < 0.2)
            {
                //ROS_INFO("stopping:urg");
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
            }
            else if(myurgFront < 0.1 || myurgRight < 0.1 || myurgLeft < 0.1)
            {
                //ROS_INFO("stopping:my");
                //cmd_vel.linear.x = 0.0;
                //cmd_vel.angular.z = 0.0;
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

