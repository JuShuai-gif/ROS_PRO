#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>

class NavPublisherSubscriber
{
public:
    NavPublisherSubscriber() : nh("~")
    {
        // 创建ROS发布者，发布nav_msgs::Odometry消息
        odom_pub = nh.advertise<nav_msgs::Odometry>("/state_estimation", 10);

        // 创建ROS订阅者，订阅sensor_msgs::Joy消息
        joy_sub = nh.subscribe("/joy", 10, &NavPublisherSubscriber::joyCallback, this);

        // 创建ROS订阅者，订阅geometry_msgs::PointStamped消息
        point_sub = nh.subscribe("/way_point", 10, &NavPublisherSubscriber::pointCallback, this);
    }

    void publish()
    {
        // 创建一个nav_msgs::Odometry消息
        nav_msgs::Odometry odom_msg;

        // 设置一些假数据，这里只是演示
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = 1.0;
        odom_msg.pose.pose.position.y = 2.0;
        odom_msg.pose.pose.position.z = 3.0;

        // 发布消息
        odom_msg.header.stamp = ros::Time::now();
        odom_pub.publish(odom_msg);
        ROS_INFO("odom 发布了！！！");
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        // 处理sensor_msgs::Joy消息
        // 示例：读取手柄的某个按钮值
        if (joy_msg->buttons[0] == 1)
        {
            ROS_INFO("Button 0 pressed!");
        }
    }

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& point_msg)
    {
        // 处理geometry_msgs::PointStamped消息
        // 示例：读取PointStamped消息的坐标值
        ROS_INFO("pointCallback");
        ROS_INFO("Received point at (%f, %f, %f)", point_msg->point.x, point_msg->point.y, point_msg->point.z);
    }

private:
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber point_sub;
};

int main(int argc, char** argv)
{
    // 设置编码，保证输出不是乱码
    setlocale(LC_ALL,"");
    // 初始化ROS节点
    ros::init(argc, argv, "nav_publisher_subscriber_node");

    NavPublisherSubscriber node;

    ros::Rate loop_rate(1); // 发布频率1Hz
    while (ros::ok())
    {
        node.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
