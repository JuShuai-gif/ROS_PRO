#include "ros/ros.h"
#include "person_gdb/PersonGDB.h" // 包含自定义消息类型的头文件

class CallBackFun
{
public:
    int num{15};

public:
    void do_Msg(const person_gdb::PersonGDB::ConstPtr &p)
    {
        // 尝试使用未初始化的指针
        int *ptr;
        //这里故意插入会引起段错误，导致程序崩溃的代码，看看gdb会输出什么

        ROS_INFO("Attempting to use uninitialized pointer...");
        if (ptr != nullptr)
        {
            ROS_INFO("Value: %d", *ptr);
        }
        else
        {
            ROS_INFO("Error: Null pointer!");
        }
        ROS_INFO("我叫：%s,今年：%d岁,身高：%d", p->name.c_str(), p->age, p->height);
    }
};

class LisPerson
{
public:
    ros::NodeHandle nh_{};
    ros::Subscriber sub;
    CallBackFun callback_;

public:
    LisPerson(ros::NodeHandle nh, CallBackFun callback) : nh_(nh), callback_(callback)
    {
        // 注意此处修改
        sub = nh.subscribe("chatter_person", 10, &CallBackFun::do_Msg, &callback_);
        int *b = 0;
        *b = 10;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener_person");
    ros::NodeHandle nh;

    CallBackFun callback;
    LisPerson listener(nh, callback);

    ros::spin();

    return 0;
}
