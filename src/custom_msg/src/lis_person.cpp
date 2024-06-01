#include "ros/ros.h"
#include "person_pkg/Person.h"

void do_Msg(const person_pkg::Person::ConstPtr& p)
{
    ROS_INFO("我叫：%s,今年：%d岁，身高：%d",p->name.c_str(),p->age,p->height);
}

int main(int argc,char**argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"listener_person");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<person_pkg::Person>("chatter_person",10,do_Msg);
    ros::spin();
    return 0;
}