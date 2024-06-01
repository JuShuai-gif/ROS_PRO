#include "ros/ros.h"
#include "person_gdb/PersonGDB.h"

int main(int argc,char **argv){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"talker_person");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<person_gdb::PersonGDB>("chatter_person",100);
    person_gdb::PersonGDB p;
    p.name = "ghr";
    p.age = 25;
    p.height=180;
    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(p);
        p.age +=1;
        ROS_INFO("我叫：%s,今年：%d岁，身高：%d",p.name,p.age,p.height);

        r.sleep();
        ros::spinOnce();
    }
    return 0;
}