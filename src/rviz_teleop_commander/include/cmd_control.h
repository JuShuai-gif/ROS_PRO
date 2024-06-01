#ifndef CMD_CONTROL_H
#define CMD_CONTROL_H

#include <stdio.h>

//所需要包含的头文件
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   //plugin基类的头文件


#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <geometry_msgs/Twist.h>
#include <QDebug>
#include <QCheckBox>
#include <QSlider>
namespace rviz_teleop_commander {
class cmd_control:public rviz::Panel
{
    Q_OBJECT
public:
    // 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可
    cmd_control(QWidget* parent=0);
    // 重载rviz::Panel积累中的函数，用于保存、加载配置文件中的数据，在我们这个plugin中，数据就是topic的名称
//    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;
    void move_base(char k,float speed_linear,float speed_trun);
public Q_SLOTS:
    void slot_cmd_control();
    void on_Slider_raw_valueChanged(int);
    // 内部变量.
protected:
    QPushButton* pushButton_i;
    QPushButton* pushButton_u;
    QPushButton* pushButton_o;
    QPushButton* pushButton_j;
    QPushButton* pushButton_l;
    QPushButton* pushButton_m;
    QPushButton* pushButton_back;
    QPushButton* pushButton_backr;

    QCheckBox* is_all_check;

    QSlider* yaw_slider;
    QSlider* linera_slider;
    // ROS的publisher，用来发布速度topic
    ros::Publisher velocity_publisher_;
    QString output_topic_="cmd_vel";

    // The ROS node handle.
    ros::NodeHandle nh_;

    // 当前保存的线速度和角速度值
    float linear_velocity_;
    float angular_velocity_;

};
}

#endif // CMD_CONTROL_H
