#include "cmd_control.h"
#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <geometry_msgs/Twist.h>
#include <QDebug>

namespace rviz_teleop_commander
{
    cmd_control::cmd_control(QWidget *parent)
        : rviz::Panel(parent)
    {

        // 初始化ui
        QVBoxLayout *mainlayout = new QVBoxLayout;
        pushButton_i = new QPushButton;
        pushButton_i->setText("i");
        // 快捷键
        pushButton_i->setShortcut(QKeySequence(QLatin1String("i")));
        pushButton_i->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        pushButton_u = new QPushButton;
        // 快捷键
        pushButton_u->setShortcut(QKeySequence(QLatin1String("u")));
        pushButton_u->setText("u");
        pushButton_u->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        pushButton_o = new QPushButton;
        // 快捷键
        pushButton_o->setShortcut(QKeySequence(QLatin1String("o")));
        pushButton_o->setText("o");
        pushButton_o->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        QHBoxLayout *first = new QHBoxLayout;
        first->addWidget(pushButton_u);
        first->addWidget(pushButton_i);
        first->addWidget(pushButton_o);
        mainlayout->addLayout(first);

        pushButton_j = new QPushButton;
        pushButton_j->setText("j");
        // 快捷键
        pushButton_j->setShortcut(QKeySequence(QLatin1String("j")));
        pushButton_j->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        is_all_check = new QCheckBox;
        is_all_check->setText("使用全向轮模式");
        // 快捷键
        is_all_check->setShortcut(QKeySequence(QLatin1String("k")));
        pushButton_l = new QPushButton;
        pushButton_l->setText("l");
        // 快捷键
        pushButton_l->setShortcut(QKeySequence(QLatin1String("l")));
        pushButton_l->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        QHBoxLayout *second = new QHBoxLayout;
        second->addWidget(pushButton_j);
        second->addWidget(is_all_check);
        second->addWidget(pushButton_l);
        mainlayout->addLayout(second);

        pushButton_m = new QPushButton;
        // 快捷键
        pushButton_m->setShortcut(QKeySequence(QLatin1String("m")));
        pushButton_m->setText("m");
        pushButton_m->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        pushButton_back = new QPushButton;
        // 快捷键
        pushButton_back->setShortcut(QKeySequence(QLatin1String(",")));
        pushButton_back->setText(",");
        pushButton_back->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        pushButton_backr = new QPushButton;
        // 快捷键
        pushButton_backr->setShortcut(QKeySequence(QLatin1String(".")));
        pushButton_backr->setText(".");
        pushButton_backr->setStyleSheet("QPushButton#pushButton_i:pressed{background-color:rgb(239, 41, 41)}");
        QHBoxLayout *third = new QHBoxLayout;
        third->addWidget(pushButton_m);
        third->addWidget(pushButton_back);
        third->addWidget(pushButton_backr);
        mainlayout->addLayout(third);

        QLabel *linera_label = new QLabel;
        linera_label->setText("线速度（cm/s）");
        linera_slider = new QSlider;
        linera_slider->setOrientation(Qt::Horizontal); // 水平方向
        linera_slider->setRange(0, 100);
        linera_slider->setValue(50);
        QHBoxLayout *five = new QHBoxLayout;
        five->addWidget(linera_label);
        five->addWidget(linera_slider);
        mainlayout->addLayout(five);

        QLabel *yaw_label = new QLabel;
        yaw_label->setText("角速度（cm/s）");
        yaw_slider = new QSlider;
        yaw_slider->setOrientation(Qt::Horizontal); // 水平方向
        yaw_slider->setRange(0, 100);
        yaw_slider->setValue(100);
        QHBoxLayout *six = new QHBoxLayout;
        six->addWidget(yaw_label);
        six->addWidget(yaw_slider);
        mainlayout->addLayout(six);

        setLayout(mainlayout);

        // 绑定信号

        // 绑定速度控制按钮
        connect(pushButton_i, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_u, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_o, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_j, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_l, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_m, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_back, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));
        connect(pushButton_backr, SIGNAL(clicked()), this, SLOT(slot_cmd_control()));

        // 创建发布者
        velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>(output_topic_.toStdString(), 1);
    }

    void cmd_control::slot_cmd_control()
    {

        QPushButton *btn = qobject_cast<QPushButton *>(sender());
        char key = btn->text().toStdString()[0];
        // 速度
        float liner = linera_slider->value() * 0.01;
        float turn = yaw_slider->value() * 0.01;
        bool is_all = is_all_check->isChecked();
        switch (key)
        {
        case 'u':
            move_base(is_all ? 'U' : 'u', liner, turn);
            break;
        case 'i':
            move_base(is_all ? 'I' : 'i', liner, turn);
            break;
        case 'o':
            move_base(is_all ? 'O' : 'o', liner, turn);
            break;
        case 'j':
            move_base(is_all ? 'J' : 'j', liner, turn);
            break;
        case 'l':
            move_base(is_all ? 'L' : 'l', liner, turn);
            break;
        case 'm':
            move_base(is_all ? 'M' : 'm', liner, turn);
            break;
        case ',':
            move_base(is_all ? '<' : ',', liner, turn);
            break;
        case '.':
            move_base(is_all ? '>' : '.', liner, turn);
            break;
        }
    }
    void cmd_control::move_base(char k, float speed_linear, float speed_trun)
    {
        std::map<char, std::vector<float>> moveBindings{
            {'i', {1, 0, 0, 0}},
            {'o', {1, 0, 0, -1}},
            {'j', {0, 0, 0, 1}},
            {'l', {0, 0, 0, -1}},
            {'u', {1, 0, 0, 1}},
            {',', {-1, 0, 0, 0}},
            {'.', {-1, 0, 0, 1}},
            {'m', {-1, 0, 0, -1}},
            {'O', {1, -1, 0, 0}},
            {'I', {1, 0, 0, 0}},
            {'J', {0, 1, 0, 0}},
            {'L', {0, -1, 0, 0}},
            {'U', {1, 1, 0, 0}},
            {'<', {-1, 0, 0, 0}},
            {'>', {-1, -1, 0, 0}},
            {'M', {-1, 1, 0, 0}},
            {'t', {0, 0, 1, 0}},
            {'b', {0, 0, -1, 0}},
            {'k', {0, 0, 0, 0}},
            {'K', {0, 0, 0, 0}}};
        char key = k;
        // 计算是往哪个方向
        float x = moveBindings[key][0];
        float y = moveBindings[key][1];
        float z = moveBindings[key][2];
        float th = moveBindings[key][3];
        // 计算线速度和角速度
        float speed = speed_linear;
        float turn = speed_trun;
        // Update the Twist message
        geometry_msgs::Twist twist;
        twist.linear.x = x * speed;
        twist.linear.y = y * speed;
        twist.linear.z = z * speed;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = th * turn;

        // Publish it and resolve any remaining callbacks
        velocity_publisher_.publish(twist);
        ros::spinOnce();
    }

    // 重载父类的功能
    void cmd_control::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
        config.mapSetValue("Topic", output_topic_);
    }

}

// 声明此类是一个rviz的插件
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_teleop_commander::cmd_control, rviz::Panel)
// END_TUTORIAL
