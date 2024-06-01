//包含pluginlib的头文件，使用pluginlib的宏来注册插件
#include <pluginlib/class_list_macros.h>
#include "Square.h"

//注册插件，宏参数：plugin的实现类，plugin的基类
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon);