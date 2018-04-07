#include "MainWindow.h"
#include <QApplication>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    ros::init(argc, argv, "kitti_player");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    MainWindow w;

    // set stylesheet
    QFile file(":/BreezeStyleSheet/light.qss");
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream stream(&file);
    w.setStyleSheet(stream.readAll());

    w.ros_init(node, private_nh);
    w.show();

    return a.exec();
}
