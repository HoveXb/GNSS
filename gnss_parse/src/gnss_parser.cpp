#include "ros/ros.h"
#include "msg_position/msg_position_long.h"
#include <iostream>
#include <fstream>
#include<Eigen/Dense>

void parseGNSS(const msg_position::msg_position_long::ConstPtr& msg_p,std::ofstream * gnssGrab){
    // std::cout<<msg_p->x<<std::endl;
    double t=msg_p->header.stamp.sec+(msg_p->header.stamp.nsec/(1e9));
    *gnssGrab<<t<<" "<<msg_p->x<<" "<<msg_p->y<<" "<<msg_p->z<<" ";
    //角度制转弧度
    float roll = (msg_p->roll/180*M_PI);
    float pitch = (msg_p->pitch/180*M_PI);
    float yaw = (msg_p->yaw/180*M_PI);
    //欧拉角转四元数
    Eigen::AngleAxisf rollAngle(roll,Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch,Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw,Eigen::Vector3f::UnitZ());    
    Eigen::Quaternionf q;
    q=yawAngle*pitchAngle*rollAngle;
    *gnssGrab<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    //2.初始化 ROS 节点:命名(唯一)
    ros::init(argc,argv,"parse_gnss");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;

    std::ofstream gnssGrab("groudtruth.txt");
    gnssGrab << std::fixed;
    //4.实例化 订阅者 对象
    std::cout<<"writing gnss infotmation to txt"<<std::endl;
    ros::Subscriber sub = nh.subscribe<msg_position::msg_position_long>
                                        ("msg_gnss_prep_long",100,
                                        boost::bind(&parseGNSS, _1, &gnssGrab ));
    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}