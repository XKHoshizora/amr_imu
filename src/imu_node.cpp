#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

ros::Publisher vel_pub;

void IMUCallback(sensor_msgs::Imu msg){
    // 判断协方差矩阵的第一个数，若是 -1，则说明该数据无效，不再继续读取
    if (msg.orientation_covariance[0] == -1) {
        return;
    }

    // 将接收到的四元数数据转换为 TF 的四元数对象
    tf::Quaternion quaternion(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );

    // 定义用于接收滚转、俯仰、朝向数据的变量
    double roll, pitch,yaw;
    // 先将 TF 的四元数对象 quaternion 转换为 TF 的 3x3 矩阵对象
    // 然后调用 3x3 矩阵对象的 getRPY() 函数，将其转换为欧拉角
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // 将弧度转换为角度
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);

    double target_yaw = 90; // 目标朝向角
    double diff_angle = target_yaw - yaw; // 当前朝向角与目标朝向角的差值

    geometry_msgs::Twist vel_cmd;
    vel_cmd.angular.z = diff_angle * 0.01; // 设置与差值的比例，即可实现差值越大转速越快，差值越小转速越慢（比例控制）
    vel_cmd.linear.x = 0.1;
    vel_pub.publish(vel_cmd);
}

int main(int argc, char **argv){
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "imu_node");

    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, IMUCallback);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin(); // 维持程序运行

    return 0;
}