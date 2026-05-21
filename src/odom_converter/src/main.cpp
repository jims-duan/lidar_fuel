#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class OdometryPublisher {
public:
    OdometryPublisher() {
        // 创建发布者
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/sensor_pose", 10);
        
        // 创建订阅者，订阅 Odometry 消息
        odom_sub_ = nh_.subscribe("/Odom_high_freq", 10, &OdometryPublisher::odomCallback, this);
        
        // 定义要添加的额外旋转（四元数形式：x, y, z, w）
        additional_quat_[0] = 0.5;   // x
        additional_quat_[1] = -0.5;  // y
        additional_quat_[2] = 0.5;   // z
        additional_quat_[3] = -0.5;  // w
        
        // 验证额外旋转四元数是否有效
        double norm = sqrt(additional_quat_[0]*additional_quat_[0] + 
                          additional_quat_[1]*additional_quat_[1] + 
                          additional_quat_[2]*additional_quat_[2] + 
                          additional_quat_[3]*additional_quat_[3]);
        
        if (fabs(norm - 1.0) > 0.001) {
            ROS_WARN("额外旋转四元数模长不是1: %.6f，将进行归一化", norm);
            for (int i = 0; i < 4; i++) {
                additional_quat_[i] /= norm;
            }
        }
        
        seq_ = 0;
        
        ROS_INFO("开始运行，等待接收 /Odom_high_freq 消息...");
        ROS_INFO("将在原始旋转基础上添加额外旋转: (%.3f, %.3f, %.3f, %.3f)", 
                 additional_quat_[0], additional_quat_[1], 
                 additional_quat_[2], additional_quat_[3]);
    }
    
    // 四元数乘法: q = q1 * q2
    void quatMultiply(const double q1[4], const double q2[4], double result[4]) {
        double x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
        double x2 = q2[0], y2 = q2[1], z2 = q2[2], w2 = q2[3];
        
        result[0] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // x
        result[1] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // y
        result[2] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // z
        result[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // w
    }
    
    // 应用额外旋转到原始四元数
    void applyRotation(const double original_quat[4], const double additional_quat[4], double result[4]) {
        // 四元数乘法：先应用原始旋转，再应用额外旋转
        quatMultiply(original_quat, additional_quat, result);
    }
    
    // 回调函数：处理接收到的 Odometry 消息
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        int current_seq = seq_++;
        
        // ========== 1. 发布 Odometry 格式 ==========
        nav_msgs::Odometry odom_msg;
        
        // 设置 header
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.seq = current_seq;
        odom_msg.header.frame_id = "/simulator";
        odom_msg.child_frame_id = "";
        
        // 复制 Pose 数据
        odom_msg.pose.pose = msg->pose.pose;
        odom_msg.pose.covariance = msg->pose.covariance;
        
        // 复制 Twist 数据
        odom_msg.twist.twist = msg->twist.twist;
        odom_msg.twist.covariance = msg->twist.covariance;
        
        // 发布 Odometry
        odom_pub_.publish(odom_msg);
        
        // ========== 2. 发布 PoseStamped 格式（添加旋转后）==========
        // 获取原始四元数
        double original_quat[4] = {
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        };
        
        // 应用额外旋转
        double new_quat[4];
        applyRotation(original_quat, additional_quat_, new_quat);
        
        // 创建并发布 PoseStamped
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.seq = current_seq;
        pose_msg.header.frame_id = "/map";
        
        // 复制位置数据（x + 0.1）
        pose_msg.pose.position.x = msg->pose.pose.position.x + 0.1;
        pose_msg.pose.position.y = msg->pose.pose.position.y;
        pose_msg.pose.position.z = msg->pose.pose.position.z;
        
        // 设置旋转后的四元数
        pose_msg.pose.orientation.x = new_quat[0];
        pose_msg.pose.orientation.y = new_quat[1];
        pose_msg.pose.orientation.z = new_quat[2];
        pose_msg.pose.orientation.w = new_quat[3];
        
        // 发布
        pose_pub_.publish(pose_msg);
        
        // 打印日志（每10帧打印一次，可选）
        if (current_seq % 100 == 0) {
            ROS_INFO("已发布 - seq: %d", current_seq);
            ROS_INFO("  原始旋转: (%.3f, %.3f, %.3f, %.3f)", 
                    original_quat[0], original_quat[1], 
                    original_quat[2], original_quat[3]);
            ROS_INFO("  结果旋转: (%.3f, %.3f, %.3f, %.3f)", 
                    new_quat[0], new_quat[1], new_quat[2], new_quat[3]);
        }
    }
    
private:
    ros::NodeHandle nh_;                          // ROS 节点句柄
    ros::Publisher odom_pub_;                     // Odometry 发布者
    ros::Publisher pose_pub_;                     // PoseStamped 发布者
    ros::Subscriber odom_sub_;                    // 订阅者
    int seq_;                                     // 序列号
    double additional_quat_[4];                   // 额外旋转四元数
};

int main(int argc, char** argv) {
    // 设置中文编码（针对终端输出）
    setlocale(LC_ALL, "zh_CN.UTF-8");
    // 初始化 ROS 节点
    ros::init(argc, argv, "odometry_publisher");
    
    // 创建类实例
    OdometryPublisher node;
    
    // 循环等待回调
    ros::spin();
    
    return 0;
}