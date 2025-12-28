#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp> // 【新增】引入可视化消息头文件
#include <moveit/move_group_interface/move_group_interface.h>

// 使用占位符，方便回调函数绑定
using std::placeholders::_1;

class RobotPlannerNode : public rclcpp::Node {
public:
    RobotPlannerNode() : Node("robot_planner_node") {
        // 【修改建议】队列长度设为 1，对于遥操作，我们要丢弃旧数据，只取最新的
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vr_target_pose", 
            1, 
            std::bind(&RobotPlannerNode::topic_callback, this, _1));

        // 【新增】创建 Marker 发布者，话题名为 /vr_target_marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/vr_target_marker", 10);

        RCLCPP_INFO(this->get_logger(), "Robot Planner Ready: Listening to VR data...");
    }

    // 初始化 MoveIt (必须在节点启动后调用)
    void init_moveit() {
        // 【这里就是组】"panda_arm" 是组名，如果报错找不到组，请检查你的 SRDF 文件
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fr3_arm");
        
        // 设置最大速度/加速度比例，安全起见先设低一点
        move_group_->setMaxVelocityScalingFactor(0.2);
        move_group_->setMaxAccelerationScalingFactor(0.2);
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Target: x=%.2f, y=%.2f", 
                    msg->pose.position.x, msg->pose.position.y);

        // 【新增】在规划开始前，先发布可视化 Marker，这样无论规划是否成功，Rviz里都能看到目标
        publish_marker(msg->pose);

        // TODO: 在这里加入你的"动态尺度自适应映射"逻辑
        // double scale = ...;

        // 设置 MoveIt 目标并规划 (为保证实时性，实际控制中可能需要用 MoveIt Servo)
        // 1. 设置目标 (Set Target)
        // 【建议】明确告诉 MoveIt 这个位姿是相对于哪个坐标系的
        // 如果你的 TF 树配置好了，MoveIt 会自动转换，但这行代码能让逻辑更严谨
        move_group_->setPoseReferenceFrame("base"); 
        move_group_->setPoseTarget(msg->pose);

        // 2. 创建并计算规划 (Create a plan)
        // 使用 const 引用，确保 my_plan 不会被意外修改
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        // 这里的 plan() 会根据"panda_arm"组的运动学求解器来计算
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        // 3. 执行规划 (Execute)
        if (success) {
            RCLCPP_INFO(this->get_logger(), "规划成功，开始执行移动...");
            
            // 注意：execute 是阻塞的！机械臂动完之前，回调函数会卡在这里
            move_group_->execute(my_plan);
            
            RCLCPP_INFO(this->get_logger(), "移动完成，准备接收下一个 VR 指令");
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败！目标不可达（太远或有碰撞）");
        }
    }

    // 【新增】辅助函数：构造并发布一个红色的箭头 Marker
    void publish_marker(const geometry_msgs::msg::Pose &pose) {
        visualization_msgs::msg::Marker marker;
        // 必须和你的 setPoseReferenceFrame 保持一致
        marker.header.frame_id = "base"; 
        marker.header.stamp = this->now();
        marker.ns = "vr_target";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW; // 形状：箭头
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置位姿
        marker.pose = pose;

        // 设置尺寸 (单位：米)
        marker.scale.x = 0.1;  // 箭头长度
        marker.scale.y = 0.01; // 箭头宽度
        marker.scale.z = 0.01; // 箭头高度

        // 设置颜色 (RGBA) -> 红色，不透明
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        // 0表示一直显示，直到新的消息替换它
        marker.lifetime = rclcpp::Duration::from_seconds(0); 

        marker_pub_->publish(marker);
    }
    

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    // 【新增】声明 Marker 发布者
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPlannerNode>();

    // 使用多线程执行器，这对 MoveIt 很重要
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 必须在 shared_ptr 创建之后初始化 MoveIt
    node->init_moveit();

    executor.spin();
    rclcpp::shutdown();
    return 0;
}