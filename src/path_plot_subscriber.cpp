#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class PathPlotSubscriber : public rclcpp::Node
{
public:
    PathPlotSubscriber()
        : Node("path_plot_subscriber")
    {
        // Create the subscription
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/path_plot_publisher", 10,
            std::bind(&PathPlotSubscriber::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /path_plot_publisher");
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());

        for (size_t i = 0; i < msg->poses.size(); ++i)
        {
            const auto &pose = msg->poses[i];
            RCLCPP_INFO(
                this->get_logger(),
                "Pose[%zu]: position(x=%.2f, y=%.2f, z=%.2f), orientation(x=%.2f, y=%.2f, z=%.2f, w=%.2f)",
                i,
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlotSubscriber>());
    rclcpp::shutdown();
    return 0;
}
