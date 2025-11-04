
// Math
#include <cmath>
#include <chrono>

// ROS
#include "rclcpp/rclcpp.hpp"

// ROS Messages
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

// Custom ROS Messages
#include "pumpkin_msgs/srv/plan_motion.hpp"

// YAML Utility
#include <fstream>
#include "yaml_utils.h"


class MahajanPathPlanner : public rclcpp::Node
{
public:
    MahajanPathPlanner()
    : Node("mahajan_path_planner") 
    {
        // Create a client
        path_planner_client_ = this->create_client<pumpkin_msgs::srv::PlanMotion>("/generate_motion_plan");

        // Create a plot publisher
        path_plot_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/tool_path_poses", 5);
        
        // Create a joint trajectory publisher
        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/trajectory", 5);

        // Create a path line trace publisher
        path_line_trace_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/tool_path_trace", 5);

        // Initialize request data structure
        request_ = std::make_shared<pumpkin_msgs::srv::PlanMotion::Request>();

        // Set Path Parameters
        tooling_length_ = 0.025;        // depth of cut into pumpkin
        tooling_radius_ = 0.007;
        pumpkin_radius_ = 0.05;          // pumpkin radius

        moon_y_offset_ = 0.01;
        moon_x_axis_left_ = -0.06;
        moon_x_axis_right_ = 0.06;
        b_1_const_ = -0.06;             // some constant to define moon part 1
        moon_radius_const_ = 0.06;      // radius of moon
        
        bat_y1_ = 0.03;
        bat_x1_ = 0.035;
        bat_y2_ = 0.01;
        bat_x2_ = 0.01;
        bat_y3_ = 0.025;
        bat_y4_ = 0.02;
        

        // Define Output YAML Filename
        output_toolpath_yaml_filename = "toolpath_bat.yaml";
        output_trajectory_yaml_filename = "trajectory_bat.yaml";
        
        // Report Setup
        RCLCPP_INFO(this->get_logger(), "Node Setup Complete!");
    }

    /*
    * Generate Moon Path
    * */
    bool makeTopPath()
    {
        // Setup moon segment pose array
        auto top_segment = geometry_msgs::msg::PoseArray();

        // Stuff Header
        top_segment.header.frame_id = "pumpkin_face";
        top_segment.header.stamp = this->now();
        
        // Pose #1 - Appraoch pumkin
        geometry_msgs::msg::Pose approach_moon;
        approach_moon.position.x = moon_x_axis_left_;
        approach_moon.position.y = moon_y_offset_;
        approach_moon.position.z = 0.0;
        approach_moon.orientation.x = 0.0;
        approach_moon.orientation.y = 0.0;
        approach_moon.orientation.z = 0.0;
        approach_moon.orientation.w = 1.0;
        top_segment.poses.push_back(approach_moon);
        
        // Pose #2 - Cut into pumkin
        geometry_msgs::msg::Pose cut_moon;
        cut_moon.position.x = moon_x_axis_left_;
        cut_moon.position.y = moon_y_offset_;
        cut_moon.position.z = tooling_length_;
        cut_moon.orientation.x = 0.0;
        cut_moon.orientation.y = 0.0;
        cut_moon.orientation.z = 0.0;
        cut_moon.orientation.w = 1.0;
        top_segment.poses.push_back(cut_moon);

        // Pose loop #3: cut moon part 1
        for (double i = moon_x_axis_left_; i <= moon_x_axis_right_; i += tooling_radius_/2)
        {
            geometry_msgs::msg::Pose moon_part1;
            moon_part1.position.x = i;
            moon_part1.position.y = (b_1_const_ * sqrt(pow(moon_radius_const_, 2) - pow(i, 2))) / moon_radius_const_;
            moon_part1.position.z = tooling_length_;
            moon_part1.orientation.x = 0.0;
            moon_part1.orientation.y = 0.0;
            moon_part1.orientation.z = 0.0;
            moon_part1.orientation.w = 1.0;
            top_segment.poses.push_back(moon_part1);
        }
        
        // right side of bat
        geometry_msgs::msg::Pose bat_R_part1;
        bat_R_part1.position.x = bat_x1_;
        bat_R_part1.position.y = bat_y1_;
        bat_R_part1.position.z = tooling_length_;
        bat_R_part1.orientation.x = 0.0;
        bat_R_part1.orientation.y = 0.0;
        bat_R_part1.orientation.z = 0.0;
        bat_R_part1.orientation.w = 1.0;
        top_segment.poses.push_back(bat_R_part1);

        geometry_msgs::msg::Pose bat_R_part2;
        bat_R_part2.position.x = bat_x2_;
        bat_R_part2.position.y = bat_y2_;
        bat_R_part2.position.z = tooling_length_;
        bat_R_part2.orientation.x = 0.0;
        bat_R_part2.orientation.y = 0.0;
        bat_R_part2.orientation.z = 0.0;
        bat_R_part2.orientation.w = 1.0;
        top_segment.poses.push_back(bat_R_part2);

        geometry_msgs::msg::Pose bat_R_part3;
        bat_R_part3.position.x = bat_x2_;
        bat_R_part3.position.y = bat_y3_;
        bat_R_part3.position.z = tooling_length_;
        bat_R_part3.orientation.x = 0.0;
        bat_R_part3.orientation.y = 0.0;
        bat_R_part3.orientation.z = 0.0;
        bat_R_part3.orientation.w = 1.0;
        top_segment.poses.push_back(bat_R_part3);

        geometry_msgs::msg::Pose bat_R_part4;
        bat_R_part4.position.x = 0.0;
        bat_R_part4.position.y = bat_y4_;
        bat_R_part4.position.z = tooling_length_;
        bat_R_part4.orientation.x = 0.0;
        bat_R_part4.orientation.y = 0.0;
        bat_R_part4.orientation.z = 0.0;
        bat_R_part4.orientation.w = 1.0;
        top_segment.poses.push_back(bat_R_part4);

        // left side of bat
        geometry_msgs::msg::Pose bat_L_part1;
        bat_L_part1.position.x = -bat_x2_;
        bat_L_part1.position.y = bat_y3_;
        bat_L_part1.position.z = tooling_length_;
        bat_L_part1.orientation.x = 0.0;
        bat_L_part1.orientation.y = 0.0;
        bat_L_part1.orientation.z = 0.0;
        bat_L_part1.orientation.w = 1.0;
        top_segment.poses.push_back(bat_L_part1);

        geometry_msgs::msg::Pose bat_L_part2;
        bat_L_part2.position.x = -bat_x2_;
        bat_L_part2.position.y = bat_y2_;
        bat_L_part2.position.z = tooling_length_;
        bat_L_part2.orientation.x = 0.0;
        bat_L_part2.orientation.y = 0.0;
        bat_L_part2.orientation.z = 0.0;
        bat_L_part2.orientation.w = 1.0;
        top_segment.poses.push_back(bat_L_part2);

        geometry_msgs::msg::Pose bat_L_part3;
        bat_L_part3.position.x = -bat_x1_;
        bat_L_part3.position.y = bat_y1_;
        bat_L_part3.position.z = tooling_length_;
        bat_L_part3.orientation.x = 0.0;
        bat_L_part3.orientation.y = 0.0;
        bat_L_part3.orientation.z = 0.0;
        bat_L_part3.orientation.w = 1.0;
        top_segment.poses.push_back(bat_L_part3);

        geometry_msgs::msg::Pose bat_L_part4;
        bat_L_part4.position.x = moon_x_axis_left_;
        bat_R_part4.position.y = bat_y2_;
        bat_L_part4.position.z = tooling_length_;
        bat_L_part4.orientation.x = 0.0;
        bat_L_part4.orientation.y = 0.0;
        bat_L_part4.orientation.z = 0.0;
        bat_L_part4.orientation.w = 1.0;
        top_segment.poses.push_back(bat_L_part4);

        // Pose #5 - depart pumkin
        geometry_msgs::msg::Pose depart_moon;
        depart_moon.position.x = 0.0;
        depart_moon.position.y = moon_x_axis_left_;
        depart_moon.position.z = 0.0;
        depart_moon.orientation.x = 0.0;
        depart_moon.orientation.y = 0.0;
        depart_moon.orientation.z = 0.0;
        depart_moon.orientation.w = 0.0;
        top_segment.poses.push_back(depart_moon);


        // Plot the moon segment
        RCLCPP_INFO(this->get_logger(), "Top Done. Plotting now...");
        plotPathSegments(top_segment);

        // Save moon segment to request
        request_->path.push_back(top_segment);

        return false;
        } // MahajanPathPlanner::makeTopPath

    /*
    * Generate Star Path
    * */
    bool makeBottomPath()
    {
        // Setup star segment pose array
        auto bottom_segment = geometry_msgs::msg::PoseArray();

        // Stuff Header
        bottom_segment.header.frame_id = "pumpkin_face";
        bottom_segment.header.stamp = this->now();
        
        // Pose #6 - Aproach bottom part 1
        geometry_msgs::msg::Pose approach_bottom_part1;
        approach_bottom_part1.position.x = moon_x_axis_left_;
        approach_bottom_part1.position.y = moon_y_offset_;
        approach_bottom_part1.position.z = 0.0;
        approach_bottom_part1.orientation.x = 0.0;
        approach_bottom_part1.orientation.y = 0.0;
        approach_bottom_part1.orientation.z = 0.0;
        approach_bottom_part1.orientation.w = 1.0;
        bottom_segment.poses.push_back(approach_bottom_part1);

        // Pose #7 - cut into pumkin
        geometry_msgs::msg::Pose cut_bottom_part1;
        cut_bottom_part1.position.x = moon_x_axis_left_;
        cut_bottom_part1.position.y = moon_y_offset_;
        cut_bottom_part1.position.z = tooling_length_;
        cut_bottom_part1.orientation.x = 0.0;
        cut_bottom_part1.orientation.y = 0.0;
        cut_bottom_part1.orientation.z = 0.0;
        cut_bottom_part1.orientation.w = 1.0;
        bottom_segment.poses.push_back(cut_bottom_part1);

        for (double i = moon_x_axis_left_; i <= moon_x_axis_right_; i += tooling_radius_/2)
        {
            geometry_msgs::msg::Pose moon_part2;
            moon_part2.position.x = i;
            moon_part2.position.y = (-b_1_const_ * sqrt(pow(moon_radius_const_, 2) - pow(i, 2))) / moon_radius_const_;
            moon_part2.position.z = tooling_length_;
            moon_part2.orientation.x = 0.0;
            moon_part2.orientation.y = 0.0;
            moon_part2.orientation.z = 0.0;
            moon_part2.orientation.w = 1.0;
            bottom_segment.poses.push_back(moon_part2);
        }

        // Pose #8 - star part 1
        geometry_msgs::msg::Pose bat_bottom1;
        bat_bottom1.position.x = 0.0;
        bat_bottom1.position.y = -0.05;
        bat_bottom1.position.z = tooling_length_;
        bat_bottom1.orientation.x = 0.0;
        bat_bottom1.orientation.y = 0.0;
        bat_bottom1.orientation.z = 0.0;
        bat_bottom1.orientation.w = 1.0;
        bottom_segment.poses.push_back(bat_bottom1);

        geometry_msgs::msg::Pose bat_bottom2;
        bat_bottom2.position.x = moon_x_axis_left_;
        bat_bottom2.position.y = -0.01;
        bat_bottom2.position.z = tooling_length_;
        bat_bottom2.orientation.x = 0.0;
        bat_bottom2.orientation.y = 0.0;
        bat_bottom2.orientation.z = 0.0;
        bat_bottom2.orientation.w = 1.0;
        bottom_segment.poses.push_back(bat_bottom2);

        // Pose #14 - depart pumpkin
        geometry_msgs::msg::Pose depart_star_part2;
        depart_star_part2.position.x = moon_x_axis_left_;
        depart_star_part2.position.y = -0.01;
        depart_star_part2.position.z = 0.0;
        depart_star_part2.orientation.x = 0.0;
        depart_star_part2.orientation.y = 0.0;
        depart_star_part2.orientation.z = 0.0;
        depart_star_part2.orientation.w = 1.0;
        bottom_segment.poses.push_back(depart_star_part2);

        // Plot the star segment
        RCLCPP_INFO(this->get_logger(), "Bottom Done. Plotting now...");
        plotPathSegments(bottom_segment);

        // Save star segment to request
        request_->path.push_back(bottom_segment);

        return false;
    } // MahajanPathPlanner::makeBottomPath
 
    /*
    * Plot the entire tool path as one PoseArray
    */
    void plotToolPathPoses()
    {
        // Setup combined path pose array
        auto combined_path = geometry_msgs::msg::PoseArray();
        combined_path.header.frame_id = "pumpkin_face";
        combined_path.header.stamp = this->now();

        // Combine all segments into one PoseArray
        for (const auto& segment : request_->path)
        {
            for (const auto& pose : segment.poses)
            {
                combined_path.poses.push_back(pose);
            }
        }

        // Plot the combined path
        plotPathSegments(combined_path);
    } // MahajanPathPlanner::combineToolPath

    /*
    * Plot Single PoseArray for Visualization in RViz
    */
    void plotPathSegments(geometry_msgs::msg::PoseArray segment)
    {
        path_plot_publisher_->publish(segment);
        RCLCPP_INFO(this->get_logger(), "Sent PoseArray plot message");
    } // MahajanPathPlanner::plotPathSegments

    /*
    * Plot Trajectory for Visualization in RViz
    */
    void plotJointTrajectory()
    {
        joint_trajectory_publisher_->publish(response_->trajectory);
        RCLCPP_INFO(this->get_logger(), "Sent JointTrajectory plot message");
    } // MahajanPathPlanner::plotJointTrajectory

    /*
    * Plot path as line marker in RViz
    */
    void plotToolPathLineTrace()
    {
        // Setup Marker Object
        visualization_msgs::msg::Marker line_trace;
        line_trace.header.frame_id = "pumpkin_face";
        line_trace.header.stamp = this->now();
        line_trace.id = 0;
        line_trace.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_trace.action = visualization_msgs::msg::Marker::ADD;
        line_trace.scale.x = 0.004; // Line width
        line_trace.color.r = 1.0;   // Red
        line_trace.color.g = 1.0;   // Green
        line_trace.color.b = 1.0;   // Blue
        line_trace.color.a = 1.0;   // Alpha
        line_trace.pose.orientation.w = 1.0;

        // Populate line trace points
        for (const auto& segment : request_->path)
        {
            for (const auto& pose : segment.poses)
            {
                geometry_msgs::msg::Point p;
                p.x = pose.position.x;
                p.y = pose.position.y;
                p.z = pose.position.z;

                line_trace.points.push_back(p);
            }
        }
        // Publish line trace
        path_line_trace_publisher_->publish(line_trace);
        RCLCPP_INFO(this->get_logger(), "Sent Tool Path Line Trace marker message");

    } // MahajanPathPlanner::plotJointTrajectory

    /*
    * Send Motion Plan
    */
    bool sendMotionPlanRequest()
    {
        // Ensure Server is alive
        while (!this->path_planner_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Client interrupted while waiting for service to appear.");
                return 1;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for motion planning service to appear...");
        }
        
        // Send request to path planner server
        auto result_future = this->path_planner_client_->async_send_request(request_);
        auto node_ptr = shared_from_this();
        if (rclcpp::spin_until_future_complete(node_ptr, result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "service call failed :(");
            path_planner_client_->remove_pending_request(result_future);
            return 1;
        }

        response_ = result_future.get();

        RCLCPP_INFO(
            this->get_logger(), "Success. Recieved motion plan with %zu points.",
            response_->trajectory.points.size());
        
        return 0;
    } //sendMotionPlanRequest

    /*
    Export Toolpath to YAML File
    */
    bool exportToolPathToYAML()
    {
        YAML::Node yaml;
        try {
            yaml = YAML::convert<std::vector<geometry_msgs::msg::PoseArray>>::encode(request_->path);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to convert tool path to YAML: %s", e.what());
            return 1;
        }

        std::ofstream fout(output_toolpath_yaml_filename.c_str());
        fout << yaml;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Exported tool path to YAML file: %s", output_toolpath_yaml_filename.c_str());
        return 0;
    } // MahajanPathPlanner::exportToolPathToYAML

    /*
    Export Trajectory to YAML File
    */
    bool exportTrajectoryToYAML()
    {
        YAML::Node yaml;
        try {
            yaml = YAML::convert<trajectory_msgs::msg::JointTrajectory>::encode(response_->trajectory);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("yaml_utils"), "Failed to convert trajectory message to YAML: %s", e.what());
            return 1;
        }

        std::ofstream fout(output_trajectory_yaml_filename.c_str());
        fout << yaml;
        fout.close();

        RCLCPP_INFO(this->get_logger(), "Exported trajectory to YAML file: %s", output_trajectory_yaml_filename.c_str());
        return 0;
    } // MahajanPathPlanner::exportTrajectoryToYAML


private:
    rclcpp::Client<pumpkin_msgs::srv::PlanMotion>::SharedPtr path_planner_client_;    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_plot_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_line_trace_publisher_;

    // Motion Plan data structure pointers
    pumpkin_msgs::srv::PlanMotion::Request::SharedPtr request_;
    pumpkin_msgs::srv::PlanMotion::Response::SharedPtr response_;

    // Define Path Parameters
    double tooling_length_;
    double moon_x_axis_left_;
    double moon_x_axis_right_;
    double b_1_const_;
    double b_2_const_; 
    double moon_radius_const_;
    double tooling_radius_;
    double pumpkin_radius_;
    double moon_y_offset_;
    double bat_x1_;
    double bat_y1_;
    double bat_x2_;     
    double bat_y2_;
    double bat_y3_;
    double bat_y4_;

    // Output YAML Filename
    std::string output_toolpath_yaml_filename;
    std::string output_trajectory_yaml_filename;
    
}; // MahajanPathPlanner


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // Timer Start
    auto start = std::chrono::high_resolution_clock::now();

    // Initialize Planner Node
    auto node = std::make_shared<MahajanPathPlanner>();
    
    // Sleep to allow node to fully initialize
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Generate Paths (in desired order)
    node->makeTopPath();    
    node->makeBottomPath();

    // Save Tool Path to YAML
    node->exportToolPathToYAML();

    // Plot Full Tool Path
    node->plotToolPathPoses();
    node->plotToolPathLineTrace();

    // Send Motion Plan Request
    //node->sendMotionPlanRequest();
    //node->exportTrajectoryToYAML();
    //node->plotJointTrajectory();

    // Timer End
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    RCLCPP_INFO(rclcpp::get_logger("TIMER"), "Duration %f", duration.count());

    // Cleanup
    rclcpp::shutdown();

} //main

// EOF