#include "rclcpp/rclcpp.hpp"
#include "task_msgs/msg/task_array.hpp"
#include "task_msgs/msg/task.hpp"
#include "dialog_interfaces/msg/dialog_state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "leap_gesture_interface/srv/intersect_finite_cylinder.hpp"
#include <unordered_map>
#include <string>
#include <vector>
#include <algorithm>
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

struct ObjectInfo
{
    std::string name;
    geometry_msgs::msg::Point position;
    double radius;
    double height;

    ObjectInfo(const std::string &n, const geometry_msgs::msg::Point &p, double r, double h)
        : name(n), position(p), radius(r), height(h) {}
};

class TaskExecutorNode : public rclcpp::Node
{
public:
    TaskExecutorNode() : Node("task_executor_node")
    {
        task_array_sub_ = this->create_subscription<task_msgs::msg::TaskArray>(
            "/dialog_manager/task_array_complete", 10,
            std::bind(&TaskExecutorNode::task_array_callback, this, _1));

        dialog_state_sub_ = this->create_subscription<dialog_interfaces::msg::DialogState>(
            "/dialog_state", 10,
            std::bind(&TaskExecutorNode::dialog_state_callback, this, _1));

        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/intersection_point/right/transformed", 10,
            std::bind(&TaskExecutorNode::point_callback, this, _1));

        dialog_state_pub_ = this->create_publisher<dialog_interfaces::msg::DialogState>("/dialog_state", 10);

        intersection_client_ = this->create_client<leap_gesture_interface::srv::IntersectFiniteCylinder>("/intersect_finite_cylinder");

        initialize_environment();

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);
        auto markers = create_visualization_markers();
        marker_pub_->publish(markers);

        intersection_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/intersection_marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "✅ Task Executor Node initialized and ready.");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intersection_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    visualization_msgs::msg::MarkerArray create_visualization_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto &loc : environment_locations_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "/world";
            marker.header.stamp = now();
            marker.ns = "locations";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = loc.position;
            marker.pose.position.z += 0.01;
            marker.scale.x = 2 * loc.radius;
            marker.scale.y = 2 * loc.radius;
            marker.scale.z = 0.02;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            marker_array.markers.push_back(marker);
        }
        for (const auto &obj : environment_objects_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "/world";
            marker.header.stamp = now();
            marker.ns = "objects";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = obj.position;
            marker.scale.x = 2 * obj.radius;
            marker.scale.y = 2 * obj.radius;
            marker.scale.z = obj.height;
            marker.pose.position.z += obj.height / 2.0;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 0.7f;
            marker_array.markers.push_back(marker);
        }
        return marker_array;
    }
    struct LocationInfo
    {
        std::string name;
        geometry_msgs::msg::Point position;
        double radius;

        LocationInfo(const std::string &n, const geometry_msgs::msg::Point &p, double r)
            : name(n), position(p), radius(r) {}
    };

    std::vector<LocationInfo> environment_locations_;
    rclcpp::Subscription<task_msgs::msg::TaskArray>::SharedPtr task_array_sub_;
    rclcpp::Subscription<dialog_interfaces::msg::DialogState>::SharedPtr dialog_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Publisher<dialog_interfaces::msg::DialogState>::SharedPtr dialog_state_pub_;
    rclcpp::Client<leap_gesture_interface::srv::IntersectFiniteCylinder>::SharedPtr intersection_client_;

    std::vector<task_msgs::msg::Task> task_queue_;
    std::string current_state_ = "IDLE";
    geometry_msgs::msg::Point latest_target_point_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<ObjectInfo> environment_objects_;

    void initialize_environment()
    {
        // Initialize known locations
        geometry_msgs::msg::Point p1;
        p1.x = 0.6;
        p1.y = 0.0;
        p1.z = 0.0;
        environment_locations_.push_back(LocationInfo("marker", p1, 0.1));
        geometry_msgs::msg::Point p2;
        p2.x = 0.4;
        p2.y = 0.2;
        p2.z = 0.0;
        environment_locations_.push_back(LocationInfo("disk", p2, 0.15));
        geometry_msgs::msg::Point p3;
        p3.x = 0.7;
        p3.y = -0.1;
        p3.z = 0.0;
        environment_locations_.push_back(LocationInfo("disk", p3, 0.15));
        geometry_msgs::msg::Point p4;
        p4.x = 0.3;
        p4.y = -0.3;
        p4.z = 0.0;
        environment_locations_.push_back(LocationInfo("yellow marker", p4, 0.1));
        geometry_msgs::msg::Point o1;
        o1.x = 0.3;
        o1.y = 0.2;
        o1.z = 0.0;
        environment_objects_.push_back(ObjectInfo("can", o1, 0.03, 0.12));
        geometry_msgs::msg::Point o2;
        o2.x = 0.5;
        o2.y = 0.25;
        o2.z = 0.0;
        environment_objects_.push_back(ObjectInfo("can", o2, 0.03, 0.12));
        geometry_msgs::msg::Point o3;
        o3.x = 0.5;
        o3.y = 0.1;
        o3.z = 0.0;
        environment_objects_.push_back(ObjectInfo("bottle", o3, 0.035, 0.25));
        geometry_msgs::msg::Point o4;
        o4.x = 0.4;
        o4.y = -0.2;
        o4.z = 0.0;
        environment_objects_.push_back(ObjectInfo("cup", o4, 0.04, 0.1));

        RCLCPP_INFO(this->get_logger(), "🏗️ Environment initialized with multiple simulated objects.");
    }

    void dialog_state_callback(const dialog_interfaces::msg::DialogState::SharedPtr msg)
    {
        current_state_ = msg->current_state;
        RCLCPP_INFO(this->get_logger(), "🧠 Dialog state updated: %s", current_state_.c_str());
    }

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) // Extended to publish marker
    {
        geometry_msgs::msg::PointStamped transformed;
        try
        {
            tf_buffer_->transform(*msg, transformed, "world", tf2::durationFromSec(0.2));
            latest_target_point_ = transformed.point;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ TF transform failed: %s", ex.what());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "🎯 Received point: x=%.2f, y=%.2f, z=%.2f",
                    msg->point.x, msg->point.y, msg->point.z);
        RCLCPP_INFO(this->get_logger(), "📍 Transformed point: x=%.2f, y=%.2f, z=%.2f",
                    latest_target_point_.x, latest_target_point_.y, latest_target_point_.z);

        // Publish intersection marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = now();
        marker.ns = "intersection";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = msg->point;
        marker.scale.x = 0.04;
        marker.scale.y = 0.04;
        marker.scale.z = 0.04;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        intersection_marker_pub_->publish(marker);
    }

    void task_array_callback(const task_msgs::msg::TaskArray::SharedPtr msg)
    {
        task_queue_ = msg->tasks;
        RCLCPP_INFO(this->get_logger(), "📦 Received %zu tasks to execute.", task_queue_.size());
        execute_next_task();
    }

    void execute_next_task()
    {
        if (task_queue_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "✅ All tasks completed.");
            publish_dialog_state("AWAITING_TASK");
            return;
        }

        auto task = task_queue_.front();
        task_queue_.erase(task_queue_.begin());

        RCLCPP_INFO(this->get_logger(), "🚧 Executing task: %s", task.task_type.c_str());

        if (task.task_type == "Pick" || task.task_type == "Place" || task.task_type == "Inspect")
        {
            const auto &object_name = task.object_of_interest.empty() ? "unknown" : task.object_of_interest[0];
            RCLCPP_INFO(this->get_logger(), "🤖 Picking object: [%s]", object_name.c_str());
        
          std::vector<ObjectInfo> matching_objects;
          for (const auto &info : environment_objects_)
            {
                if (info.name == object_name)
                    matching_objects.push_back(info);
            }
        
            if (matching_objects.size() == 1)
            {
              const auto &info = matching_objects[0];
              RCLCPP_INFO(this->get_logger(), "✅ Only one [%s] found. Selecting automatically at (%.2f, %.2f, %.2f).",
                        info.name.c_str(), info.position.x, info.position.y, info.position.z);
              publish_dialog_state("AWAITING_TASK");
              return;
            }

            for (const auto &info : environment_objects_)
            {
                if (info.name == object_name)
                {
                    auto request = std::make_shared<leap_gesture_interface::srv::IntersectFiniteCylinder::Request>();
                    request->line_point1.x = latest_target_point_.x;
                    request->line_point1.y = latest_target_point_.y;
                    request->line_point1.z = latest_target_point_.z + 0.5;
                    request->line_point2.x = latest_target_point_.x;
                    request->line_point2.y = latest_target_point_.y;
                    request->line_point2.z = latest_target_point_.z - 0.5;
                    request->cylinder_base = info.position;
                    request->cylinder_tip = info.position;
                    request->cylinder_tip.z += info.height;
                    request->radius = info.radius;
                    request->height = info.height;

                    while (!intersection_client_->wait_for_service(std::chrono::seconds(1)))
                    {
                        RCLCPP_WARN(this->get_logger(), "Waiting for intersection service...");
                    }

                    auto result_future = intersection_client_->async_send_request(request);
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
                        rclcpp::FutureReturnCode::SUCCESS)
                    {
                        auto response = result_future.get();
                        if (!response->intersections.empty())
                        {
                            RCLCPP_INFO(this->get_logger(), "📍 Intersection detected with [%s] — task complete.", object_name.c_str());
                            publish_dialog_state("AWAITING_TASK");
                            return;
                        }
                    }
                }
            }
            RCLCPP_WARN(this->get_logger(), "❓ No intersection detected for [%s] in %s task", object_name.c_str(), task.task_type.c_str());
        }
        else if (task.task_type == "Navigate")
        {
            std::string target = task.target_location.empty() ? "" : task.target_location[0];
            std::string lowercase_target = target;
            std::transform(lowercase_target.begin(), lowercase_target.end(), lowercase_target.begin(), ::tolower);

            // Handle vague references to pointing location
            if (lowercase_target.find("there") != std::string::npos ||
                lowercase_target.find("that") != std::string::npos ||
                lowercase_target.find("pointed location") != std::string::npos ||
                lowercase_target.find("pointing") != std::string::npos)
            {
                RCLCPP_INFO(this->get_logger(), "🧭 Navigating to pointed location: x=%.2f, y=%.2f, z=%.2f",
                            latest_target_point_.x, latest_target_point_.y, latest_target_point_.z);
                publish_dialog_state("AWAITING_TASK");
                return;
            }

            // Match exact location name in the environment
            for (const auto &loc : environment_locations_)
            {
                if (lowercase_target == loc.name)
                {
                    RCLCPP_INFO(this->get_logger(), "🗺️ Navigating to location [%s] at (%.2f, %.2f, %.2f)",
                                loc.name.c_str(), loc.position.x, loc.position.y, loc.position.z);
                    publish_dialog_state("AWAITING_TASK");
                    return;
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "🔧 Default handling for task type: %s", task.task_type.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "✅ Task done. Moving to next task...");
        execute_next_task();
    }

    void publish_dialog_state(const std::string &new_state)
    {
        dialog_interfaces::msg::DialogState msg;
        msg.previous_state = current_state_;
        msg.current_state = new_state;
        dialog_state_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🔄 Published DialogState: %s", new_state.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskExecutorNode>());
    rclcpp::shutdown();
    return 0;
}
