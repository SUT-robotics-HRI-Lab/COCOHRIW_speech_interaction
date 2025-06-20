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
#include "task_executor/intersection_library.hpp"

#include "leap_gesture_interface/msg/leap_hand.hpp"
#include "leap_gesture_interface/msg/leap_finger.hpp"
#include "leap_gesture_interface/msg/leap_frame.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <manipulator_control/action/execute_task_sequence.hpp>
#include <geometry_msgs/msg/pose.hpp>

using ExecuteTaskSequence = manipulator_control::action::ExecuteTaskSequence;
using TaskStep = manipulator_control::msg::TaskStep;

// Enum for finger types (must match LeapFinger.msg values)
enum FingerType
{
    THUMB = 0,
    INDEX = 1,
    MIDDLE = 2,
    RING = 3,
    PINKY = 4
};

// Enum for joint indices in the joints[] array
enum JointType
{
    METACARPAL = 0,
    PROXIMAL = 1,
    INTERMEDIATE = 2,
    DISTAL = 3,
    TIP = 4
};

using std::placeholders::_1;

using leap_gesture_interface::msg::LeapFinger;
using leap_gesture_interface::msg::LeapFrame;
using leap_gesture_interface::msg::LeapHand;

struct ObjectInfo
{
    std::string name;
    geometry_msgs::msg::Point position;
    double radius;
    double height;

    ObjectInfo() : name(""), radius(0.0), height(0.0) {} // Default constructor

    ObjectInfo(const std::string &n, const geometry_msgs::msg::Point &p, double r, double h)
        : name(n), position(p), radius(r), height(h) {}
};

struct LocationInfo
    {
        std::string name;
        geometry_msgs::msg::Point position;
        double radius;

        LocationInfo(const std::string &n, const geometry_msgs::msg::Point &p, double r)
            : name(n), position(p), radius(r) {}
    };

class TaskExecutorNode : public rclcpp::Node
{
public:
    TaskExecutorNode() : Node("task_executor_node")
    {

        is_picked_ = false;

        leap_sub_ = this->create_subscription<LeapFrame>(
            "/leap_frame", 10, std::bind(&TaskExecutorNode::leap_frame_callback, this, _1));

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

        taskClient_ = rclcpp_action::create_client<ExecuteTaskSequence>(this, "/execute_task_sequence");

        initialize_environment();

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/environment_objects", 10);
        auto markers = create_visualization_markers();
        marker_pub_->publish(markers);

        intersection_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/intersection_marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "✅ Task Executor Node initialized and ready.");
    }

    geometry_msgs::msg::Point to_ros_coords(const geometry_msgs::msg::Point &ultra)
    {
        geometry_msgs::msg::Point pt;
        pt.x = ultra.x;
        pt.y = -ultra.z;
        pt.z = ultra.y;
        return pt;
    }

    void leap_frame_callback(const LeapFrame::SharedPtr msg)
    {
        for (const auto &hand : msg->hands)
        {
            for (const auto &finger : hand.fingers)
            {
                if (finger.finger_type == FingerType::INDEX &&
                    finger.joints.size() > JointType::TIP &&
                    finger.joints.size() > JointType::PROXIMAL)
                {

                    index_tip_point_ros_ = to_ros_coords(finger.joints[JointType::TIP]);
                    index_prox_point_ros_ = to_ros_coords(finger.joints[JointType::PROXIMAL]);

                    geometry_msgs::msg::PointStamped tip_stamped, prox_stamped;
                    tip_stamped.header = msg->header;
                    prox_stamped.header = msg->header;
                    tip_stamped.header.frame_id = "leap_hands";
                    prox_stamped.header.frame_id = "leap_hands";
                    tip_stamped.point = index_tip_point_ros_;
                    prox_stamped.point = index_prox_point_ros_;

                    try
                    {
                        tf_buffer_->transform(tip_stamped, index_tip_point_, "world");
                        tf_buffer_->transform(prox_stamped, index_prox_point_, "world");
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "TF transform failed (leap frame): %s", ex.what());
                    }
                }
            }
        }

        // Check intersections with all known objects
        for (const auto &info : environment_objects_)
        {
            geometry_msgs::msg::Point line_point1, line_point2, base, axis;
            line_point1.x = index_prox_point_.point.x;
            line_point1.y = index_prox_point_.point.y;
            line_point1.z = index_prox_point_.point.z;

            line_point2.x = index_tip_point_.point.x;
            line_point2.y = index_tip_point_.point.y;
            line_point2.z = index_tip_point_.point.z;

            base = info.position;
            axis.x = 0.0;
            axis.y = 0.0;
            axis.z = 1.0;

            auto intersections = intersectLineFiniteCylinder(
                IntersectionLibrary::Vector3{line_point1.x, line_point1.y, line_point1.z},
                IntersectionLibrary::Vector3{line_point2.x, line_point2.y, line_point2.z},
                IntersectionLibrary::Vector3{base.x, base.y, base.z},
                IntersectionLibrary::Vector3{axis.x, axis.y, axis.z},
                info.height, info.radius);
            //RCLCPP_INFO(this->get_logger(), "🔍 Found %zu intersections with [%s].",
            //            intersections.size(), info.name.c_str());


            for (const auto &pt : intersections)
            {
                visualization_msgs::msg::Marker green_marker;
                green_marker.header.frame_id = "world";
                green_marker.header.stamp = now();
                green_marker.ns = "intersection";
                green_marker.id = 1000 + static_cast<int>(&info - &environment_objects_[0]);
                green_marker.type = visualization_msgs::msg::Marker::SPHERE;
                green_marker.action = visualization_msgs::msg::Marker::ADD;
                green_marker.pose.position.x = pt.x;
                green_marker.pose.position.y = pt.y;
                green_marker.pose.position.z = pt.z;
                green_marker.scale.x = 0.05;
                green_marker.scale.y = 0.05;
                green_marker.scale.z = 0.05;
                green_marker.color.r = 0.0f;
                green_marker.color.g = 1.0f;
                green_marker.color.b = 0.0f;
                green_marker.color.a = 1.0f;
                intersection_marker_pub_->publish(green_marker);
            }
        }
    }

    geometry_msgs::msg::Pose setManipulatorPose(double x, double y, double z, double rx = 0.0, double ry = 1.0, double rz = 0.0, double rw = 0.0)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = rx;
        pose.orientation.y = ry;
        pose.orientation.z = rz;
        pose.orientation.w = rw;
        return pose;
    }

    void pickObjectTask(const ObjectInfo &object)
    {
        TaskStep prePickPose;
        TaskStep pickPose;
        TaskStep gripperCommand;
        TaskStep postPickPose;
        auto goal_msg = ExecuteTaskSequence::Goal();

        prePickPose.type = TaskStep::MOVE_POSE;
        prePickPose.frame_id = "world";
        prePickPose.is_cartesian = false;
        prePickPose.pose = setManipulatorPose(object.position.x, object.position.y, (object.height + 0.2), 0.0, 1.0, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Moving to pre-pick pose: (%f, %f, %f)", object.position.x, object.position.y, (object.position.z + 0.2));
        goal_msg.steps.push_back(prePickPose);

        pickPose.type = TaskStep::MOVE_POSE;
        pickPose.frame_id = "world";
        pickPose.is_cartesian = true;
        pickPose.pose = setManipulatorPose(object.position.x, object.position.y, (object.height + 0.16), 0.0, 1.0, 0.0, 0.0);
        goal_msg.steps.push_back(pickPose);
        
        gripperCommand.type = TaskStep::GRIPPER_COMMAND;
        gripperCommand.close_gripper = true;
        gripperCommand.speed = 5;
        gripperCommand.force = 10;
        goal_msg.steps.push_back(gripperCommand);

        postPickPose.type = TaskStep::MOVE_POSE;
        postPickPose.frame_id = "world";
        postPickPose.is_cartesian = false;
        postPickPose.pose = prePickPose.pose; // Move back to pre-pick pose
        goal_msg.steps.push_back(postPickPose);

        auto send_goal_options = rclcpp_action::Client<ExecuteTaskSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::SharedPtr,
                   const std::shared_ptr<const ExecuteTaskSequence::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Feedback: Step %ld - %s", feedback->current_step_index,
                            feedback->step_description.c_str());
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    is_picked_ = true;
                    RCLCPP_INFO(this->get_logger(), "Task sequence succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Task sequence failed or aborted: %s", result.result->message.c_str());
                }
                rclcpp::shutdown();
            };

        
        picked_object_ = object; // Store the picked object info
        taskClient_->async_send_goal(goal_msg, send_goal_options);
    }

    void place()
    {
        TaskStep prePlacePose;
        TaskStep placePose;
        TaskStep gripperCommand;
        auto goal_msg = ExecuteTaskSequence::Goal();

        prePlacePose.type = TaskStep::MOVE_POSE;
        prePlacePose.frame_id = "world";
        prePlacePose.is_cartesian = false;
        prePlacePose.pose = setManipulatorPose(latest_target_point_.x, latest_target_point_.y, picked_object_.height + 0.04, 0.0, 1.0, 0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Moving to place location: (%f, %f, %f)", latest_target_point_.x, latest_target_point_.y,  (picked_object_.height + 0.04));
        goal_msg.steps.push_back(prePlacePose);

        placePose.type = TaskStep::MOVE_POSE;
        placePose.frame_id = "world";
        placePose.is_cartesian = true;
        placePose.pose = setManipulatorPose(latest_target_point_.x, latest_target_point_.y, picked_object_.height + 0.005, 0.0, 1.0, 0.0, 0.0);
        goal_msg.steps.push_back(placePose);

        gripperCommand.type = TaskStep::GRIPPER_COMMAND;
        gripperCommand.close_gripper = false; // Open gripper to release object
        gripperCommand.speed = 5;
        gripperCommand.force = 10;
        goal_msg.steps.push_back(gripperCommand);

        // Back to pre-place pose
        TaskStep postPlacePose;
        postPlacePose.type = TaskStep::MOVE_POSE;
        postPlacePose.frame_id = "world";
        postPlacePose.is_cartesian = false;
        postPlacePose.pose = prePlacePose.pose; // Move back to pre-place pose
        goal_msg.steps.push_back(postPlacePose);

        auto send_goal_options = rclcpp_action::Client<ExecuteTaskSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::SharedPtr,
                   const std::shared_ptr<const ExecuteTaskSequence::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Feedback: Step %ld - %s", feedback->current_step_index,
                            feedback->step_description.c_str());
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(this->get_logger(), "Task sequence succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Task sequence failed or aborted: %s", result.result->message.c_str());
                }
                rclcpp::shutdown();
            };

        is_picked_ = false; // Reset picked state after placing
        taskClient_->async_send_goal(goal_msg, send_goal_options);
    }

    void moveToLocationTask(const LocationInfo &location)
    {
        TaskStep movePose;
        auto goal_msg = ExecuteTaskSequence::Goal();

        movePose.type = TaskStep::MOVE_POSE;
        movePose.frame_id = "world";
        movePose.is_cartesian = false;
        if (is_picked_)
        {
            // If an object is picked, move to the location above the object
            movePose.pose = setManipulatorPose(location.position.x, location.position.y, picked_object_.height + 0.04, 0.0, 1.0, 0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Moving to location above picked object: (%f, %f, %f)", location.position.x, location.position.y, (picked_object_.height + 0.04));
        }
        else
        {
            // If no object is picked, move to the location above the target point
            movePose.pose = setManipulatorPose(location.position.x, location.position.y, location.position.z + 0.2, 0.0, 1.0, 0.0, 0.0);
            RCLCPP_INFO(this->get_logger(), "Moving to location: (%f, %f, %f)", location.position.x, location.position.y, (location.position.z + 0.2));
        }
        goal_msg.steps.push_back(movePose);

        auto send_goal_options = rclcpp_action::Client<ExecuteTaskSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::SharedPtr,
                   const std::shared_ptr<const ExecuteTaskSequence::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Feedback: Step %ld - %s", feedback->current_step_index,
                            feedback->step_description.c_str());
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(this->get_logger(), "Task sequence succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Task sequence failed or aborted: %s", result.result->message.c_str());
                }
                rclcpp::shutdown();
            };

        taskClient_->async_send_goal(goal_msg, send_goal_options);
    }

    void moveToLocationTask(const geometry_msgs::msg::Point &point)
    {
        TaskStep movePose;
        auto goal_msg = ExecuteTaskSequence::Goal();

        movePose.type = TaskStep::MOVE_POSE;
        movePose.frame_id = "world";
        movePose.is_cartesian = false;
        movePose.pose = setManipulatorPose(point.x, point.y, point.z + 0.2, 0.0, 1.0, 0.0, 0.0);
        goal_msg.steps.push_back(movePose);

        auto send_goal_options = rclcpp_action::Client<ExecuteTaskSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::SharedPtr,
                   const std::shared_ptr<const ExecuteTaskSequence::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Feedback: Step %ld - %s", feedback->current_step_index,
                            feedback->step_description.c_str());
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(this->get_logger(), "Task sequence succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Task sequence failed or aborted: %s", result.result->message.c_str());
                }
                rclcpp::shutdown();
            };

        taskClient_->async_send_goal(goal_msg, send_goal_options);
    }



private:
    rclcpp::Subscription<LeapFrame>::SharedPtr leap_sub_;
    rclcpp_action::Client<ExecuteTaskSequence>::SharedPtr taskClient_;
    
    geometry_msgs::msg::Point index_tip_point_ros_, index_prox_point_ros_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intersection_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    geometry_msgs::msg::PointStamped index_tip_point_;
    geometry_msgs::msg::PointStamped index_prox_point_;
    std::vector<geometry_msgs::msg::Point> intersection_points_;

    bool is_picked_;
    ObjectInfo picked_object_;

    visualization_msgs::msg::MarkerArray create_visualization_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        for (const auto &loc : environment_locations_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = now();
            marker.ns = "locations";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = loc.position;
            marker.pose.position.z += 0.01;
            marker.scale.x = 2 * loc.radius;
            marker.scale.y = 2 * loc.radius;
            marker.scale.z = 0.0005;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            marker_array.markers.push_back(marker);
        }
        for (const auto &obj : environment_objects_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world";
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
        p1.x = 0.0;
        p1.y = 0.0;
        p1.z = 0.0;
        environment_locations_.push_back(LocationInfo("marker", p1, 0.1));
        geometry_msgs::msg::Point p2;
        p2.x = 0.4;
        p2.y = 0.2;
        p2.z = 0.0;
        environment_locations_.push_back(LocationInfo("disk", p2, 0.1));
        geometry_msgs::msg::Point p3;
        p3.x = 0.7;
        p3.y = -0.1;
        p3.z = 0.0;
        environment_locations_.push_back(LocationInfo("disk", p3, 0.1));
        geometry_msgs::msg::Point p4;
        p4.x = 0.3;
        p4.y = -0.3;
        p4.z = 0.0;
        environment_locations_.push_back(LocationInfo("yellow marker", p4, 0.1));
        geometry_msgs::msg::Point o1;
        o1.x = 1.05;
        o1.y = 0.39;
        o1.z = 0.0;
        environment_objects_.push_back(ObjectInfo("vial", o1, 0.05, 0.23));
        geometry_msgs::msg::Point o2;
        o2.x = 1.203;
        o2.y = 0.3028;
        o2.z = 0.0;
        environment_objects_.push_back(ObjectInfo("bottle", o2, 0.05, 0.23));
        geometry_msgs::msg::Point o3;
        o3.x = 0.876;
        o3.y = 0.4198;
        o3.z = 0.0;
        environment_objects_.push_back(ObjectInfo("flask", o3, 0.05, 0.23));
        geometry_msgs::msg::Point o4;
        o4.x = 0.677;
        o4.y = 0.4177;
        o4.z = 0.0;
        environment_objects_.push_back(ObjectInfo("cylinder", o4, 0.04, 0.23));

        RCLCPP_INFO(this->get_logger(), "🏗️ Environment initialized with multiple simulated objects.");
    }

    void dialog_state_callback(const dialog_interfaces::msg::DialogState::SharedPtr msg)
    {
        current_state_ = msg->current_state;
        RCLCPP_INFO(this->get_logger(), "🧠 Dialog state updated: %s", current_state_.c_str());
    }

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) // Extended to publish marker
    {
        geometry_msgs::msg::PointStamped stamped = *msg;
        stamped.header.frame_id = "leap_hands";
        geometry_msgs::msg::PointStamped transformed;
        try
        {
            tf_buffer_->transform(stamped, transformed, "world", tf2::durationFromSec(0.2));
            latest_target_point_ = transformed.point;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "⚠️ TF transform failed: %s", ex.what());
            return;
        }
        //RCLCPP_INFO(this->get_logger(), "🎯 Received point: x=%.2f, y=%.2f, z=%.2f",
        //              msg->point.x, msg->point.y, msg->point.z);
        //RCLCPP_INFO(this->get_logger(), "📍 Transformed point: x=%.2f, y=%.2f, z=%.2f",
        //              latest_target_point_.x, latest_target_point_.y, latest_target_point_.z);

        // Publish intersection marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/world";
        marker.header.stamp = now();
        marker.ns = "intersection";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = transformed.point;
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
                {
                    //Compute intersection with the object
                    geometry_msgs::msg::Point line_point1, line_point2, base, axis;
                    line_point1.x = index_prox_point_.point.x;
                    line_point1.y = index_prox_point_.point.y;
                    line_point1.z = index_prox_point_.point.z;
                    line_point2.x = index_tip_point_.point.x;
                    line_point2.y = index_tip_point_.point.y;
                    line_point2.z = index_tip_point_.point.z;
                    base = info.position;
                    axis.x = 0.0;
                    axis.y = 0.0;
                    axis.z = 1.0;
                    auto intersections = intersectLineFiniteCylinder(
                        IntersectionLibrary::Vector3{line_point1.x, line_point1.y, line_point1.z},
                        IntersectionLibrary::Vector3{line_point2.x, line_point2.y, line_point2.z},
                        IntersectionLibrary::Vector3{base.x, base.y, base.z},
                        IntersectionLibrary::Vector3{axis.x, axis.y, axis.z},
                        info.height, info.radius);
                    // RCLCPP_INFO(this->get_logger(), "🔍 Found %zu intersections with [%s].",
                    //             intersections.size(), info.name.c_str());
                    if (intersections.empty())
                    {
                        //RCLCPP_WARN(this->get_logger(), "❌ No intersection found with object [%s]", info.name.c_str());
                        continue;
                        matching_objects.push_back(info);
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "✅ Intersection found with object [%s]", info.name.c_str());
                        pickObjectTask(info);
                        publish_dialog_state("TASK_COMPLETED");
                        return;
                    }
                }
            }

            RCLCPP_WARN(this->get_logger(), "❓ No intersection detected for [%s] in %s task", object_name.c_str(), task.task_type.c_str());

            if(matching_objects.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "❌ No matching objects found for task: %s", task.task_type.c_str());
                publish_dialog_state("TASK_FAILED");
                return;
            }
            else if (matching_objects.size() == 1)
            {
                RCLCPP_INFO(this->get_logger(), "✅ Found one matching object: %s", matching_objects[0].name.c_str());
                pickObjectTask(matching_objects[0]);
                publish_dialog_state("TASK_COMPLETED");
                return;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "❗ Multiple matching objects found for task: %s", task.task_type.c_str());
                publish_dialog_state("TASK_FAILED");
                return;
            }
            

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

                moveToLocationTask(latest_target_point_);
                publish_dialog_state("TASK_COMPLETED");
                RCLCPP_INFO(this->get_logger(), "📦 Task done. Moving to next task...");
                return;
            }

            // Match exact location name in the environment
            for (const auto &loc : environment_locations_)
            {
                if (lowercase_target == loc.name)
                {
                    RCLCPP_INFO(this->get_logger(), "🗺️ Navigating to location [%s] at (%.2f, %.2f, %.2f)",
                                loc.name.c_str(), loc.position.x, loc.position.y, loc.position.z);

                    moveToLocationTask(loc);
                    latest_target_point_ = loc.position; // Update latest target point
                    publish_dialog_state("TASK_COMPLETED");
                    RCLCPP_INFO(this->get_logger(), "📦 Task done. Moving to next task...");
                    return;
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "🔧 Default handling for task type: %s", task.task_type.c_str());
            publish_dialog_state("TASK_COMPLETED");
            RCLCPP_INFO(this->get_logger(), "📦 Task done. Moving to next task...");
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
