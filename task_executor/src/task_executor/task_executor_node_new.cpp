// new_task_executor_node.cpp
// A clean, modular Task Executor node that consumes dialog tasks and pointing_intersection input.
// NOTE: This file is a design-first draft. You asked not to build yet; headers/types are referenced as-is.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Task + dialog interfaces
#include "task_msgs/msg/task_array.hpp"
#include "task_msgs/msg/task.hpp"
#include "dialog_interfaces/msg/dialog_state.hpp"

// Pointing intersection input
#include "pointing_interaction/msg/pointing_intersection.hpp"  // geometry_msgs/PointStamped point, bool on_object, string object_name, shape_msgs/SolidPrimitive primitive
#include <shape_msgs/msg/solid_primitive.hpp>

// Manipulator action interface
#include <manipulator_control/action/execute_task_sequence.hpp>
#include <manipulator_control/msg/task_step.hpp>
#include <geometry_msgs/msg/pose.hpp>

// TF2 for frame normalization (pointing point -> world)
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <algorithm>
#include <string>
#include <vector>
#include <optional>

using std::placeholders::_1;
using ExecuteTaskSequence = manipulator_control::action::ExecuteTaskSequence;
using TaskStep            = manipulator_control::msg::TaskStep;

// Represents the latest pointing info from the pointing_interaction package
struct PointingSnapshot {
  geometry_msgs::msg::PointStamped point_world;   // normalized to world
  bool on_object = false;                         // whether ray hit a known object
  std::string object_name;                        // name of hit object (if any)
  shape_msgs::msg::SolidPrimitive primitive;      // shape, e.g., CYLINDER
  rclcpp::Time stamp;                             // when it was updated
  bool valid = false;                             // whether we have valid data
};

class TaskExecutorNode : public rclcpp::Node {
public:
  TaskExecutorNode()
  : Node("task_executor_node"),
    world_frame_(declare_parameter<std::string>("world_frame", "world")),
    pointing_fresh_ms_(declare_parameter<int>("pointing_fresh_ms", 500))
  {
    // --- Create subscribers ---
    task_array_sub_ = create_subscription<task_msgs::msg::TaskArray>(
      "/dialog_manager/task_array_complete", rclcpp::QoS(10),
      std::bind(&TaskExecutorNode::on_task_array, this, _1));

    dialog_state_sub_ = create_subscription<dialog_interfaces::msg::DialogState>(
      "/dialog_state", rclcpp::QoS(20),
      std::bind(&TaskExecutorNode::on_dialog_state, this, _1));

    pointing_sub_ = create_subscription<pointing_interaction::msg::PointingIntersection>(
      "/pointing_intersection", rclcpp::QoS(20),
      std::bind(&TaskExecutorNode::on_pointing_intersection, this, _1));

    // --- Publishers ---
    dialog_state_pub_ = create_publisher<dialog_interfaces::msg::DialogState>("/dialog_state", 10);

    // --- Action client ---
    task_client_ = rclcpp_action::create_client<ExecuteTaskSequence>(this, "/execute_task_sequence");

    // --- TF2 ---
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(), "TaskExecutorNode (clean) started. world_frame='%s', freshness=%dms",
                world_frame_.c_str(), pointing_fresh_ms_);
  }

private:
  // =================== Subscriptions / Publishers / Action ===================
  rclcpp::Subscription<task_msgs::msg::TaskArray>::SharedPtr task_array_sub_;
  rclcpp::Subscription<dialog_interfaces::msg::DialogState>::SharedPtr dialog_state_sub_;
  rclcpp::Subscription<pointing_interaction::msg::PointingIntersection>::SharedPtr pointing_sub_;

  rclcpp::Publisher<dialog_interfaces::msg::DialogState>::SharedPtr dialog_state_pub_;

  rclcpp_action::Client<ExecuteTaskSequence>::SharedPtr task_client_;

  // =================== TF & params ===================
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string world_frame_;
  int pointing_fresh_ms_;

  // =================== State ===================
  std::vector<task_msgs::msg::Task> task_queue_;
  std::string dialog_state_ = "IDLE";
  PointingSnapshot last_pointing_;

  // =================== Callbacks ===================
  void on_task_array(const task_msgs::msg::TaskArray::SharedPtr msg) {
    task_queue_ = msg->tasks; // replace queue (or push_back for append semantics)
    RCLCPP_INFO(get_logger(), "Received %zu tasks. Beginning execution...", task_queue_.size());
    execute_next_task();
  }

  void on_dialog_state(const dialog_interfaces::msg::DialogState::SharedPtr msg) {
    dialog_state_ = msg->current_state;
    RCLCPP_DEBUG(get_logger(), "Dialog state -> %s", dialog_state_.c_str());
  }

  void on_pointing_intersection(const pointing_interaction::msg::PointingIntersection::SharedPtr msg) {
    // Normalize to world
    geometry_msgs::msg::PointStamped world_pt;
    try {
      if (msg->point.header.frame_id != world_frame_) {
        tf_buffer_->transform(msg->point, world_pt, world_frame_, tf2::durationFromSec(0.15));
      } else {
        world_pt = msg->point;
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF to %s failed for pointing_intersection: %s", world_frame_.c_str(), ex.what());
      return;
    }

    last_pointing_.point_world = world_pt;
    last_pointing_.on_object   = msg->on_object;
    last_pointing_.object_name = msg->object_name;
    last_pointing_.primitive   = msg->primitive;
    last_pointing_.stamp       = now();
    last_pointing_.valid       = true;

    RCLCPP_DEBUG(get_logger(), "Pointing at (%.3f, %.3f, %.3f) | on_object=%d | name='%s' | type=%u",
                 world_pt.point.x, world_pt.point.y, world_pt.point.z,
                 msg->on_object, msg->object_name.c_str(), msg->primitive.type);
  }

  // =================== Task execution ===================
  void execute_next_task() {
    if (task_queue_.empty()) {
      publish_dialog_state("AWAITING_TASK");
      RCLCPP_INFO(get_logger(), "All tasks completed.");
      return;
    }

    const auto task = task_queue_.front();
    task_queue_.erase(task_queue_.begin());

    RCLCPP_INFO(get_logger(), "Executing task: %s", task.task_type.c_str());

    if (task.task_type == "Pick") {
      handle_pick(task);
      return;
    }
    if (task.task_type == "Place") {
      handle_place(task);
      return;
    }
    if (task.task_type == "Navigate") {
      handle_navigate(task);
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown task type: %s (marking complete)", task.task_type.c_str());
    publish_dialog_state("TASK_COMPLETED");
    execute_next_task();
  }

  // =================== Handlers ===================
  void handle_pick(const task_msgs::msg::Task &task) {
    const std::string requested = task.object_of_interest.empty() ? std::string("") : task.object_of_interest[0];

    // 1) Freshness gate
    if (!pointing_is_fresh()) {
      RCLCPP_WARN(get_logger(), "Pick: pointing data is stale or missing");
      publish_dialog_state("TASK_FAILED");
      execute_next_task();
      return;
    }

    // 2) Must be pointing at an object
    if (!last_pointing_.on_object) {
      RCLCPP_WARN(get_logger(), "Pick: pointing not on an object");
      publish_dialog_state("TASK_FAILED");
      execute_next_task();
      return;
    }

    // 3) If a specific object was requested, enforce name match
    if (!requested.empty() && requested != last_pointing_.object_name) {
      RCLCPP_WARN(get_logger(), "Pick: pointed object '%s' does not match requested '%s'", last_pointing_.object_name.c_str(), requested.c_str());
      publish_dialog_state("TASK_FAILED");
      execute_next_task();
      return;
    }

    // 4) Build the pick sequence using the primitive's top surface as reference.
    //    Pre-pick:  top_z + 0.20
    //    Pick:      top_z + 0.16
    const auto p = last_pointing_.point_world.point;
    const double top_z = p.z + top_offset_from_primitive(last_pointing_.primitive);

    std::vector<TaskStep> steps;
    steps.push_back(make_move_pose(p.x, p.y, top_z + 0.20, false)); // pre-pick over primitive
    steps.push_back(make_move_pose(p.x, p.y, top_z + 0.16,  true)); // pick over primitive
    steps.push_back(make_gripper(true,  5, 10));                     // close gripper
    steps.push_back(make_move_pose(p.x, p.y, top_z + 0.20, false)); // retreat

    send_task_sequence(steps, [this](bool ok){
      publish_dialog_state(ok ? "TASK_COMPLETED" : "TASK_FAILED");
      execute_next_task();
    });
  }

  void handle_place(const task_msgs::msg::Task & /*task*/) {
    if (!pointing_is_fresh()) {
      RCLCPP_WARN(get_logger(), "Place: pointing data is stale or missing");
      publish_dialog_state("TASK_FAILED");
      execute_next_task();
      return;
    }

    auto p = last_pointing_.point_world.point;

    // If placing on top of a CYLINDER, lift Z by half height if primitive assumed center-origin
    if (last_pointing_.primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER &&
        last_pointing_.primitive.dimensions.size() > shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT)
    {
      const double H = last_pointing_.primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
      p.z += 0.5 * H;  // Adjust to top surface if needed by your convention
    }

    std::vector<TaskStep> steps;
    steps.push_back(make_move_pose(p.x, p.y, p.z + 0.20, false)); // pre-place
    steps.push_back(make_move_pose(p.x, p.y, p.z + 0.02,  true)); // descend
    steps.push_back(make_gripper(false, 5, 10));                   // open
    steps.push_back(make_move_pose(p.x, p.y, p.z + 0.20, false)); // retreat

    send_task_sequence(steps, [this](bool ok){
      publish_dialog_state(ok ? "TASK_COMPLETED" : "TASK_FAILED");
      execute_next_task();
    });
  }

  void handle_navigate(const task_msgs::msg::Task &task) {
    const std::string target = (task.target_location.empty() ? std::string("") : task.target_location[0]);
    const std::string key    = to_lower_copy(target);

    if (is_pointing_keyword(key)) {
      if (!pointing_is_fresh()) {
        RCLCPP_WARN(get_logger(), "Navigate: pointing keyword used but pointing is stale");
        publish_dialog_state("TASK_FAILED");
        execute_next_task();
        return;
      }

      const auto p = last_pointing_.point_world.point;
      std::vector<TaskStep> steps;
      steps.push_back(make_move_pose(p.x, p.y, p.z + 0.20, false));

      send_task_sequence(steps, [this](bool ok){
        publish_dialog_state(ok ? "TASK_COMPLETED" : "TASK_FAILED");
        execute_next_task();
      });
      return;
    }

    // TODO: extend with named waypoints/locations if desired
    RCLCPP_WARN(get_logger(), "Navigate: unsupported target '%s' (expected pointing keywords)", target.c_str());
    publish_dialog_state("TASK_FAILED");
    execute_next_task();
  }

  // =================== Helpers ===================
  // Compute how much to lift Z from the pointed contact point to reach the TOP surface of the primitive.
  // This assumes the primitive center is at its geometric center in Z (common for SolidPrimitive),
  // so top surface = point.z + half height (CYLINDER/CONE/BOX) or + radius (SPHERE).
  double top_offset_from_primitive(const shape_msgs::msg::SolidPrimitive &prim) const {
    using SP = shape_msgs::msg::SolidPrimitive;
    double off = 0.0;
    switch (prim.type) {
      case SP::CYLINDER:
        if (prim.dimensions.size() > SP::CYLINDER_HEIGHT)
          off = 0.5 * prim.dimensions[SP::CYLINDER_HEIGHT];
        break;
      case SP::BOX:
        if (prim.dimensions.size() > SP::BOX_Z)
          off = 0.5 * prim.dimensions[SP::BOX_Z];
        break;
      case SP::SPHERE:
        if (prim.dimensions.size() > SP::SPHERE_RADIUS)
          off = prim.dimensions[SP::SPHERE_RADIUS];
        break;
      case SP::CONE:
        if (prim.dimensions.size() > SP::CONE_HEIGHT)
          off = 0.5 * prim.dimensions[SP::CONE_HEIGHT];
        break;
      default:
        break;
    }
    return off;
  }
  bool pointing_is_fresh() const {
    if (!last_pointing_.valid) return false;
    const auto age = (now() - last_pointing_.stamp).nanoseconds() / 1e6; // ms
    return age <= static_cast<double>(pointing_fresh_ms_);
  }

  static bool is_pointing_keyword(const std::string &s) {
    return s.find("there") != std::string::npos ||
           s.find("that") != std::string::npos ||
           s.find("pointed location") != std::string::npos ||
           s.find("pointing") != std::string::npos;
  }

  static std::string to_lower_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
  }

  geometry_msgs::msg::Pose make_pose(double x, double y, double z,
                                     double qx = 0.0, double qy = 1.0, double qz = 0.0, double qw = 0.0) const {
    geometry_msgs::msg::Pose p;
    p.position.x = x; p.position.y = y; p.position.z = z;
    p.orientation.x = qx; p.orientation.y = qy; p.orientation.z = qz; p.orientation.w = qw;
    return p;
  }

  TaskStep make_move_pose(double x, double y, double z, bool cartesian) const {
    TaskStep s; s.type = TaskStep::MOVE_POSE; s.frame_id = world_frame_; s.is_cartesian = cartesian; s.pose = make_pose(x,y,z);
    return s;
  }

  TaskStep make_gripper(bool close, int speed, int force) const {
    TaskStep s; s.type = TaskStep::GRIPPER_COMMAND; s.close_gripper = close; s.speed = speed; s.force = force;
    return s;
  }

  void send_task_sequence(const std::vector<TaskStep> &steps, std::function<void(bool)> done_cb) {
    if (!task_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "ExecuteTaskSequence action server not available");
      done_cb(false);
      return;
    }

    ExecuteTaskSequence::Goal goal; goal.steps = steps;

    auto options = rclcpp_action::Client<ExecuteTaskSequence>::SendGoalOptions();
    options.goal_response_callback = [this](auto handle){
      if (!handle) RCLCPP_ERROR(get_logger(), "Goal rejected by server");
      else         RCLCPP_INFO (get_logger(), "Goal accepted by server");
    };
    options.feedback_callback = [this](auto, const std::shared_ptr<const ExecuteTaskSequence::Feedback> feedback){
      RCLCPP_DEBUG(get_logger(), "Feedback: step %ld - %s", feedback->current_step_index, feedback->step_description.c_str());
    };
    options.result_callback = [this, done_cb](const rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::WrappedResult &res){
      const bool ok = (res.code == rclcpp_action::ResultCode::SUCCEEDED);
      if (ok) RCLCPP_INFO (get_logger(), "Task sequence succeeded: %s", res.result->message.c_str());
      else    RCLCPP_ERROR(get_logger(), "Task sequence failed: %s", res.result ? res.result->message.c_str() : "<no result>");
      done_cb(ok);
    };

    task_client_->async_send_goal(goal, options);
  }

  void publish_dialog_state(const std::string &state) {
    dialog_interfaces::msg::DialogState msg; msg.previous_state = dialog_state_; msg.current_state = state;
    dialog_state_pub_->publish(msg);
    dialog_state_ = state;
    RCLCPP_INFO(get_logger(), "DialogState -> %s", state.c_str());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskExecutorNode>());
  rclcpp::shutdown();
  return 0;
}
