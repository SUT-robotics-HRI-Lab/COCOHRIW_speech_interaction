#include <memory>
#include <vector>
#include <string>
#include <limits>
#include <optional>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "leap_gesture_interface/msg/leap_frame.hpp"
#include "leap_gesture_interface/msg/leap_finger.hpp"

#include "pointing_interaction/msg/pointing_intersection.hpp"

// Intersection utilities (your header)
#include "pointing_interaction/intersection_library.hpp"

// MoveIt (live planning scene)
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <Eigen/Geometry>

using leap_gesture_interface::msg::LeapFinger;
using leap_gesture_interface::msg::LeapFrame;
using std::placeholders::_1;

enum FingerType
{
  THUMB = 0,
  INDEX = 1,
  MIDDLE = 2,
  RING = 3,
  PINKY = 4
};

// Local cache structs
struct CachedCylinder
{
  std::string name;
  geometry_msgs::msg::Point base;
  IntersectionLibrary::Vector3 axis;
  double radius{0.0};
  double height{0.0};
  geometry_msgs::msg::Quaternion orientation;
};

struct CachedSphere
{
  std::string name;
  geometry_msgs::msg::Point center;
  double radius{0.0};
  geometry_msgs::msg::Quaternion orientation;
};

struct CachedBox
{
  std::string name;
  geometry_msgs::msg::Point center;
  double width{0.0}, length{0.0}, height{0.0};
  IntersectionLibrary::Vector3 x_axis, y_axis, z_axis;
  geometry_msgs::msg::Quaternion orientation;
};

// Convert Ultraleap coords -> ROS-style coords (as in your working code)
static geometry_msgs::msg::Point to_ros_coords(const geometry_msgs::msg::Point &ultra)
{
  geometry_msgs::msg::Point pt;
  pt.x = ultra.x;
  pt.y = -ultra.z;
  pt.z = ultra.y;
  return pt;
}

class PointingInteractionNode : public rclcpp::Node
{
public:
  PointingInteractionNode()
      : Node("pointing_interaction_node",
             rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Parameters
    target_frame_ = this->get_parameter("target_frame").as_string();
    publish_marker_ = this->get_parameter("publish_marker").as_bool();
    // Add refresh rate parameter (in Hz, default 0.5 Hz = every 2 seconds)
    refresh_rate_ = this->get_parameter("refresh_rate").as_double();

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriptions
    // Connects to Leap Motion via leap_gesture_interface
    leap_sub_ = this->create_subscription<LeapFrame>(
        "/leap_frame", 10, std::bind(&PointingInteractionNode::onLeapFrame, this, _1));

    // Publications
    out_pub_ = this->create_publisher<pointing_interaction::msg::PointingIntersection>("/pointing_intersection", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pointing_interaction/marker", 10);

    RCLCPP_INFO(get_logger(), "✅ pointing_interaction_node started. Connects to Leap Motion & MoveIt Planning Scene.");

    // Refresh collision objects initially
    refreshCollisionObjects();

    // Timer for refreshing objects, using refresh_rate parameter
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / refresh_rate_),
        std::bind(&PointingInteractionNode::refreshCollisionObjects, this));
  }

private:
  // Parameters
  std::string target_frame_;
  bool publish_marker_;
  double refresh_rate_{0.5}; // Hz

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Sub/Pub
  rclcpp::Subscription<LeapFrame>::SharedPtr leap_sub_;
  rclcpp::Publisher<pointing_interaction::msg::PointingIntersection>::SharedPtr out_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Timer for refreshing objects
  rclcpp::TimerBase::SharedPtr timer_;

  // Collision object caches
  std::vector<CachedCylinder> cylinders_;
  std::vector<CachedSphere> spheres_;
  std::vector<CachedBox> boxes_;

  // MoveIt planning scene interface
  moveit::planning_interface::PlanningSceneInterface psi_;

  // ========== Core callbacks ==========

  void onLeapFrame(const LeapFrame::SharedPtr msg)
  {
    geometry_msgs::msg::Point leap_prox{}, leap_tip{};
    if (!extractIndexProxTip(*msg, leap_prox, leap_tip))
      return;

    const geometry_msgs::msg::Point ros_prox = to_ros_coords(leap_prox);
    const geometry_msgs::msg::Point ros_tip = to_ros_coords(leap_tip);

    geometry_msgs::msg::PointStamped prox_in, tip_in, prox_out, tip_out;
    prox_in.header = msg->header;
    tip_in.header = msg->header;
    if (prox_in.header.frame_id.empty())
      prox_in.header.frame_id = "leap_hands";
    if (tip_in.header.frame_id.empty())
      tip_in.header.frame_id = "leap_hands";
    prox_in.point = ros_prox;
    tip_in.point = ros_tip;

    try
    {
      tf_buffer_->transform(prox_in, prox_out, target_frame_, tf2::durationFromSec(0.2));
      tf_buffer_->transform(tip_in, tip_out, target_frame_, tf2::durationFromSec(0.2));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(), "Leap TF failed: %s", ex.what());
      return;
    }

    const IntersectionLibrary::Vector3 L1{prox_out.point.x, prox_out.point.y, prox_out.point.z};
    const IntersectionLibrary::Vector3 L2{tip_out.point.x, tip_out.point.y, tip_out.point.z};

    bool hit_any = false;
    geometry_msgs::msg::Point best_pt{};
    std::string best_name;
    double best_dist = std::numeric_limits<double>::infinity();

    // Cylinders
    for (const auto &cyl : cylinders_)
    {
      const IntersectionLibrary::Vector3 base{cyl.base.x, cyl.base.y, cyl.base.z};
      const auto pts = intersectLineFiniteCylinder(L1, L2, base, cyl.axis, cyl.height, cyl.radius);
      for (const auto &p : pts)
      {
        double d = std::hypot(p.x - L1.x, p.y - L1.y, p.z - L1.z);
        if (d < best_dist)
        {
          best_dist = d;
          best_pt.x = p.x;
          best_pt.y = p.y;
          best_pt.z = p.z;
          best_name = cyl.name;
          hit_any = true;
        }
      }
    }

    // Spheres
    for (const auto &sph : spheres_)
    {
      const IntersectionLibrary::Vector3 center{sph.center.x, sph.center.y, sph.center.z};
      const auto pts = IntersectionLibrary::intersectLineSphere(L1, L2, center, sph.radius);
      for (const auto &p : pts)
      {
        double d = std::hypot(p.x - L1.x, p.y - L1.y, p.z - L1.z);
        if (d < best_dist)
        {
          best_dist = d;
          best_pt.x = p.x;
          best_pt.y = p.y;
          best_pt.z = p.z;
          best_name = sph.name;
          hit_any = true;
        }
      }
    }

    // Boxes
    for (const auto &box : boxes_)
    {
      IntersectionLibrary::Vector3 center{box.center.x, box.center.y, box.center.z};
      auto pts = IntersectionLibrary::intersectRayCuboidOriented(
          L1, L2, center, box.width, box.length, box.height,
          box.x_axis, box.y_axis, box.z_axis);
      for (const auto &p : pts)
      {
        double d = std::hypot(p.x - L1.x, p.y - L1.y, p.z - L1.z);
        if (d < best_dist)
        {
          best_dist = d;
          best_pt.x = p.x;
          best_pt.y = p.y;
          best_pt.z = p.z;
          best_name = box.name;
          hit_any = true;
        }
      }
    }

    if (hit_any)
    {
      publishResult(best_pt, true, best_name);
      RCLCPP_INFO(get_logger(), "Intersection with '%s' at (%.2f, %.2f, %.2f)",
                  best_name.c_str(), best_pt.x, best_pt.y, best_pt.z);
      return;
    }

    // Fallback: plane intersection Z=0
    if (auto plane_opt = intersectRayWithZ0(L1, L2))
    {
      publishResult(*plane_opt, false, "");
      RCLCPP_INFO(get_logger(), "Fallback Z=0 plane at (%.2f, %.2f, %.2f)",
                  plane_opt->x, plane_opt->y, plane_opt->z);
    }
  }

  // ========== Helpers ==========

  bool extractIndexProxTip(const LeapFrame &frame,
                           geometry_msgs::msg::Point &prox,
                           geometry_msgs::msg::Point &tip)
  {
    for (const auto &hand : frame.hands)
    {
      for (const auto &finger : hand.fingers)
      {
        if (finger.finger_type != INDEX)
          continue;
        if (finger.joints.empty())
          continue;
        // TIP = last joint; PROX = 2nd joint if available, else first
        tip = finger.joints.back();
        prox = (finger.joints.size() > 1) ? finger.joints[1] : finger.joints.front();
        return true;
      }
    }
    return false;
  }

  std::optional<geometry_msgs::msg::Point>
  intersectRayWithZ0(const IntersectionLibrary::Vector3 &L1,
                     const IntersectionLibrary::Vector3 &L2)
  {
    // Plane in target_frame: point (0,0,0), normal (0,0,1)
    const IntersectionLibrary::Vector3 pp{0.0, 0.0, 0.0};
    const IntersectionLibrary::Vector3 nn{0.0, 0.0, 1.0};
    auto [ok, res] = IntersectionLibrary::intersectLinePlane(L1, L2, pp, nn);
    if (!ok)
      return std::nullopt;
    geometry_msgs::msg::Point p;
    p.x = res.x;
    p.y = res.y;
    p.z = res.z;
    return p;
  }

  void publishResult(const geometry_msgs::msg::Point &p, bool on_object, const std::string &name)
  {
    pointing_interaction::msg::PointingIntersection out;
    out.point.header.stamp = this->now();
    out.point.header.frame_id = target_frame_;
    out.point.point = p;
    out.on_object = on_object;
    out.object_name = on_object ? name : "";
    out_pub_->publish(out);

    if (publish_marker_)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = target_frame_;
      m.header.stamp = now();
      m.ns = "pointing_interaction";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = p;
      m.scale.x = m.scale.y = m.scale.z = 0.04;
      if (on_object)
      {
        m.color.g = 1.0f;
        m.color.r = 0.0f;
      }
      else
      {
        m.color.r = 1.0f;
        m.color.g = 0.0f;
      }
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      marker_pub_->publish(m);
    }
  }

  void refreshCollisionObjects()
{
  auto objects = psi_.getObjects();
  RCLCPP_INFO(this->get_logger(), "Retrieved %zu collision objects.", objects.size());

  cylinders_.clear();
  spheres_.clear();
  boxes_.clear();

  // ---- Helpers -------------------------------------------------------------

  // Pose composition: C = A ∘ B
  auto composePoses = [](const geometry_msgs::msg::Pose& A,
                         const geometry_msgs::msg::Pose& B) -> geometry_msgs::msg::Pose
  {
    Eigen::Quaterniond qA(A.orientation.w, A.orientation.x, A.orientation.y, A.orientation.z);
    Eigen::Vector3d    pA(A.position.x, A.position.y, A.position.z);

    Eigen::Quaterniond qB(B.orientation.w, B.orientation.x, B.orientation.y, B.orientation.z);
    Eigen::Vector3d    pB(B.position.x, B.position.y, B.position.z);

    Eigen::Quaterniond qC = qA * qB;
    Eigen::Vector3d    pC = pA + qA * pB;

    geometry_msgs::msg::Pose C;
    C.position.x = pC.x(); C.position.y = pC.y(); C.position.z = pC.z();
    C.orientation.w = qC.w(); C.orientation.x = qC.x(); C.orientation.y = qC.y(); C.orientation.z = qC.z();
    return C;
  };

  // TF Pose from src frame -> target_frame_
  auto tf_pose = [&](const geometry_msgs::msg::Pose &pose,
                     const std::string &src_frame) -> std::optional<geometry_msgs::msg::Pose>
  {
    geometry_msgs::msg::PoseStamped in, out;
    in.header.frame_id = src_frame.empty() ? target_frame_ : src_frame;
    in.header.stamp = this->now();
    in.pose = pose;
    try {
      if (in.header.frame_id != target_frame_) {
        tf_buffer_->transform(in, out, target_frame_, tf2::durationFromSec(0.2));
      } else {
        out = in;
      }
      return out.pose;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF pose %s->%s failed: %s",
                  in.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return std::nullopt;
    }
  };

  // quaternion -> unit axis triad
  auto quat_to_axes = [](const geometry_msgs::msg::Quaternion &q,
                         IntersectionLibrary::Vector3 &x_axis,
                         IntersectionLibrary::Vector3 &y_axis,
                         IntersectionLibrary::Vector3 &z_axis)
  {
    Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
    Eigen::Matrix3d R = eq.normalized().toRotationMatrix();
    x_axis = {R(0,0), R(1,0), R(2,0)};
    y_axis = {R(0,1), R(1,1), R(2,1)};
    z_axis = {R(0,2), R(1,2), R(2,2)};
    auto norm = [](IntersectionLibrary::Vector3& v){
      double n = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
      if (n > 1e-12) { v.x/=n; v.y/=n; v.z/=n; }
    };
    norm(x_axis); norm(y_axis); norm(z_axis);
  };

  // ---- Extract & cache -----------------------------------------------------

  for (const auto &[id, obj] : objects)
  {
    const std::string src_frame = obj.header.frame_id;
    const geometry_msgs::msg::Pose object_pose = obj.pose; // may be identity

    for (size_t i = 0; i < obj.primitives.size(); ++i)
    {
      if (i >= obj.primitive_poses.size()) continue;

      const auto &prim      = obj.primitives[i];
      const auto &prim_pose = obj.primitive_poses[i];

      // 1) Compose object-level pose with primitive pose (still in src_frame)
      geometry_msgs::msg::Pose composed_src = composePoses(object_pose, prim_pose);

      // 2) Transform composed pose into target_frame_
      auto pose_tf_opt = tf_pose(composed_src, src_frame);
      if (!pose_tf_opt) continue;
      const auto &pose_tf = *pose_tf_opt;

      // 3) Derive unit axes from transformed orientation
      IntersectionLibrary::Vector3 x_axis, y_axis, z_axis;
      quat_to_axes(pose_tf.orientation, x_axis, y_axis, z_axis);

      // 4) Cache objects with transformed pose & axes
      if (prim.type == shape_msgs::msg::SolidPrimitive::BOX) {
        if (prim.dimensions.size() < 3) continue;
        CachedBox box;
        box.name    = (obj.primitives.size() == 1 ? id : id + "#" + std::to_string(i));
        box.center  = pose_tf.position;
        box.width   = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
        box.length  = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
        box.height  = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
        box.x_axis  = x_axis;  box.y_axis = y_axis;  box.z_axis = z_axis;
        box.orientation = pose_tf.orientation;
        boxes_.push_back(std::move(box));

        RCLCPP_INFO(this->get_logger(),
          "Cached box '%s': center=(%.2f,%.2f,%.2f) size=(%.2f,%.2f,%.2f) axes=(%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f) (%.2f,%.2f,%.2f)",
          box.name.c_str(),
          box.center.x, box.center.y, box.center.z,
          box.width, box.length, box.height,
          box.x_axis.x, box.x_axis.y, box.x_axis.z,
          box.y_axis.x, box.y_axis.y, box.y_axis.z,
          box.z_axis.x, box.z_axis.y, box.z_axis.z);
      }
      else if (prim.type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
        if (prim.dimensions.size() < 2) continue;
        CachedCylinder cyl;
        cyl.name    = (obj.primitives.size() == 1 ? id : id + "#" + std::to_string(i));
        cyl.radius  = prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
        cyl.height  = prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
        cyl.axis    = z_axis;  // cylinder axis = +Z of orientation
        // base = center - 0.5 * height * axis  (all in target_frame_)
        cyl.base.x  = pose_tf.position.x - 0.5 * cyl.height * cyl.axis.x;
        cyl.base.y  = pose_tf.position.y - 0.5 * cyl.height * cyl.axis.y;
        cyl.base.z  = pose_tf.position.z - 0.5 * cyl.height * cyl.axis.z;
        cyl.orientation = pose_tf.orientation;
        cylinders_.push_back(std::move(cyl));

        RCLCPP_INFO(this->get_logger(),
          "Cached cylinder '%s': center=(%.2f,%.2f,%.2f) base=(%.2f,%.2f,%.2f) axis=(%.2f,%.2f,%.2f) r=%.3f h=%.3f",
          cyl.name.c_str(),
          pose_tf.position.x, pose_tf.position.y, pose_tf.position.z,
          cyl.base.x, cyl.base.y, cyl.base.z,
          cyl.axis.x, cyl.axis.y, cyl.axis.z,
          cyl.radius, cyl.height);
      }
      else if (prim.type == shape_msgs::msg::SolidPrimitive::SPHERE) {
        if (prim.dimensions.size() < 1) continue;
        CachedSphere sph;
        sph.name    = (obj.primitives.size() == 1 ? id : id + "#" + std::to_string(i));
        sph.center  = pose_tf.position;
        sph.radius  = prim.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
        sph.orientation = pose_tf.orientation; // usually unused for intersection
        spheres_.push_back(std::move(sph));
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Cached: %zu boxes, %zu cylinders, %zu spheres.",
              boxes_.size(), cylinders_.size(), spheres_.size());
}


};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointingInteractionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
