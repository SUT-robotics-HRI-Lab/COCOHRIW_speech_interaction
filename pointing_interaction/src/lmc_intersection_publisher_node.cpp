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

using std::placeholders::_1;
using leap_gesture_interface::msg::LeapFrame;
using leap_gesture_interface::msg::LeapFinger;

enum FingerType { THUMB=0, INDEX=1, MIDDLE=2, RING=3, PINKY=4 };

// Local cache type (kept only in this .cpp)
struct CachedCylinder {
  std::string name;
  geometry_msgs::msg::Point base;
  IntersectionLibrary::Vector3 axis; // <-- Add this
  double radius{0.0};
  double height{0.0};
};

struct CachedSphere {
  std::string name;
  geometry_msgs::msg::Point center;
  double radius{0.0};
};

struct CachedBox {
  std::string name;
  geometry_msgs::msg::Point center;
  double width{0.0}, length{0.0}, height{0.0};
  // Local axes in target_frame_
  IntersectionLibrary::Vector3 x_axis, y_axis, z_axis;
};

// Convert Ultraleap coords -> ROS-style coords (as in your working code)
static geometry_msgs::msg::Point to_ros_coords(const geometry_msgs::msg::Point &ultra) {
  geometry_msgs::msg::Point pt;
  pt.x = ultra.x;
  pt.y = -ultra.z;
  pt.z = ultra.y;
  return pt;
}

class PointingInteractionNode : public rclcpp::Node {
public:
  PointingInteractionNode()
  : Node("pointing_interaction_node",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
    // Parameters
    target_frame_   = this->get_parameter("target_frame").as_string();
    publish_marker_ = this->get_parameter("publish_marker").as_bool();

    // TF
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
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

    // Or, to refresh every 2 seconds:
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&PointingInteractionNode::refreshCollisionObjects, this)
    );
  }

private:
  // Parameters
  std::string target_frame_;
  bool publish_marker_;

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

  void onLeapFrame(const LeapFrame::SharedPtr msg) {
    // Extract INDEX finger PROX (near-hand) and TIP from Leap coords
    geometry_msgs::msg::Point leap_prox{}, leap_tip{};
    if (!extractIndexProxTip(*msg, leap_prox, leap_tip)) {
      // No ray possible → nothing to do
      return;
    }

    // Convert Ultraleap coords → ROS-style coords BEFORE TF
    const geometry_msgs::msg::Point ros_prox = to_ros_coords(leap_prox);
    const geometry_msgs::msg::Point ros_tip  = to_ros_coords(leap_tip);

    // Transform to target_frame_
    geometry_msgs::msg::PointStamped prox_in, tip_in, prox_out, tip_out;
    prox_in.header = msg->header;
    tip_in.header  = msg->header;
    if (prox_in.header.frame_id.empty()) prox_in.header.frame_id = "leap_hands";
    if (tip_in.header.frame_id.empty())  tip_in.header.frame_id  = "leap_hands";
    prox_in.point  = ros_prox;
    tip_in.point   = ros_tip;

    try {
      tf_buffer_->transform(prox_in, prox_out, target_frame_, tf2::durationFromSec(0.2));
      tf_buffer_->transform(tip_in,  tip_out,  target_frame_, tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Leap TF failed: %s", ex.what());
      return;
    }

    // Build ray L1->L2 in target_frame
    const IntersectionLibrary::Vector3 L1{prox_out.point.x, prox_out.point.y, prox_out.point.z};
    const IntersectionLibrary::Vector3 L2{tip_out.point.x,  tip_out.point.y,  tip_out.point.z};

    bool hit_any = false;
    geometry_msgs::msg::Point best_pt{};
    std::string best_name;
    double best_dist = std::numeric_limits<double>::infinity();

    // Cylinders
    for (const auto &cyl : cylinders_) {
      const IntersectionLibrary::Vector3 base{cyl.base.x, cyl.base.y, cyl.base.z};
      const IntersectionLibrary::Vector3 axis = cyl.axis; // <-- Use cached axis
      const auto pts = intersectLineFiniteCylinder(L1, L2, base, axis, cyl.height, cyl.radius);

      for (const auto &p : pts) {
        const double dx = p.x - L1.x, dy = p.y - L1.y, dz = p.z - L1.z;
        const double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d < best_dist) {
          best_dist = d;
          best_pt.x = p.x; best_pt.y = p.y; best_pt.z = p.z;
          best_name = cyl.name;
          hit_any = true;
        }
      }
    }

    // Spheres
    for (const auto &sph : spheres_) {
      const IntersectionLibrary::Vector3 center{sph.center.x, sph.center.y, sph.center.z};
      const auto pts = IntersectionLibrary::intersectLineSphere(L1, L2, center, sph.radius);

      for (const auto &p : pts) {
        const double dx = p.x - L1.x, dy = p.y - L1.y, dz = p.z - L1.z;
        const double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d < best_dist) {
          best_dist = d;
          best_pt.x = p.x; best_pt.y = p.y; best_pt.z = p.z;
          best_name = sph.name;
          hit_any = true;
        }
      }
    }

    // Boxes
    for (const auto &box : boxes_) {
      IntersectionLibrary::Vector3 center{box.center.x, box.center.y, box.center.z};
      auto pts = IntersectionLibrary::intersectLineCuboidOriented(
        L1, L2, center, box.width, box.length, box.height,
        box.x_axis, box.y_axis, box.z_axis);

      for (const auto &p : pts) {
        double dx = p.x - L1.x, dy = p.y - L1.y, dz = p.z - L1.z;
        double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d < best_dist) {
          best_dist = d;
          best_pt.x = p.x; best_pt.y = p.y; best_pt.z = p.z;
          best_name = box.name;
          hit_any = true;
        }
      }
    }

    if (hit_any) {
      publishResult(best_pt, true, best_name);
      RCLCPP_INFO(get_logger(), "Intersection found with object '%s' at (%.2f, %.2f, %.2f)",
    best_name.c_str(), best_pt.x, best_pt.y, best_pt.z);
      return;
    }

    // 2) Fallback: plane intersection in target_frame (Z=0)
    if (auto plane_opt = intersectRayWithZ0(L1, L2)) {
      RCLCPP_INFO(get_logger(), "No object intersection. Fallback to Z=0 plane at (%.2f, %.2f, %.2f)",
        plane_opt->x, plane_opt->y, plane_opt->z);
      publishResult(*plane_opt, false, "");
    }
  }

  // ========== Helpers ==========

  bool extractIndexProxTip(const LeapFrame& frame,
                           geometry_msgs::msg::Point& prox,
                           geometry_msgs::msg::Point& tip) {
    for (const auto &hand : frame.hands) {
      for (const auto &finger : hand.fingers) {
        if (finger.finger_type != INDEX) continue;
        if (finger.joints.empty()) continue;
        // TIP = last joint; PROX = 2nd joint if available, else first
        tip = finger.joints.back();
        prox = (finger.joints.size() > 1) ? finger.joints[1] : finger.joints.front();
        return true;
      }
    }
    return false;
  }

  std::optional<geometry_msgs::msg::Point>
  intersectRayWithZ0(const IntersectionLibrary::Vector3& L1,
                     const IntersectionLibrary::Vector3& L2) {
    // Plane in target_frame: point (0,0,0), normal (0,0,1)
    const IntersectionLibrary::Vector3 pp{0.0, 0.0, 0.0};
    const IntersectionLibrary::Vector3 nn{0.0, 0.0, 1.0};
    auto [ok, res] = IntersectionLibrary::intersectLinePlane(L1, L2, pp, nn);
    if (!ok) return std::nullopt;
    geometry_msgs::msg::Point p; p.x = res.x; p.y = res.y; p.z = res.z;
    return p;
  }

  void publishResult(const geometry_msgs::msg::Point &p, bool on_object, const std::string &name) {
    pointing_interaction::msg::PointingIntersection out;
    out.point.header.stamp = this->now();
    out.point.header.frame_id = target_frame_;
    out.point.point = p;
    out.on_object = on_object;
    out.object_name = on_object ? name : "";
    out_pub_->publish(out);

    if (publish_marker_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = target_frame_;
      m.header.stamp = now();
      m.ns = "pointing_interaction";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position = p;
      m.scale.x = m.scale.y = m.scale.z = 0.04;
      if (on_object) { m.color.g = 1.0f; m.color.r = 0.0f; }
      else { m.color.r = 1.0f; m.color.g = 0.0f; }
      m.color.b = 0.0f;
      m.color.a = 1.0f;
      marker_pub_->publish(m);
    }
  }

  void refreshCollisionObjects() {
    auto objects = psi_.getObjects();
    RCLCPP_INFO(this->get_logger(), "Retrieved %zu collision objects.", objects.size());

    cylinders_.clear();
    spheres_.clear();
    boxes_.clear();

    for (const auto& [id, obj] : objects) {
        for (size_t i = 0; i < obj.primitives.size(); ++i) {
            const auto& prim = obj.primitives[i];
            const auto& pose = obj.primitive_poses[i];

            if (prim.type == shape_msgs::msg::SolidPrimitive::BOX) {
                CachedBox box;
                box.name = id;
                box.center = pose.position;
                box.width  = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X];
                box.length = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y];
                box.height = prim.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z];
                // For axis-aligned boxes, axes are (1,0,0), (0,1,0), (0,0,1)
                box.x_axis = IntersectionLibrary::Vector3(1,0,0);
                box.y_axis = IntersectionLibrary::Vector3(0,1,0);
                box.z_axis = IntersectionLibrary::Vector3(0,0,1);
                boxes_.push_back(box);
            } else if (prim.type == shape_msgs::msg::SolidPrimitive::CYLINDER) {
                CachedCylinder cyl;
                cyl.name = id;
                cyl.base = pose.position;
                cyl.radius = prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
                cyl.height = prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
                // For axis-aligned, axis is (0,0,1)
                cyl.axis = IntersectionLibrary::Vector3(0,0,1);
                cylinders_.push_back(cyl);
            } else if (prim.type == shape_msgs::msg::SolidPrimitive::SPHERE) {
                CachedSphere sph;
                sph.name = id;
                sph.center = pose.position;
                sph.radius = prim.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
                spheres_.push_back(sph);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Cached: %zu boxes, %zu cylinders, %zu spheres.",
        boxes_.size(), cylinders_.size(), spheres_.size());
}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointingInteractionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
