#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <yaml-cpp/yaml.h>
#include <unordered_set>
#include <vector>
#include <string>
#include <stdexcept>

// Mesh loading utilities
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <resource_retriever/retriever.hpp>
#include <Eigen/Geometry>
#include <boost/variant/get.hpp>

namespace mpi = moveit::planning_interface;

enum class ShapeKind { CYLINDER, BOX, SPHERE, MESH, COMPOUND, UNKNOWN };

struct PoseSpec {
  geometry_msgs::msg::Pose pose{};
};

struct ComponentSpec {
  ShapeKind kind{ShapeKind::UNKNOWN};
  geometry_msgs::msg::Pose pose{};           // relative to object frame
  std::vector<double> dimensions;            // by kind
  std::string mesh_resource;                 // mesh
  std::vector<double> scale{1.0,1.0,1.0};    // mesh
};

struct ObjectSpec {
  std::string name;
  ShapeKind   kind{ShapeKind::UNKNOWN};
  std::string frame_id{"world"};
  geometry_msgs::msg::Pose base_pose{};      // for compound: optional base pose
  geometry_msgs::msg::Pose pose{};           // for simple objects (legacy)
  std::vector<double> dimensions;            // simple primitives
  std::string mesh_resource;                 // mesh
  std::vector<double> scale{1.0,1.0,1.0};    // mesh
  std::vector<ComponentSpec> components;     // for compound
};

class WorkspaceObjectsNode : public rclcpp::Node {
public:
  explicit WorkspaceObjectsNode(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions())
  : Node("workspace_objects", opts) {
    declare_parameter<std::string>("config_file", "");
    std::string cfg;
    get_parameter("config_file", cfg);
    if (cfg.empty()) {
      RCLCPP_FATAL(get_logger(), "Parameter 'config_file' is required");
      throw std::runtime_error("missing config_file");
    }

    auto specs = load_config(cfg);
    if (specs.empty()) {
      RCLCPP_FATAL(get_logger(), "No valid objects loaded from '%s'", cfg.c_str());
      throw std::runtime_error("empty or invalid config");
    }

    publish_objects(specs);
    RCLCPP_INFO(get_logger(), "Published %zu object(s) to the planning scene.", specs.size());
  }

private:
  mpi::PlanningSceneInterface psi_;

  // ---- utils ----
  static ShapeKind parse_kind(const std::string &s) {
    if (s == "cylinder") return ShapeKind::CYLINDER;
    if (s == "box")      return ShapeKind::BOX;
    if (s == "sphere")   return ShapeKind::SPHERE;
    if (s == "mesh")     return ShapeKind::MESH;
    if (s == "compound") return ShapeKind::COMPOUND;
    return ShapeKind::UNKNOWN;
  }

  static geometry_msgs::msg::Pose parse_pose(const YAML::Node &node) {
    geometry_msgs::msg::Pose p; p.orientation.w = 1.0; // identity
    if (node) {
      if (node["position"]) {
        auto pos = node["position"].as<std::vector<double>>();
        if (pos.size() != 3) throw std::runtime_error("pose.position must be [x,y,z]");
        p.position.x = pos[0]; p.position.y = pos[1]; p.position.z = pos[2];
      }
      if (node["orientation"]) {
        auto q = node["orientation"].as<std::vector<double>>();
        if (q.size() != 4) throw std::runtime_error("pose.orientation must be [w,x,y,z]");
        p.orientation.w = q[0]; p.orientation.x = q[1]; p.orientation.y = q[2]; p.orientation.z = q[3];
      }
    }
    return p;
  }

  static ComponentSpec parse_component(const YAML::Node &n) {
    ComponentSpec c;
    c.kind = parse_kind(n["type"].as<std::string>("unknown"));
    c.pose = parse_pose(n["pose"]);
    if (n["dimensions"])    c.dimensions = n["dimensions"].as<std::vector<double>>();
    if (n["mesh_resource"]) c.mesh_resource = n["mesh_resource"].as<std::string>();
    if (n["scale"])         c.scale = n["scale"].as<std::vector<double>>();
    return c;
  }

  static ObjectSpec parse_object(const YAML::Node &n) {
    ObjectSpec o;
    o.name     = n["name"].as<std::string>();
    o.kind     = parse_kind(n["type"].as<std::string>("unknown"));
    o.frame_id = n["frame_id"].as<std::string>("world");

    // Simple objects
    o.pose = parse_pose(n["pose"]);
    if (n["dimensions"])     o.dimensions = n["dimensions"].as<std::vector<double>>();
    if (n["mesh_resource"])  o.mesh_resource = n["mesh_resource"].as<std::string>();
    if (n["scale"])          o.scale = n["scale"].as<std::vector<double>>();

    // Compound
    if (n["base_pose"])      o.base_pose = parse_pose(n["base_pose"]);
    if (n["components"]) {
      for (const auto &cnode : n["components"]) {
        o.components.push_back(parse_component(cnode));
      }
    }
    return o;
  }

  // Validation
  static bool validate_component(const ComponentSpec &c, std::string &why) {
    switch (c.kind) {
      case ShapeKind::CYLINDER:
        if (c.dimensions.size() != 2) { why = "cylinder needs [height,radius]"; return false; }
        if (c.dimensions[0] <= 0.0 || c.dimensions[1] <= 0.0) { why = "height & radius must be > 0"; return false; }
        return true;
      case ShapeKind::BOX:
        if (c.dimensions.size() != 3) { why = "box needs [x,y,z]"; return false; }
        if (c.dimensions[0] <= 0.0 || c.dimensions[1] <= 0.0 || c.dimensions[2] <= 0.0) { why = "sizes must be > 0"; return false; }
        return true;
      case ShapeKind::SPHERE:
        if (c.dimensions.size() != 1) { why = "sphere needs [radius]"; return false; }
        if (c.dimensions[0] <= 0.0) { why = "radius must be > 0"; return false; }
        return true;
      case ShapeKind::MESH:
        if (c.mesh_resource.empty()) { why = "mesh needs mesh_resource"; return false; }
        if (c.scale.size() != 3) { why = "scale must be [sx,sy,sz]"; return false; }
        if (c.scale[0] <= 0.0 || c.scale[1] <= 0.0 || c.scale[2] <= 0.0) { why = "scale must be > 0"; return false; }
        return true;
      default:
        why = "unknown component type"; return false;
    }
  }

  static bool validate_object(const ObjectSpec &o, std::string &why) {
    if (o.name.empty()) { why = "missing name"; return false; }
    if (o.frame_id.empty()) { why = "missing frame_id"; return false; }

    if (o.kind == ShapeKind::COMPOUND) {
      if (o.components.empty()) { why = "compound needs components"; return false; }
      for (const auto &c : o.components) {
        std::string w; if (!validate_component(c, w)) { why = "component invalid: " + w; return false; }
      }
      return true;
    }

    // simple objects
    switch (o.kind) {
      case ShapeKind::CYLINDER:
        if (o.dimensions.size() != 2) { why = "cylinder needs [height,radius]"; return false; }
        if (o.dimensions[0] <= 0.0 || o.dimensions[1] <= 0.0) { why = "height & radius must be > 0"; return false; }
        return true;
      case ShapeKind::BOX:
        if (o.dimensions.size() != 3) { why = "box needs [x,y,z]"; return false; }
        if (o.dimensions[0] <= 0.0 || o.dimensions[1] <= 0.0 || o.dimensions[2] <= 0.0) { why = "sizes must be > 0"; return false; }
        return true;
      case ShapeKind::SPHERE:
        if (o.dimensions.size() != 1) { why = "sphere needs [radius]"; return false; }
        if (o.dimensions[0] <= 0.0) { why = "radius must be > 0"; return false; }
        return true;
      case ShapeKind::MESH:
        if (o.mesh_resource.empty()) { why = "mesh needs mesh_resource"; return false; }
        if (o.scale.size() != 3) { why = "scale must be [sx,sy,sz]"; return false; }
        if (o.scale[0] <= 0.0 || o.scale[1] <= 0.0 || o.scale[2] <= 0.0) { why = "scale must be > 0"; return false; }
        return true;
      default:
        why = "unknown type"; return false;
    }
  }

  // Builders
  static shape_msgs::msg::SolidPrimitive make_cylinder(const std::vector<double> &d) {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    prim.dimensions.resize(2);
    prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = d[0];
    prim.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = d[1];
    return prim;
  }
  static shape_msgs::msg::SolidPrimitive make_box(const std::vector<double> &d) {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = { d[0], d[1], d[2] };
    return prim;
  }
  static shape_msgs::msg::SolidPrimitive make_sphere(const std::vector<double> &d) {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    prim.dimensions = { d[0] };
    return prim;
  }
  static shape_msgs::msg::Mesh load_mesh_msg(const std::string &resource, const std::vector<double> &scale) {
    shapes::Mesh *mesh = shapes::createMeshFromResource(resource, Eigen::Vector3d(scale[0], scale[1], scale[2]));
    if (!mesh) throw std::runtime_error("failed to load mesh: " + resource);
    shapes::ShapeMsg shape_msg; shapes::constructMsgFromShape(mesh, shape_msg); delete mesh;
    return boost::get<shape_msgs::msg::Mesh>(shape_msg);
  }

  static geometry_msgs::msg::Pose compose(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b) {
    // returns pose c = a ⊕ b (apply b after a)
    Eigen::Quaterniond qa(a.orientation.w, a.orientation.x, a.orientation.y, a.orientation.z);
    Eigen::Quaterniond qb(b.orientation.w, b.orientation.x, b.orientation.y, b.orientation.z);
    Eigen::Quaterniond qc = qa * qb;
    Eigen::Vector3d ta(a.position.x, a.position.y, a.position.z);
    Eigen::Vector3d tb(b.position.x, b.position.y, b.position.z);
    Eigen::Vector3d tc = ta + qa * tb;
    geometry_msgs::msg::Pose c; c.orientation.w = qc.w(); c.orientation.x = qc.x(); c.orientation.y = qc.y(); c.orientation.z = qc.z();
    c.position.x = tc.x(); c.position.y = tc.y(); c.position.z = tc.z();
    return c;
  }

  // YAML load
  std::vector<ObjectSpec> load_config(const std::string &file) {
    std::vector<ObjectSpec> out;
    try {
      YAML::Node root = YAML::LoadFile(file);
      if (!root["objects"]) throw std::runtime_error("YAML must contain 'objects' list");

      std::unordered_set<std::string> seen;
      for (const auto &n : root["objects"]) {
        auto spec = parse_object(n);
        if (seen.count(spec.name)) {
          RCLCPP_WARN(get_logger(), "Duplicate name '%s' — skipping", spec.name.c_str());
          continue;
        }
        std::string why;
        if (!validate_object(spec, why)) {
          RCLCPP_WARN(get_logger(), "Skipping '%s': %s", spec.name.c_str(), why.c_str());
          continue;
        }
        seen.insert(spec.name);
        out.push_back(std::move(spec));
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "YAML error: %s", e.what());
    }
    return out;
  }

  // Publisher
  void publish_objects(const std::vector<ObjectSpec> &specs) {
    std::vector<moveit_msgs::msg::CollisionObject> batch;
    batch.reserve(specs.size());

    for (const auto &o : specs) {
      moveit_msgs::msg::CollisionObject co;
      co.id = o.name;
      co.header.frame_id = o.frame_id;
      co.operation = moveit_msgs::msg::CollisionObject::ADD;

      if (o.kind == ShapeKind::COMPOUND) {
        const auto base = o.base_pose; // identity by default
        for (const auto &c : o.components) {
          switch (c.kind) {
            case ShapeKind::BOX:
              co.primitives.push_back(make_box(c.dimensions));
              co.primitive_poses.push_back(compose(base, c.pose));
              break;
            case ShapeKind::CYLINDER:
              co.primitives.push_back(make_cylinder(c.dimensions));
              co.primitive_poses.push_back(compose(base, c.pose));
              break;
            case ShapeKind::SPHERE:
              co.primitives.push_back(make_sphere(c.dimensions));
              co.primitive_poses.push_back(compose(base, c.pose));
              break;
            case ShapeKind::MESH: {
              try {
                auto mesh_msg = load_mesh_msg(c.mesh_resource, c.scale);
                co.meshes.push_back(mesh_msg);
                co.mesh_poses.push_back(compose(base, c.pose));
              } catch (const std::exception &e) {
                RCLCPP_WARN(get_logger(), "Skipping mesh component in '%s': %s", o.name.c_str(), e.what());
                continue;
              }
              break;
            }
            default:
              RCLCPP_WARN(get_logger(), "Unknown component type in '%s'", o.name.c_str());
              break;
          }
        }
      } else {
        // simple objects
        switch (o.kind) {
          case ShapeKind::CYLINDER:
            co.primitives.push_back(make_cylinder(o.dimensions));
            co.primitive_poses.push_back(o.pose);
            break;
          case ShapeKind::BOX:
            co.primitives.push_back(make_box(o.dimensions));
            co.primitive_poses.push_back(o.pose);
            break;
          case ShapeKind::SPHERE:
            co.primitives.push_back(make_sphere(o.dimensions));
            co.primitive_poses.push_back(o.pose);
            break;
          case ShapeKind::MESH: {
            try {
              auto mesh_msg = load_mesh_msg(o.mesh_resource, o.scale);
              co.meshes.push_back(mesh_msg);
              co.mesh_poses.push_back(o.pose);
            } catch (const std::exception &e) {
              RCLCPP_WARN(get_logger(), "Skipping mesh '%s': %s", o.name.c_str(), e.what());
              continue;
            }
            break;
          }
          default:
            RCLCPP_WARN(get_logger(), "Unsupported type for '%s'", o.name.c_str());
            break;
        }
      }

      // sanity: matching counts
      if (co.primitives.size() != co.primitive_poses.size()) {
        RCLCPP_WARN(get_logger(), "Primitive count mismatch for '%s' (prims=%zu, poses=%zu)",
                    o.name.c_str(), co.primitives.size(), co.primitive_poses.size());
      }
      if (co.meshes.size() != co.mesh_poses.size()) {
        RCLCPP_WARN(get_logger(), "Mesh count mismatch for '%s' (meshes=%zu, poses=%zu)",
                    o.name.c_str(), co.meshes.size(), co.mesh_poses.size());
      }

      batch.push_back(std::move(co));
    }

    if (!batch.empty()) {
      psi_.applyCollisionObjects(batch);
    } else {
      RCLCPP_WARN(get_logger(), "No objects to publish");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WorkspaceObjectsNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}