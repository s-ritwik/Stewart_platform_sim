#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

namespace gazebo
{

class SDFJointController : public ModelPlugin
{
public:
  SDFJointController() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    if (!model) {
      gzerr << "[SDFJointController] Invalid model pointer, plugin not loaded\n";
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "[SDFJointController] Failed to get gazebo_ros node, plugin not loaded\n";
      return;
    }

    model_ = model;
    std::string top_link_name = "platform_link";
    std::string base_link_name = "base_link";
    
    top_link_ = model_->GetLink(top_link_name);
    if (!top_link_) {
      gzerr << "Top platform link '" << top_link_name
            << "' not found in model '" << model_->GetName() << "'\n";
    }
    base_link_ = model_->GetLink(base_link_name);
    if (!base_link_) {
      gzerr << "Base platform link '" << base_link_name
            << "' not found in model '" << model_->GetName() << "'\n";
    }

    // Collect only prismatic "piston" joints and sort them by name
    auto all_joints = model_->GetJoints();
    for (const auto & joint : all_joints) {
      if (joint->GetMsgType() == gazebo::msgs::Joint::PRISMATIC &&
          joint->GetName().find("piston") != std::string::npos)
      {
        actuated_joints_.push_back(joint);
      }
    }

    std::sort(
      actuated_joints_.begin(), actuated_joints_.end(),
      [](const physics::JointPtr & a, const physics::JointPtr & b) {
        return a->GetName() < b->GetName();
      });

    if (actuated_joints_.empty()) {
      gzerr << "[SDFJointController] No prismatic 'piston' joints found.\n";
      return;
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "[SDFJointController] Model '%s': %zu actuated joints",
      model_->GetName().c_str(), actuated_joints_.size());

    for (const auto & joint : actuated_joints_) {
      RCLCPP_INFO(
        ros_node_->get_logger(),
        "  Actuated joint: %s", joint->GetName().c_str());
    }

    initPidFromSdf(sdf);
    initLegHomeFromLimits();     // <— now matches the no-arg signature
    setupRosInterfaces();
  }

private:
  void initPidFromSdf(const sdf::ElementPtr & sdf)
  {
    const sdf::ElementPtr pid_block =
      (sdf && sdf->HasElement("gazebo_controller_init"))
      ? sdf->GetElement("gazebo_controller_init")
      : nullptr;

    // Default: RL-tuned gains (can be overridden from SDF)
    propor_ = getValue(pid_block, "propor", 64.0);
    integr_ = getValue(pid_block, "integr", 35.0);
    deriv_  = getValue(pid_block, "deriv",  0.5);

    double i_max   = getValue(pid_block, "i_max",   1.0);
    double i_min   = getValue(pid_block, "i_min",  -1.0);
    double cmd_max = getValue(pid_block, "cmd_max", 500.0);
    double cmd_min = getValue(pid_block, "cmd_min",-500.0);

    pid_ = common::PID(
      propor_, integr_, deriv_,
      i_max, i_min,
      cmd_max, cmd_min);

    auto controller = model_->GetJointController();
    for (const auto & joint : actuated_joints_) {
      controller->SetPositionPID(joint->GetScopedName(), pid_);
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "[SDFJointController] PID: P=%.3f I=%.3f D=%.3f, cmd=[%.2f, %.2f]",
      propor_, integr_, deriv_, cmd_min, cmd_max);
  }

  // Define "home" as mid-stroke (centre between lower and upper limits)
  void initLegHomeFromLimits()
  {
    auto controller = model_->GetJointController();
    initial_positions_.clear();
    initial_positions_.reserve(actuated_joints_.size());

    for (const auto & joint : actuated_joints_) {
      double lower = joint->LowerLimit(0);
      double upper = joint->UpperLimit(0);
      double center;

      if (std::isfinite(lower) && std::isfinite(upper)) {
        center = 0.5 * (lower + upper);
      } else {
        // Fallback: keep spawn value if limits are not finite
        center = joint->Position(0);
      }

      initial_positions_.push_back(center);
      controller->SetPositionTarget(joint->GetScopedName(), center);

      RCLCPP_INFO(
        ros_node_->get_logger(),
        "[SDFJointController] Joint %s limits [%.4f, %.4f], center=%.4f",
        joint->GetName().c_str(), lower, upper, center);
    }
  }

  void setupRosInterfaces()
  {
    using std::placeholders::_1;

    // Topics are hard-coded to "/stewart/...".
    // Ensure your IK and pose publisher use the same names.
    position_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/stewart/legs_position_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handlePositionCommand, this, _1));

    pid_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Vector3>(
      "/stewart/pid_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handlePidCommand, this, _1));

    force_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/stewart/legs_force_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handleForceCommand, this, _1));

    base_pose_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Pose>(
      "/stewart/base_pose_cmd", rclcpp::SystemDefaultsQoS(),
      std::bind(&SDFJointController::handleBasePoseCommand, this, _1));

    effort_pub_ = ros_node_->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/stewart/joint_efforts", rclcpp::SystemDefaultsQoS());

    top_pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/stewart/top_platform_pose", rclcpp::SystemDefaultsQoS());

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SDFJointController::onUpdate, this, std::placeholders::_1));
  }

  // IK sends offsets from "home" (mid-stroke). We add these to the home positions.
  void handlePositionCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < actuated_joints_.size()) {
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "[SDFJointController] legs_position_cmd size %zu < num actuated joints %zu",
        msg->data.size(), actuated_joints_.size());
      return;
    }

    auto controller = model_->GetJointController();
    for (size_t i = 0; i < actuated_joints_.size(); ++i) {
      const float delta = msg->data[i];   // +/- offset from mid-stroke
      const double q_target = initial_positions_[i] + static_cast<double>(delta);
      controller->SetPositionTarget(actuated_joints_[i]->GetScopedName(), q_target);
    }
  }

  void handlePidCommand(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    pid_ = common::PID(msg->x, msg->y, msg->z);
    auto controller = model_->GetJointController();
    for (const auto & joint : actuated_joints_) {
      controller->SetPositionPID(joint->GetScopedName(), pid_);
    }

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "[SDFJointController] PID updated from /stewart/pid_cmd: P=%.3f I=%.3f D=%.3f",
      msg->x, msg->y, msg->z);
  }

  void handleForceCommand(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    auto joint_it = actuated_joints_.begin();
    for (const float force : msg->data) {
      if (joint_it == actuated_joints_.end()) {
        break;
      }
      (*joint_it)->SetForce(0, force);
      ++joint_it;
    }
  }

  void handleBasePoseCommand(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    base_pose_cmd_.Pos().Set(
      msg->position.x,
      msg->position.y,
      msg->position.z);
    base_pose_cmd_.Rot() = ignition::math::Quaterniond(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);
    has_base_pose_cmd_ = true;
  }

  void onUpdate(const common::UpdateInfo &)
  {
    if (base_link_ && has_base_pose_cmd_) {
      base_link_->SetWorldPose(base_pose_cmd_);
    }

    std_msgs::msg::Int32MultiArray msg;
    msg.data.reserve(actuated_joints_.size());
    for (const auto & joint : actuated_joints_) {
      msg.data.push_back(static_cast<int32_t>(joint->GetForce(0)));
    }
    effort_pub_->publish(msg);

    // publish top platform pose in world frame
    if (top_link_ && top_pose_pub_) {
      ignition::math::Pose3d pose = top_link_->WorldPose();

      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = ros_node_->get_clock()->now();
      pose_msg.header.frame_id = "world";

      pose_msg.pose.position.x = -pose.Pos().Y();
      pose_msg.pose.position.y = pose.Pos().X();
      pose_msg.pose.position.z = pose.Pos().Z();

      pose_msg.pose.orientation.x = pose.Rot().X();
      pose_msg.pose.orientation.y = pose.Rot().Y();
      pose_msg.pose.orientation.z = pose.Rot().Z();
      pose_msg.pose.orientation.w = pose.Rot().W();

      top_pose_pub_->publish(pose_msg);
    }
  }

  static double getValue(const sdf::ElementPtr & parent, const std::string & key, double fallback)
  {
    if (!parent || !parent->HasElement(key)) {
      return fallback;
    }
    return parent->Get<double>(key);
  }

  physics::ModelPtr model_;
  physics::LinkPtr top_link_;
  physics::LinkPtr base_link_;
  std::vector<physics::JointPtr> actuated_joints_;
  std::vector<double> initial_positions_;

  std::shared_ptr<gazebo_ros::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr base_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr effort_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr top_pose_pub_;
  
  event::ConnectionPtr update_connection_;

  ignition::math::Pose3d base_pose_cmd_;
  bool has_base_pose_cmd_{false};
  double propor_{64.0};
  double integr_{35.0};
  double deriv_{0.5};
  common::PID pid_;
};

GZ_REGISTER_MODEL_PLUGIN(SDFJointController)

}  // namespace gazebo
