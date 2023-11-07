#include "pronto_quadruped_ros/conversions.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pronto_quadruper_ros_conversions");

namespace pronto {
namespace quadruped {

bool jointStateFromROS(const sensor_msgs::msg::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau)
{
    const std::vector<std::string> jointOrder{"first_arm_yaw", "first_arm_pitch", "first_knee_joint",
                                              "fourth_arm_yaw", "fourth_arm_pitch", "fourth_knee_joint",
                                              "second_arm_yaw", "second_arm_pitch", "second_knee_joint", 
                                              "third_arm_yaw", "third_arm_pitch", "third_knee_joint"};

    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const sensor_msgs::msg::JointState&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::msg::JointState&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::msg::JointState&>(msg).effort.size()) != q.size()){
        RCLCPP_WARN_STREAM(LOGGER, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000;
    for (size_t i = 0; i < 12; i++){
      auto it = std::find(jointOrder.begin(), jointOrder.end(), msg.name[i]);
      if (it == jointOrder.end()){
        RCLCPP_WARN_STREAM(LOGGER, "Joint name not matching expected joint name!");
        return false;
      }
      size_t orderedIdx = it - jointOrder.begin();
      q(orderedIdx) = msg.position[i];
      qd(orderedIdx) = msg.velocity[i];
      tau(orderedIdx) = msg.effort[i];
    }
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

    qdd.setZero(); // TODO compute the acceleration

    return true;
}

bool jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau)
{
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).acceleration.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).effort.size()) != q.size()){
        RCLCPP_WARN_STREAM(LOGGER, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.acceleration.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.sec * 1e6 + 1e-3 * msg.header.stamp.nanosec;
    for(int i=0; i<12; i++){
      q(i) = msg.position[i];
      qd(i) = msg.velocity[i];
      qdd(i) = msg.acceleration[i];
      tau(i) = msg.effort[i];
    }

    return true;
}

}  // namespace quadruped
}  // namespace pronto
