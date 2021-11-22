
// carmen_to_bag_node.cpp

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "carmen_to_bag/carmen_laser_types.hpp"
#include "carmen_to_bag/carmen_loader.hpp"

// Compute a relative pose from `poseFrom` to `poseTo`
Eigen::Vector3d ComputeRelativePose(
  const Eigen::Vector3d poseFrom, const Eigen::Vector3d poseTo)
{
  Eigen::Vector3d relativePose;
  relativePose.head<2>() = Eigen::Rotation2Dd(-poseFrom[2])
    * (poseTo.head<2>() - poseFrom.head<2>());
  relativePose[2] = poseTo[2] - poseFrom[2];
  return relativePose;
}

struct PublishFlags
{
  // Flag to publish odometry messages
  bool mOdom;
  // Flag to publish odometry-to-robot tf from odometry messages
  bool mOdomTf;
  // Flag to publish ground-truth pose messages
  bool mTruePos;
  // Flag to publish truepos-to-robot tf from ground-truth pose messages
  bool mTruePosTf;
  // Flag to publish odometry from laser scan messages
  bool mLaserOdom;
  // Flag to publish odometry-to-robot tf from laser scan messages
  bool mLaserTfOdomToRobot;
  // Flag to publish robot-to-laser tf from laser scan messages
  bool mLaserTfRobotToLaser;
};

struct NodeSettings
{
  // Node publish settings
  PublishFlags mPublishFlags;
  // Topic name for the tf messages
  std::string mTopicTf;
  // Topic name for the odometry (ODOM)
  std::string mTopicOdom;
  // Topic name for the ground-truth pose (TRUEPOS)
  std::string mTopicTruePos;
  // Topic name of the robot odometry (ROBOTLASERx, FLASER, RLASER)
  std::string mTopicLaserOdom;
  // Topic name for the raw laser (RAWLASER1 to RAWLASER5)
  std::string mTopicRawLaser[5];
  // Topic name for the robot laser (ROBOTLASER1 to ROBOTLASER5)
  std::string mTopicRobotLaser[5];
  // Topic name for the front laser
  std::string mTopicFrontLaser;
  // Topic name for the rear laser
  std::string mTopicRearLaser;
  // Topic name for the old laser (LASER3, LASER4, LASER5)
  std::string mTopicOldLaser[3];
  // Frame Id of the robot (e.g., base_link)
  std::string mFrameIdRobot;
  // Frame Id of the odometry
  // Odometry data from the odometry messages (ODOM)
  std::string mFrameIdOdom;
  // Frame Id of the ground-truth pose
  std::string mFrameIdTruePos;
  // Frame Id of the robot odometry
  // Odometry data from the laser scan messages (ROBOTLASERx, FLASER, RLASER)
  std::string mFrameIdLaserOdom;
  // Frame Id of the raw laser (RAWLASER1 to RAWLASER5)
  std::string mFrameIdRawLaser[5];
  // Frame Id of the robot laser (ROBOTLASER1 to ROBOTLASER5)
  std::string mFrameIdRobotLaser[5];
  // Frame Id of the front laser
  std::string mFrameIdFrontLaser;
  // Frame Id of the rear laser
  std::string mFrameIdRearLaser;
  // Frame Id of the old laser (LASER3, LASER4, LASER5)
  std::string mFrameIdOldLaser[3];
};

class CarmenToBagNode
{
public:
  // Constructor
  CarmenToBagNode() = default;
  // Destructor
  ~CarmenToBagNode() = default;

  // Copy constructor (disabled)
  CarmenToBagNode(const CarmenToBagNode&) = delete;
  // Copy assignment operator (disabled)
  CarmenToBagNode& operator=(const CarmenToBagNode&) = delete;
  // Move constructor (disabled)
  CarmenToBagNode(CarmenToBagNode&&) = delete;
  // Move assignment operator (disabled)
  CarmenToBagNode& operator=(CarmenToBagNode&&) = delete;

  // Read ROS node parameters
  bool ReadParameters(ros::NodeHandle& node);
  // Convert Carmen log to ROS bag
  bool Convert(const CarmenLog& carmenLog, const std::string& bagFileName);
  // Run the ROS node
  bool Run(ros::NodeHandle& node);

  // Create sensor_msgs::LaserScan message from CarmenLaserMessage
  sensor_msgs::LaserScan CreateLaserScanMsg(
    const std::uint32_t sequence,
    const std::string& frameId,
    const std::shared_ptr<CarmenLaserMessage>& message);
  // Create sensor_msgs::LaserScan message from CarmenRobotLaserMessage
  sensor_msgs::LaserScan CreateLaserScanMsg(
    const std::uint32_t sequence,
    const std::string& frameId,
    const std::shared_ptr<CarmenRobotLaserMessage>& message);
  // Create geometry_msgs::TransformStamped message from 2D pose
  geometry_msgs::TransformStamped CreateTransformStampedMsg(
    const std::uint32_t sequence,
    const double timestamp,
    const std::string& frameId,
    const std::string& childFrameId,
    const Eigen::Vector3d& pose);
  // Create nav_msgs::Odometry message (without covariance and twist)
  nav_msgs::Odometry CreateOdometryMsg(
    const std::uint32_t sequence,
    const double timestamp,
    const std::string& frameId,
    const std::string& childFrameId,
    const Eigen::Vector3d& pose);

  // Write the odometry message
  bool WriteOdometry(
    rosbag::Bag& bag, std::uint32_t& messageSequence,
    const CarmenLog& carmenLog, const std::size_t logIdx);
  // Write the ground-truth pose message
  bool WriteTruePos(
    rosbag::Bag& bag, std::uint32_t& messageSequence,
    const CarmenLog& carmenLog, const std::size_t logIdx);
  // Write the raw laser message
  bool WriteRawLaser(
    rosbag::Bag& bag, std::uint32_t& messageSequence,
    const CarmenLog& carmenLog, const std::size_t logIdx);
  // Write the robot laser message
  bool WriteRobotLaser(
    rosbag::Bag& bag, std::uint32_t& messageSequence,
    const CarmenLog& carmenLog, const std::size_t logIdx);
  // Write the old laser message with pose
  bool WriteOldLaserWithPose(
    rosbag::Bag& bag, std::uint32_t& messageSequence,
    const CarmenLog& carmenLog, const std::size_t logIdx);
  // Write the old laser message
  bool WriteOldLaser(
    rosbag::Bag& bag, std::uint32_t& messageSequence,
    const CarmenLog& carmenLog, const std::size_t logIdx);

  // Dump the relative pose between robot and laser
  void DumpRobotToLaserPose(const CarmenLog& carmenLog);

private:
  // ROS node settings
  NodeSettings mSettings;
};

// Create sensor_msgs::LaserScan message from CarmenLaserMessage
sensor_msgs::LaserScan CarmenToBagNode::CreateLaserScanMsg(
  const std::uint32_t sequence,
  const std::string& frameId,
  const std::shared_ptr<CarmenLaserMessage>& message)
{
  sensor_msgs::LaserScan laserScan;

  laserScan.header.seq = sequence;
  laserScan.header.stamp = ros::Time(message->mTimestamp);
  laserScan.header.frame_id = frameId;

  laserScan.angle_min = static_cast<float>(message->mConfig.mStartAngle);
  laserScan.angle_max = static_cast<float>(message->mConfig.mStartAngle
    + message->mConfig.mAngularResolution * (message->mRanges.size() - 1));
  laserScan.angle_increment = static_cast<float>(
    message->mConfig.mAngularResolution);
  laserScan.time_increment = 0.0f;
  laserScan.scan_time = 0.0f;
  laserScan.range_min = 0.0f;
  laserScan.range_max = static_cast<float>(message->mConfig.mMaximumRange);
  laserScan.ranges = message->mRanges;
  laserScan.intensities = message->mRemissions;

  return laserScan;
}

// Create sensor_msgs::LaserScan message from CarmenRobotLaserMessage
sensor_msgs::LaserScan CarmenToBagNode::CreateLaserScanMsg(
  const std::uint32_t sequence,
  const std::string& frameId,
  const std::shared_ptr<CarmenRobotLaserMessage>& message)
{
  sensor_msgs::LaserScan laserScan;

  laserScan.header.seq = sequence;
  laserScan.header.stamp = ros::Time(message->mTimestamp);
  laserScan.header.frame_id = frameId;

  laserScan.angle_min = static_cast<float>(message->mConfig.mStartAngle);
  laserScan.angle_max = static_cast<float>(message->mConfig.mStartAngle
    + message->mConfig.mAngularResolution * (message->mRanges.size() - 1));
  laserScan.angle_increment = static_cast<float>(
    message->mConfig.mAngularResolution);
  laserScan.time_increment = 0.0f;
  laserScan.scan_time = 0.0f;
  laserScan.range_min = 0.0f;
  laserScan.range_max = static_cast<float>(message->mConfig.mMaximumRange);
  laserScan.ranges = message->mRanges;
  laserScan.intensities = message->mRemissions;

  return laserScan;
}

// Create geometry_msgs::TransformStamped message from 2D pose
geometry_msgs::TransformStamped CarmenToBagNode::CreateTransformStampedMsg(
  const std::uint32_t sequence,
  const double timestamp,
  const std::string& frameId,
  const std::string& childFrameId,
  const Eigen::Vector3d& pose)
{
  geometry_msgs::TransformStamped transformMsg;

  transformMsg.header.seq = sequence;
  transformMsg.header.stamp = ros::Time(timestamp);
  transformMsg.header.frame_id = frameId;
  transformMsg.child_frame_id = childFrameId;

  const tf::Vector3 translation { pose[0], pose[1], 0.0 };
  tf::vector3TFToMsg(translation, transformMsg.transform.translation);
  const tf::Quaternion rotation = tf::createQuaternionFromYaw(pose[2]);
  tf::quaternionTFToMsg(rotation, transformMsg.transform.rotation);

  return transformMsg;
}

// Create nav_msgs::Odometry message (without covariance and twist)
nav_msgs::Odometry CarmenToBagNode::CreateOdometryMsg(
  const std::uint32_t sequence,
  const double timestamp,
  const std::string& frameId,
  const std::string& childFrameId,
  const Eigen::Vector3d& pose)
{
  nav_msgs::Odometry odometry;

  odometry.header.seq = sequence;
  odometry.header.stamp = ros::Time(timestamp);
  odometry.header.frame_id = frameId;
  odometry.child_frame_id = childFrameId;

  const tf::Point translation { pose[0], pose[1], 0.0 };
  tf::pointTFToMsg(translation, odometry.pose.pose.position);
  const tf::Quaternion rotation = tf::createQuaternionFromYaw(pose[2]);
  tf::quaternionTFToMsg(rotation, odometry.pose.pose.orientation);

  // `odometry.pose.covariance` and `odometry.twist` are filled with zeros
  const tf::Vector3 zero { 0.0, 0.0, 0.0 };
  tf::vector3TFToMsg(zero, odometry.twist.twist.linear);
  tf::vector3TFToMsg(zero, odometry.twist.twist.angular);
  std::fill(odometry.pose.covariance.begin(),
            odometry.pose.covariance.end(), 0.0);
  std::fill(odometry.twist.covariance.begin(),
            odometry.twist.covariance.end(), 0.0);

  return odometry;
}

// Write the odometry message
bool CarmenToBagNode::WriteOdometry(
  rosbag::Bag& bag, std::uint32_t& messageSequence,
  const CarmenLog& carmenLog, const std::size_t logIdx)
{
  const auto message = std::dynamic_pointer_cast<CarmenOdometryMessage>(
    carmenLog.mMessages[logIdx]);
  ROS_ASSERT(message != nullptr);

  if (this->mSettings.mPublishFlags.mOdom) {
    // Create nav_msgs::Odometry message
    auto odometry = this->CreateOdometryMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdOdom, this->mSettings.mFrameIdRobot,
      message->mPose);
    bag.write(this->mSettings.mTopicOdom, odometry.header.stamp, odometry);
  }

  if (this->mSettings.mPublishFlags.mOdomTf) {
    // Create geometry_msgs::TransformStamped (odometry to robot)
    auto tfOdometryToRobot = this->CreateTransformStampedMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdOdom, this->mSettings.mFrameIdRobot,
      message->mPose);
    // Create tf2_msgs::TFMessage
    tf2_msgs::TFMessage tfMsg;
    tfMsg.transforms.push_back(std::move(tfOdometryToRobot));
    bag.write(this->mSettings.mTopicTf, ros::Time(message->mTimestamp), tfMsg);
  }

  return true;
}

// Write the ground-truth pose message
bool CarmenToBagNode::WriteTruePos(
  rosbag::Bag& bag, std::uint32_t& messageSequence,
  const CarmenLog& carmenLog, const std::size_t logIdx)
{
  const auto message = std::dynamic_pointer_cast<CarmenTruePosMessage>(
    carmenLog.mMessages[logIdx]);
  ROS_ASSERT(message != nullptr);

  // `message->mOdometryPose` is ignored

  if (this->mSettings.mPublishFlags.mTruePos) {
    // Create nav_msgs::Odometry message
    auto groundTruth = this->CreateOdometryMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdTruePos, this->mSettings.mFrameIdRobot,
      message->mTruePose);
    bag.write(this->mSettings.mTopicTruePos,
              groundTruth.header.stamp, groundTruth);
  }

  if (this->mSettings.mPublishFlags.mTruePosTf) {
    // Create geometry_msgs::TransformStamped (truepos to robot)
    auto tfGroundTruth = this->CreateTransformStampedMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdTruePos, this->mSettings.mFrameIdRobot,
      message->mTruePose);
    // Create tf2_msgs::TFMessage
    tf2_msgs::TFMessage tfMsg;
    tfMsg.transforms.push_back(std::move(tfGroundTruth));
    bag.write(this->mSettings.mTopicTf, ros::Time(message->mTimestamp), tfMsg);
  }

  return true;
}

// Write the raw laser message
bool CarmenToBagNode::WriteRawLaser(
  rosbag::Bag& bag, std::uint32_t& messageSequence,
  const CarmenLog& carmenLog, const std::size_t logIdx)
{
  const auto message = std::dynamic_pointer_cast<CarmenLaserMessage>(
    carmenLog.mMessages[logIdx]);

  ROS_ASSERT(message != nullptr);
  ROS_ASSERT(message->mSensorName == "RAWLASER");
  ROS_ASSERT(message->mSensorId >= 1 && message->mSensorId <= 5);

  const std::string& frameIdLaser =
    this->mSettings.mFrameIdRawLaser[message->mSensorId - 1];

  // Create sensor_msgs::LaserScan message
  const auto laserScan = this->CreateLaserScanMsg(
    messageSequence++, frameIdLaser, message);
  const std::string& topicLaserScan =
    this->mSettings.mTopicRawLaser[message->mSensorId - 1];
  bag.write(topicLaserScan, laserScan.header.stamp, laserScan);

  return true;
}

// Write the robot laser message
bool CarmenToBagNode::WriteRobotLaser(
  rosbag::Bag& bag, std::uint32_t& messageSequence,
  const CarmenLog& carmenLog, const std::size_t logIdx)
{
  const auto message = std::dynamic_pointer_cast<CarmenRobotLaserMessage>(
    carmenLog.mMessages[logIdx]);

  ROS_ASSERT(message != nullptr);
  ROS_ASSERT(message->mSensorName == "ROBOTLASER");
  ROS_ASSERT(message->mSensorId >= 1 && message->mSensorId <= 5);

  const std::string& frameIdLaser =
    this->mSettings.mFrameIdRobotLaser[message->mSensorId - 1];

  // Create sensor_msgs::LaserScan message
  const auto laserScan = this->CreateLaserScanMsg(
    messageSequence++, frameIdLaser, message);
  const std::string& topicLaserScan =
    this->mSettings.mTopicRobotLaser[message->mSensorId - 1];
  bag.write(topicLaserScan, laserScan.header.stamp, laserScan);

  if (this->mSettings.mPublishFlags.mLaserOdom) {
    // Create nav_msgs::Odometry message
    auto odometry = this->CreateOdometryMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdLaserOdom, this->mSettings.mFrameIdRobot,
      message->mRobotPose);
    bag.write(this->mSettings.mTopicLaserOdom, odometry.header.stamp, odometry);
  }

  // Create tf2_msgs::TFMessage
  tf2_msgs::TFMessage tfMsg;

  if (this->mSettings.mPublishFlags.mLaserTfRobotToLaser) {
    // Compute relative pose from robot to laser
    const Eigen::Vector3d robotToLaserPose =
      ComputeRelativePose(message->mRobotPose, message->mLaserPose);
    // Create geometry_msgs::TransformStamped (robot to laser)
    auto tfRobotToLaser = this->CreateTransformStampedMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdRobot, frameIdLaser, robotToLaserPose);
    tfMsg.transforms.push_back(std::move(tfRobotToLaser));
  }

  if (this->mSettings.mPublishFlags.mLaserTfOdomToRobot) {
    // Create geometry_msgs::TransformStamped (odometry to robot)
    auto tfOdomToRobot = this->CreateTransformStampedMsg(
      messageSequence++, message->mTimestamp + 1e-2,
      this->mSettings.mFrameIdLaserOdom, this->mSettings.mFrameIdRobot,
      message->mRobotPose);
    tfMsg.transforms.push_back(std::move(tfOdomToRobot));
  }

  if (!tfMsg.transforms.empty())
    bag.write(this->mSettings.mTopicTf, laserScan.header.stamp, tfMsg);

  return true;
}

// Write the old laser message with pose
bool CarmenToBagNode::WriteOldLaserWithPose(
  rosbag::Bag& bag, std::uint32_t& messageSequence,
  const CarmenLog& carmenLog, const std::size_t logIdx)
{
  const auto message = std::dynamic_pointer_cast<CarmenRobotLaserMessage>(
    carmenLog.mMessages[logIdx]);

  ROS_ASSERT(message != nullptr);
  ROS_ASSERT(message->mSensorName == "FLASER" ||
             message->mSensorName == "RLASER");

  const std::string& frameIdLaser = message->mSensorName == "FLASER" ?
    this->mSettings.mFrameIdFrontLaser : this->mSettings.mFrameIdRearLaser;

  // Create sensor_msgs::LaserScan message
  const auto laserScan = this->CreateLaserScanMsg(
    messageSequence++, frameIdLaser, message);
  const std::string& topicLaserScan = message->mSensorName == "FLASER" ?
    this->mSettings.mTopicFrontLaser : this->mSettings.mTopicRearLaser;
  bag.write(topicLaserScan, laserScan.header.stamp, laserScan);

  if (this->mSettings.mPublishFlags.mLaserOdom) {
    // Create nav_msgs::Odometry message
    auto odometry = this->CreateOdometryMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdLaserOdom, this->mSettings.mFrameIdRobot,
      message->mRobotPose);
    bag.write(this->mSettings.mTopicLaserOdom, odometry.header.stamp, odometry);
  }

  // Create tf2_msgs::TFMessage
  tf2_msgs::TFMessage tfMsg;

  if (this->mSettings.mPublishFlags.mLaserTfRobotToLaser) {
    // Compute relative pose from robot to laser
    const Eigen::Vector3d robotToLaserPose =
      ComputeRelativePose(message->mRobotPose, message->mLaserPose);
    // Create geometry_msgs::TransformStamped (robot to laser)
    auto tfRobotToLaser = this->CreateTransformStampedMsg(
      messageSequence++, message->mTimestamp,
      this->mSettings.mFrameIdRobot, frameIdLaser, robotToLaserPose);
    tfMsg.transforms.push_back(std::move(tfRobotToLaser));
  }

  if (this->mSettings.mPublishFlags.mLaserTfOdomToRobot) {
    // Create geometry_msgs::TransformStamped (odometry to robot)
    // Publish the message a little bit in the future, so that
    // `lookupTransform(frameIdOdom, frameIdRobot, timestamp)` succeeds
    auto tfOdomToRobot = this->CreateTransformStampedMsg(
      messageSequence++, message->mTimestamp + 1e-2,
      this->mSettings.mFrameIdLaserOdom, this->mSettings.mFrameIdRobot,
      message->mRobotPose);
    tfMsg.transforms.push_back(std::move(tfOdomToRobot));
  }

  if (!tfMsg.transforms.empty())
    bag.write(this->mSettings.mTopicTf, laserScan.header.stamp, tfMsg);

  return true;
}

// Write the old laser message
bool CarmenToBagNode::WriteOldLaser(
  rosbag::Bag& bag, std::uint32_t& messageSequence,
  const CarmenLog& carmenLog, const std::size_t logIdx)
{
  const auto message = std::dynamic_pointer_cast<CarmenLaserMessage>(
    carmenLog.mMessages[logIdx]);

  ROS_ASSERT(message != nullptr);
  ROS_ASSERT(message->mSensorName == "LASER");
  ROS_ASSERT(message->mSensorId >= 3 && message->mSensorId <= 5);

  const std::string& frameIdLaser =
    this->mSettings.mFrameIdOldLaser[message->mSensorId - 3];

  // Create sensor_msgs::LaserScan message
  const auto laserScan = this->CreateLaserScanMsg(
    messageSequence++, frameIdLaser, message);
  const std::string& topicLaserScan =
    this->mSettings.mTopicOldLaser[message->mSensorId - 3];
  bag.write(topicLaserScan, laserScan.header.stamp, laserScan);

  return true;
}

// Dump the relative pose between robot and laser
void CarmenToBagNode::DumpRobotToLaserPose(const CarmenLog& carmenLog)
{
  using ParameterIterator = std::vector<CarmenParameterPtr>::const_iterator;
  auto getValue = [&carmenLog](const ParameterIterator& param) {
    return param == carmenLog.mParameters.end() ? 0.0 :
           !(*param)->IsDouble() ? 0.0 :
           boost::get<double>((*param)->mValue);
  };

  // Front laser (FLASER)
  // Compare the robot-to-laser relative pose specified by the parameters
  // and the relative pose in the actual message (should be the same)
  const auto frontLaserOffset = carmenLog.FindParameter(
    "robot_frontlaser_offset");
  const auto frontLaserSideOffset = carmenLog.FindParameter(
    "robot_frontlaser_side_offset");
  const auto frontLaserAngularOffset = carmenLog.FindParameter(
    "robot_frontlaser_angular_offset");

  const Eigen::Vector3d paramFrontLaserPose {
    getValue(frontLaserOffset), getValue(frontLaserSideOffset),
    getValue(frontLaserAngularOffset) };

  const auto frontLaserMsgIt = carmenLog.FindMessage("FLASER");

  if (frontLaserMsgIt != carmenLog.mMessages.end()) {
    const auto frontLaserMsg = std::dynamic_pointer_cast<
      CarmenRobotLaserMessage>(*frontLaserMsgIt);
    const Eigen::Vector3d frontLaserPose = ComputeRelativePose(
      frontLaserMsg->mRobotPose, frontLaserMsg->mLaserPose);
    ROS_INFO("Relative pose from robot to laser (FLASER): "
             "[%.6f, %.6f, %.6f] (message), "
             "[%.6f, %.6f, %.6f] (parameter)",
             frontLaserPose[0], frontLaserPose[1],
             frontLaserPose[2],
             paramFrontLaserPose[0], paramFrontLaserPose[1],
             paramFrontLaserPose[2]);
  }

  // Rear laser (RLASER)
  // Compare the robot-to-laser relative pose specified by the parameters
  // and the relative pose in the actual message (should be the same)
  const auto rearLaserOffset = carmenLog.FindParameter(
    "robot_rearlaser_offset");
  const auto rearLaserSideOffset = carmenLog.FindParameter(
    "robot_rearlaser_side_offset");
  const auto rearLaserAngularOffset = carmenLog.FindParameter(
    "robot_rearlaser_angular_offset");

  const Eigen::Vector3d paramRearLaserPose {
    getValue(rearLaserOffset), getValue(rearLaserSideOffset),
    getValue(rearLaserAngularOffset) };

  const auto rearLaserMsgIt = carmenLog.FindMessage("RLASER");

  if (rearLaserMsgIt != carmenLog.mMessages.end()) {
    const auto rearLaserMsg = std::dynamic_pointer_cast<
      CarmenRobotLaserMessage>(*rearLaserMsgIt);
    const Eigen::Vector3d rearLaserPose = ComputeRelativePose(
      rearLaserMsg->mRobotPose, rearLaserMsg->mLaserPose);
    ROS_INFO("Relative pose from robot to laser (FLASER): "
             "[%.6f, %.6f, %.6f] (message), "
             "[%.6f, %.6f, %.6f] (parameter)",
             rearLaserPose[0], rearLaserPose[1],
             rearLaserPose[2],
             paramRearLaserPose[0], paramRearLaserPose[1],
             paramRearLaserPose[2]);
  }

  // Robot laser (ROBOTLASERx)
  for (int i = 1; i <= 5; ++i) {
    const auto robotLaserMsgIt = carmenLog.FindMessage("ROBOTLASER", i);

    if (robotLaserMsgIt != carmenLog.mMessages.end()) {
      const auto robotLaserMsg = std::dynamic_pointer_cast<
        CarmenRobotLaserMessage>(*robotLaserMsgIt);
      const Eigen::Vector3d robotLaserPose = ComputeRelativePose(
        robotLaserMsg->mRobotPose, robotLaserMsg->mLaserPose);
      ROS_INFO("Relative pose from robot to laser (ROBOTLASER%d): "
               "[%.6f, %.6f, %.6f]",
               i, robotLaserPose[0], robotLaserPose[1], robotLaserPose[2]);
    }
  }
}

#define GET_PARAM(node, paramName, param) \
  if (!(node).getParam((paramName), (param))) { \
    ROS_ERROR("CarmenToBagNode: Parameter `%s` not specified", (paramName)); \
    return false; \
  }

// Read ROS node parameters
bool CarmenToBagNode::ReadParameters(ros::NodeHandle& node)
{
  auto& settings = this->mSettings;

  GET_PARAM(node, "publish_odom",
            settings.mPublishFlags.mOdom);
  GET_PARAM(node, "publish_odom_tf",
            settings.mPublishFlags.mOdomTf);
  GET_PARAM(node, "publish_true_pos",
            settings.mPublishFlags.mTruePos);
  GET_PARAM(node, "publish_true_pos_tf",
            settings.mPublishFlags.mTruePosTf);
  GET_PARAM(node, "publish_laser_odom",
            settings.mPublishFlags.mLaserOdom);
  GET_PARAM(node, "publish_laser_tf_odom_to_robot",
            settings.mPublishFlags.mLaserTfOdomToRobot);
  GET_PARAM(node, "publish_laser_tf_robot_to_laser",
            settings.mPublishFlags.mLaserTfRobotToLaser);

  GET_PARAM(node, "topic_tf", settings.mTopicTf);
  GET_PARAM(node, "topic_odom", settings.mTopicOdom);
  GET_PARAM(node, "topic_true_pos", settings.mTopicTruePos);
  GET_PARAM(node, "topic_laser_odom", settings.mTopicLaserOdom);
  GET_PARAM(node, "topic_raw_laser_1", settings.mTopicRawLaser[0]);
  GET_PARAM(node, "topic_raw_laser_2", settings.mTopicRawLaser[1]);
  GET_PARAM(node, "topic_raw_laser_3", settings.mTopicRawLaser[2]);
  GET_PARAM(node, "topic_raw_laser_4", settings.mTopicRawLaser[3]);
  GET_PARAM(node, "topic_raw_laser_5", settings.mTopicRawLaser[4]);
  GET_PARAM(node, "topic_robot_laser_1", settings.mTopicRobotLaser[0]);
  GET_PARAM(node, "topic_robot_laser_2", settings.mTopicRobotLaser[1]);
  GET_PARAM(node, "topic_robot_laser_3", settings.mTopicRobotLaser[2]);
  GET_PARAM(node, "topic_robot_laser_4", settings.mTopicRobotLaser[3]);
  GET_PARAM(node, "topic_robot_laser_5", settings.mTopicRobotLaser[4]);
  GET_PARAM(node, "topic_front_laser", settings.mTopicFrontLaser);
  GET_PARAM(node, "topic_rear_laser", settings.mTopicRearLaser);
  GET_PARAM(node, "topic_old_laser_3", settings.mTopicOldLaser[0]);
  GET_PARAM(node, "topic_old_laser_4", settings.mTopicOldLaser[1]);
  GET_PARAM(node, "topic_old_laser_5", settings.mTopicOldLaser[2]);

  GET_PARAM(node, "frame_id_robot", settings.mFrameIdRobot);
  GET_PARAM(node, "frame_id_odom", settings.mFrameIdOdom);
  GET_PARAM(node, "frame_id_true_pos", settings.mFrameIdTruePos);
  GET_PARAM(node, "frame_id_laser_odom", settings.mFrameIdLaserOdom);
  GET_PARAM(node, "frame_id_raw_laser_1", settings.mFrameIdRawLaser[0]);
  GET_PARAM(node, "frame_id_raw_laser_2", settings.mFrameIdRawLaser[1]);
  GET_PARAM(node, "frame_id_raw_laser_3", settings.mFrameIdRawLaser[2]);
  GET_PARAM(node, "frame_id_raw_laser_4", settings.mFrameIdRawLaser[3]);
  GET_PARAM(node, "frame_id_raw_laser_5", settings.mFrameIdRawLaser[4]);
  GET_PARAM(node, "frame_id_robot_laser_1", settings.mFrameIdRobotLaser[0]);
  GET_PARAM(node, "frame_id_robot_laser_2", settings.mFrameIdRobotLaser[1]);
  GET_PARAM(node, "frame_id_robot_laser_3", settings.mFrameIdRobotLaser[2]);
  GET_PARAM(node, "frame_id_robot_laser_4", settings.mFrameIdRobotLaser[3]);
  GET_PARAM(node, "frame_id_robot_laser_5", settings.mFrameIdRobotLaser[4]);
  GET_PARAM(node, "frame_id_front_laser", settings.mFrameIdFrontLaser);
  GET_PARAM(node, "frame_id_rear_laser", settings.mFrameIdRearLaser);
  GET_PARAM(node, "frame_id_old_laser_3", settings.mFrameIdOldLaser[0]);
  GET_PARAM(node, "frame_id_old_laser_4", settings.mFrameIdOldLaser[1]);
  GET_PARAM(node, "frame_id_old_laser_5", settings.mFrameIdOldLaser[2]);

  return true;
}

// Convert Carmen log to ROS bag
bool CarmenToBagNode::Convert(
  const CarmenLog& carmenLog, const std::string& bagFileName)
{
  // Open the ROS bag
  rosbag::Bag bag;
  bag.open(bagFileName, rosbag::bagmode::Write);

  if (!bag.isOpen()) {
    ROS_ERROR("CarmenToBagNode: Failed to open ROS bag: %s",
              bagFileName.c_str());
    return false;
  }

  std::uint32_t sequence = 0;

  // Write messages to the ROS bag
  for (std::size_t i = 0; i < carmenLog.mMessages.size(); ++i) {
    const CarmenMessagePtr& message = carmenLog.mMessages[i];

    switch (message->mSensorType) {
      case CarmenSensorType::Odom:
        this->WriteOdometry(bag, sequence, carmenLog, i);
        break;
      case CarmenSensorType::TruePos:
        this->WriteOdometry(bag, sequence, carmenLog, i);
        break;
      case CarmenSensorType::RawLaser:
        this->WriteRawLaser(bag, sequence, carmenLog, i);
        break;
      case CarmenSensorType::RobotLaser:
        this->WriteRobotLaser(bag, sequence, carmenLog, i);
        break;
      case CarmenSensorType::OldLaserWithPose:
        this->WriteOldLaserWithPose(bag, sequence, carmenLog, i);
        break;
      case CarmenSensorType::OldLaser:
        this->WriteOldLaser(bag, sequence, carmenLog, i);
        break;
    }
  }

  // Close the ROS bag
  bag.close();

  return true;
}

// Run the ROS node
bool CarmenToBagNode::Run(ros::NodeHandle& node)
{
  ros::NodeHandle privateNode { "~" };

  // Read the parameters
  if (!this->ReadParameters(privateNode)) {
    ROS_ERROR("CarmenToBagNode: Failed to read parameters");
    return false;
  }

  std::string carmenFileName;
  if (!privateNode.getParam("carmen_file_name", carmenFileName)) {
    ROS_ERROR("CarmenToBagNode: Parameter `carmen_file_name` not specified");
    return false;
  }

  std::string bagFileName;
  if (!privateNode.getParam("bag_file_name", bagFileName)) {
    ROS_ERROR("CarmenToBagNode: Parameter `bag_file_name` not specified");
    return false;
  }

  // Load the Carmen log
  CarmenLog carmenLog;
  CarmenLoader carmenLoader;
  if (!carmenLoader.Load(carmenFileName, carmenLog)) {
    ROS_ERROR("CarmenToBagNode: Failed to load Carmen log: %s",
              carmenFileName.c_str());
    return false;
  }

  ROS_INFO("CarmenToBagNode: Carmen log loaded: %s",
           carmenFileName.c_str());
  ROS_INFO("CarmenToBagNode: Number of the parameters: %ld, "
           "Number of the messages: %ld",
           carmenLog.mParameters.size(),
           carmenLog.mMessages.size());

  // Dump the relative pose between robot and laser
  this->DumpRobotToLaserPose(carmenLog);

  // Convert to ROS bag
  this->Convert(carmenLog, bagFileName);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carmen_to_bag_node");
  ros::NodeHandle nodeHandle;
  CarmenToBagNode carmenToBag;

  if (!carmenToBag.Run(nodeHandle)) {
    ROS_ERROR("CarmenToBagNode: Failed to convert Carmen log to ROS bag");
    return EXIT_FAILURE;
  }

  ROS_INFO("CarmenToBagNode: Exiting");

  return EXIT_SUCCESS;
}
