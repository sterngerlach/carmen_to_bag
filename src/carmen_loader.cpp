
// carmen_loader.cpp

#include "carmen_to_bag/carmen_loader.hpp"

namespace fs = boost::filesystem;

template <typename T>
inline constexpr T Pi = static_cast<T>(3.14159265358979323846);

bool StartsWith(const std::string& str, const std::string& prefix)
{
  return str.size() >= prefix.size() &&
         std::equal(prefix.begin(), prefix.end(), str.begin());
}

// Find a parameter with the given name
std::vector<CarmenParameterPtr>::const_iterator CarmenLog::FindParameter(
  const std::string& paramName) const
{
  return std::find_if(this->mParameters.begin(), this->mParameters.end(),
    [paramName](const CarmenParameterPtr& param) {
      return param->mName == paramName; });
}

// Find a message with the given message type
std::vector<CarmenMessagePtr>::const_iterator CarmenLog::FindMessage(
  const CarmenSensorType sensorType) const
{
  return std::find_if(this->mMessages.begin(), this->mMessages.end(),
    [sensorType](const CarmenMessagePtr& message) {
      return message->mSensorType == sensorType; });
}

// Find a message with the given sensor name
std::vector<CarmenMessagePtr>::const_iterator CarmenLog::FindMessage(
  const std::string& sensorName, const int sensorId) const
{
  auto predicate = [sensorName, sensorId](const CarmenMessagePtr& message) {
    if (message->mSensorType == CarmenSensorType::RawLaser ||
        message->mSensorType == CarmenSensorType::OldLaser) {
      const auto laserMessage = std::dynamic_pointer_cast<
        CarmenLaserMessage>(message);
      return laserMessage != nullptr &&
             laserMessage->mSensorName == sensorName &&
             laserMessage->mSensorId == sensorId;
    } else if (message->mSensorType == CarmenSensorType::RobotLaser ||
               message->mSensorType == CarmenSensorType::OldLaserWithPose) {
      const auto laserMessage = std::dynamic_pointer_cast<
        CarmenRobotLaserMessage>(message);
      return laserMessage != nullptr &&
             laserMessage->mSensorName == sensorName &&
             laserMessage->mSensorId == sensorId;
    }
    return false;
  };

  return std::find_if(this->mMessages.begin(), this->mMessages.end(),
                      predicate);
}

// Load Carmen log
bool CarmenLoader::Load(const std::string& carmenFileName,
                        CarmenLog& carmenLog)
{
  // Check that the specified Carmen log file exists
  if (!fs::exists(carmenFileName)) {
    ROS_ERROR("CarmenLoader: Carmen log file does not exist: %s",
              carmenFileName.c_str());
    return false;
  }

  // Open the Carmen log
  std::ifstream carmenFile { carmenFileName };
  if (!carmenFile) {
    ROS_ERROR("CarmenLoader: Failed to open Carmen log file: %s",
              carmenFileName.c_str());
    return false;
  }

  // Load the data records
  // Assume that the messages are sorted in chronologically
  carmenLog.mParameters.clear();
  carmenLog.mMessages.clear();

  std::string line;
  std::string sensorName;
  while (std::getline(carmenFile, line)) {
    std::istringstream strStream { line };

    // Skip if empty
    if (!strStream)
      continue;
    // Get the sensor name (e.g., FLASER, ODOM)
    if (!(strStream >> sensorName))
      continue;

    switch (ToCarmenSensorType(sensorName)) {
      case CarmenSensorType::Param: {
        CarmenParameter parameter;
        if (!this->ReadParameter(strStream, parameter))
          continue;
        else
          carmenLog.mParameters.push_back(
            std::make_shared<CarmenParameter>(parameter));
        break;
      }
      case CarmenSensorType::Odom: {
        CarmenOdometryMessage message;
        if (!this->ReadOdometry(strStream, message))
          continue;
        else
          carmenLog.mMessages.push_back(
            std::make_shared<CarmenOdometryMessage>(message));
        break;
      }
      case CarmenSensorType::TruePos: {
        CarmenTruePosMessage message;
        if (!this->ReadTruePos(strStream, message))
          continue;
        else
          carmenLog.mMessages.push_back(
            std::make_shared<CarmenTruePosMessage>(message));
        break;
      }
      case CarmenSensorType::RawLaser: {
        CarmenLaserMessage message;
        if (!this->ReadRawLaser(sensorName, strStream, message))
          continue;
        else
          carmenLog.mMessages.push_back(
            std::make_shared<CarmenLaserMessage>(message));
        break;
      }
      case CarmenSensorType::RobotLaser: {
        CarmenRobotLaserMessage message;
        if (!this->ReadRobotLaser(sensorName, strStream, message))
          continue;
        else
          carmenLog.mMessages.push_back(
            std::make_shared<CarmenRobotLaserMessage>(message));
        break;
      }
      case CarmenSensorType::OldLaserWithPose: {
        CarmenRobotLaserMessage message;
        if (!this->ReadOldLaserWithPose(sensorName, strStream, message))
          continue;
        else
          carmenLog.mMessages.push_back(
            std::make_shared<CarmenRobotLaserMessage>(message));
        break;
      }
      case CarmenSensorType::OldLaser: {
        CarmenLaserMessage message;
        if (!this->ReadOldLaser(sensorName, strStream, message))
          continue;
        else
          carmenLog.mMessages.push_back(
            std::make_shared<CarmenLaserMessage>(message));
        break;
      }
    }
  }

  // Close the Carmen log
  carmenFile.close();

  return true;
}

// Read parameter
bool CarmenLoader::ReadParameter(
  std::istringstream& strStream, CarmenParameter& parameter)
{
  // PARAM param_name param_value
  std::string name;
  std::string value;

  if (!strStream)
    return false;
  if (!(strStream >> name))
    return false;
  if (!(strStream >> value))
    return false;

  // Relative pose between robot and laser scanner
  if (name == "robot_frontlaser_offset" ||
      name == "robot_frontlaser_side_offset" ||
      name == "robot_frontlaser_angular_offset" ||
      name == "robot_rearlaser_offset" ||
      name == "robot_rearlaser_side_offset" ||
      name == "robot_rearlaser_angular_offset") {
    parameter.mName = name;
    parameter.mValue = std::stod(value);
  } else {
    parameter.mName = name;
    parameter.mValue = value;
  }

  return true;
}

// Read odometry message
bool CarmenLoader::ReadOdometry(
  std::istringstream& strStream, CarmenOdometryMessage& odometryMessage)
{
  // ODOM x y theta tv rv accel

  odometryMessage.mSensorType = CarmenSensorType::Odom;

  // Read the odometry pose
  if (!CarmenReadPose2D(strStream, odometryMessage.mPose))
    return false;
  // Read the translational velocity
  if (!(strStream >> odometryMessage.mVelocityTrans))
    return false;
  // Read the rotational velocity
  if (!(strStream >> odometryMessage.mVelocityRot))
    return false;
  // Read the acceleration
  if (!(strStream >> odometryMessage.mAcceleration))
    return false;
  // Read the timestamp
  if (!(strStream >> odometryMessage.mTimestamp))
    return false;
  // Read the host name
  if (!(strStream >> odometryMessage.mHostName))
    return false;

  return true;
}

// Read ground-truth pose message
bool CarmenLoader::ReadTruePos(
  std::istringstream& strStream, CarmenTruePosMessage& truePosMessage)
{
  // TRUEPOS true_x true_y true_theta odom_x odom_y odom_theta

  truePosMessage.mSensorType = CarmenSensorType::TruePos;

  // Read the ground-truth pose
  if (!CarmenReadPose2D(strStream, truePosMessage.mTruePose))
    return false;
  // Read the odometry pose
  if (!CarmenReadPose2D(strStream, truePosMessage.mOdometryPose))
    return false;
  // Read the timestamp
  if (!(strStream >> truePosMessage.mTimestamp))
    return false;
  // Read the host name
  if (!(strStream >> truePosMessage.mHostName))
    return false;

  return true;
}

// Read raw laser message
bool CarmenLoader::ReadRawLaser(
  const std::string& sensorName, std::istringstream& strStream,
  CarmenLaserMessage& rawLaserMessage)
{
  // RAWLASERx laser_type start_angle field_of_view angular_resolution
  // maximum_range accuracy remission_mode
  // num_readings [range_readings] num_remissions [remission values]

  const std::string kRawLaser = "RAWLASER";
  if (!StartsWith(sensorName, kRawLaser))
    return false;
  const int sensorId = std::stoi(sensorName.substr(kRawLaser.size()));
  if (sensorId < 1 || sensorId > 5)
    return false;

  rawLaserMessage.mSensorType = CarmenSensorType::RawLaser;
  rawLaserMessage.mSensorName = kRawLaser;
  rawLaserMessage.mSensorId = sensorId;

  // Read the sensor configuration
  if (!(strStream >> rawLaserMessage.mConfig))
    return false;
  // Read the range values
  if (!CarmenReadRangeValues(strStream, rawLaserMessage.mRanges))
    return false;
  // Read the remission values
  if (!CarmenReadRemissionValues(strStream, rawLaserMessage.mRemissions))
    return false;
  // Read the timestamp
  if (!(strStream >> rawLaserMessage.mTimestamp))
    return false;
  // Read the host name
  if (!(strStream >> rawLaserMessage.mHostName))
    return false;

  return true;
}

// Read robot laser message
bool CarmenLoader::ReadRobotLaser(
  const std::string& sensorName, std::istringstream& strStream,
  CarmenRobotLaserMessage& robotLaserMessage)
{
  // ROBOTLASERx laser_type start_angle field_of_view angular_resolution
  // maximum_range accuracy remission_mode
  // num_readings [range_readings] num_remissions [remission values]
  // laser_pose_x laser_pose_y laser_pose_theta
  // robot_pose_x robot_pose_y robot_pose_theta
  // laser_tv laser_rv forward_safety_dist side_safty_dist turn_axis

  const std::string kRobotLaser = "ROBOTLASER";
  if (!StartsWith(sensorName, kRobotLaser))
    return false;
  const int sensorId = std::stoi(sensorName.substr(kRobotLaser.size()));
  if (sensorId < 1 || sensorId > 5)
    return false;

  robotLaserMessage.mSensorType = CarmenSensorType::RobotLaser;
  robotLaserMessage.mSensorName = kRobotLaser;
  robotLaserMessage.mSensorId = sensorId;

  // Read the sensor configuration
  if (!(strStream >> robotLaserMessage.mConfig))
    return false;
  // Read the range values
  if (!CarmenReadRangeValues(strStream, robotLaserMessage.mRanges))
    return false;
  // Read the remission values
  if (!CarmenReadRemissionValues(strStream, robotLaserMessage.mRemissions))
    return false;
  // Read the laser pose
  if (!CarmenReadPose2D(strStream, robotLaserMessage.mLaserPose))
    return false;
  // Read the robot pose
  if (!CarmenReadPose2D(strStream, robotLaserMessage.mRobotPose))
    return false;
  // Read other fields
  if (!(strStream >> robotLaserMessage.mVelocityTrans))
    return false;
  if (!(strStream >> robotLaserMessage.mVelocityRot))
    return false;
  if (!(strStream >> robotLaserMessage.mForwardSafetyDist))
    return false;
  if (!(strStream >> robotLaserMessage.mSideSafetyDist))
    return false;
  if (!(strStream >> robotLaserMessage.mTurnAxis))
    return false;
  // Read the timestamp
  if (!(strStream >> robotLaserMessage.mTimestamp))
    return false;
  // Read the host name
  if (!(strStream >> robotLaserMessage.mHostName))
    return false;

  return true;
}

// Read old laser message with pose
bool CarmenLoader::ReadOldLaserWithPose(
  const std::string& sensorName, std::istringstream& strStream,
  CarmenRobotLaserMessage& robotLaserMessage)
{
  // FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
  // RLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta

  if (sensorName != "FLASER" && sensorName != "RLASER")
    return false;

  robotLaserMessage.mSensorType = CarmenSensorType::OldLaserWithPose;
  robotLaserMessage.mSensorName = sensorName;
  robotLaserMessage.mSensorId = -1;

  // Read the range values
  if (!CarmenReadRangeValues(strStream, robotLaserMessage.mRanges))
    return false;
  // Read the laser pose
  if (!CarmenReadPose2D(strStream, robotLaserMessage.mLaserPose))
    return false;
  // Read the robot pose
  if (!CarmenReadPose2D(strStream, robotLaserMessage.mRobotPose))
    return false;
  // Read the timestamp
  if (!(strStream >> robotLaserMessage.mTimestamp))
    return false;
  // Read the host name
  if (!(strStream >> robotLaserMessage.mHostName))
    return false;

  // Guess other fields
  robotLaserMessage.mConfig.mLaserType = CarmenLaserType::SICK_LMS;
  robotLaserMessage.mConfig.mStartAngle = -Pi<double> / 2.0;
  robotLaserMessage.mConfig.mFieldOfView =
    GuessFieldOfView(static_cast<int>(robotLaserMessage.mRanges.size()));
  robotLaserMessage.mConfig.mAngularResolution =
    GuessAngleIncrement(static_cast<int>(robotLaserMessage.mRanges.size()));
  robotLaserMessage.mConfig.mMaximumRange = 80.0;
  robotLaserMessage.mConfig.mAccuracy = 0.01;
  robotLaserMessage.mConfig.mRemissionMode =
    CarmenLaserRemissionType::REMISSION_NONE;

  robotLaserMessage.mVelocityTrans = 0.0;
  robotLaserMessage.mVelocityRot = 0.0;
  robotLaserMessage.mForwardSafetyDist = 0.0;
  robotLaserMessage.mSideSafetyDist = 0.0;
  robotLaserMessage.mTurnAxis = 0.0;

  return true;
}

// Read old laser message
bool CarmenLoader::ReadOldLaser(
  const std::string& sensorName, std::istringstream& strStream,
  CarmenLaserMessage& oldLaserMessage)
{
  // LASERx num_readings [range_readings]

  const std::string kLaser = "LASER";
  if (!StartsWith(sensorName, kLaser))
    return false;
  const int sensorId = std::stoi(sensorName.substr(kLaser.size()));
  if (sensorId < 3 || sensorId > 5)
    return false;

  oldLaserMessage.mSensorType = CarmenSensorType::OldLaser;
  oldLaserMessage.mSensorName = kLaser;
  oldLaserMessage.mSensorId = sensorId;

  // Read the range values
  if (!CarmenReadRangeValues(strStream, oldLaserMessage.mRanges))
    return false;
  // Read the timestamp
  if (!(strStream >> oldLaserMessage.mTimestamp))
    return false;
  // Read the host name
  if (!(strStream >> oldLaserMessage.mHostName))
    return false;

  // Guess other fields
  oldLaserMessage.mConfig.mLaserType = CarmenLaserType::SICK_LMS;
  oldLaserMessage.mConfig.mStartAngle = -Pi<double> / 2.0;
  oldLaserMessage.mConfig.mFieldOfView =
    GuessFieldOfView(oldLaserMessage.mRanges.size());
  oldLaserMessage.mConfig.mAngularResolution =
    GuessAngleIncrement(static_cast<int>(oldLaserMessage.mRanges.size()));
  oldLaserMessage.mConfig.mMaximumRange = 80.0;
  oldLaserMessage.mConfig.mAccuracy = 0.01;
  oldLaserMessage.mConfig.mRemissionMode =
    CarmenLaserRemissionType::REMISSION_NONE;

  return true;
}

// Guess the angle increment from the number of range values
float CarmenLoader::GuessAngleIncrement(const int numOfValues)
{
  switch (numOfValues) {
    case 181:
      return Pi<float> / 180.0f;
    case 180:
      return Pi<float> / 180.0f;
    case 361:
      return Pi<float> / 360.0f;
    case 360:
      return Pi<float> / 360.0f;
    case 401:
      return Pi<float> / 720.0f;
    case 400:
      return Pi<float> / 720.0f;
    default:
      return GuessFieldOfView(numOfValues)
             / static_cast<float>(numOfValues - 1);
  }
}

// Guess the field-of-view from the number of range values
float CarmenLoader::GuessFieldOfView(const int numOfValues)
{
  switch (numOfValues) {
    case 181:
      return Pi<float>;
    case 180:
      return Pi<float> * 179.0f / 180.0f;
    case 361:
      return Pi<float>;
    case 360:
      return Pi<float> * 179.5f / 180.0f;
    case 401:
      return Pi<float> * 100.0f / 180.0f;
    case 400:
      return Pi<float> * 99.75f / 180.0f;
    default:
      return Pi<float>;
  }
}
