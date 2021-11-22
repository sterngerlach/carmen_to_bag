
// carmen_laser_types.hpp

#ifndef CARMEN_TO_BAG_CARMEN_LASER_TYPES_HPP
#define CARMEN_TO_BAG_CARMEN_LASER_TYPES_HPP

#include <istream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/variant.hpp>

#include <Eigen/Core>

//
// Supported sensor types
//
enum class CarmenSensorType
{
  Unknown,
  Param,
  Odom,
  TruePos,
  RawLaser,
  RobotLaser,
  OldLaserWithPose,
  OldLaser,
};

//
// Supported laser scanner types
// Taken from src/laser-new/laser_messages.h
//
enum class CarmenLaserType
{
  SICK_LMS = 0,
  SICK_PLS = 1,
  HOKUYO_URG = 2,
  SIMULATED_LASER = 3,
  SICK_S300 = 4,
  UNKNOWN_PROXIMITY_SENSOR = 99,
};

//
// Supported remission value types
// Taken from src/laser-new/laser_messages.h
//
enum class CarmenLaserRemissionType
{
  REMISSION_NONE = 0,
  REMISSION_DIRECT = 1,
  REMISSION_NORMALIZED = 2,
};

//
// Configuration of the laser scanner
// Taken from src/laser-new/laser_messages.h
//
struct CarmenLaserConfig
{
  // Laser scanner type
  CarmenLaserType mLaserType;
  // Start angle
  double mStartAngle;
  // Field of view
  double mFieldOfView;
  // Angular resolution
  double mAngularResolution;
  // Maximum valid range
  double mMaximumRange;
  // Accuracy of the range measurements
  double mAccuracy;
  // Remission value type
  CarmenLaserRemissionType mRemissionMode;
};

//
// Base type for the message
//
struct CarmenMessage
{
  // Virtual destructor
  virtual ~CarmenMessage() = default;

  // Sensor type
  CarmenSensorType mSensorType;
  // Timestamp of the message
  double mTimestamp;
  // Host name
  std::string mHostName;
};

//
// Scan data from the laser scanner (RawLaser or OldLaser)
// Taken from src/laser-new/laser_messages.h
//
struct CarmenLaserMessage : public CarmenMessage
{
  // Laser scanner name
  std::string mSensorName;
  // Laser scanner Id (-1 if not available)
  int mSensorId;
  // Laser scanner configuration
  CarmenLaserConfig mConfig;
  // Range values
  std::vector<float> mRanges;
  // Remission values
  std::vector<float> mRemissions;
};

//
// Scan data from the laser scanner with pose (RobotLaser or OldLaserWithPose)
// Taken from src/robot/robot_messages.h
//
struct CarmenRobotLaserMessage : public CarmenMessage
{
  // Laser scanner name
  std::string mSensorName;
  // Laser scanner Id (-1 if not available)
  int mSensorId;
  // Laser scanner configuration
  CarmenLaserConfig mConfig;
  // Range values
  std::vector<float> mRanges;
  // Remission values
  std::vector<float> mRemissions;
  // Laser scanner pose
  Eigen::Vector3d mLaserPose;
  // Robot pose
  Eigen::Vector3d mRobotPose;
  // Translational velocity
  double mVelocityTrans;
  // Rotational velocity
  double mVelocityRot;
  // Safety distance (X axis)
  double mForwardSafetyDist;
  // Safety distance (Y axis)
  double mSideSafetyDist;
  // Unknown field
  double mTurnAxis;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//
// Odometry data (Odom)
// Taken from src/base/base_messages.h
//
struct CarmenOdometryMessage : public CarmenMessage
{
  // Odometry pose
  Eigen::Vector3d mPose;
  // Translational velocity
  double mVelocityTrans;
  // Rotational velocity
  double mVelocityRot;
  // Acceleration
  double mAcceleration;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//
// Ground-truth pose data (TruePos)
// Taken from src/simulator/simulator_messages.h
//
struct CarmenTruePosMessage : public CarmenMessage
{
  // Ground-truth pose
  Eigen::Vector3d mTruePose;
  // Odometry pose
  Eigen::Vector3d mOdometryPose;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//
// Parameter in the Carmen log
//
struct CarmenParameter
{
  std::string mName;
  boost::variant<int, double, std::string> mValue;

  // Check if the parameter type is int
  inline bool IsInt() const { return this->mValue.which() == 0; }
  // Check if the parameter type is double
  inline bool IsDouble() const { return this->mValue.which() == 1; }
  // Check if the parameter type is std::string
  inline bool IsString() const { return this->mValue.which() == 2; }
};

//
// Type definitions
//
using CarmenParameterPtr = std::shared_ptr<CarmenParameter>;
using CarmenMessagePtr = std::shared_ptr<CarmenMessage>;

// Convert std::string to Carmen sensor type constant
inline CarmenSensorType ToCarmenSensorType(const std::string& type)
{
  if (type == "PARAM")
    return CarmenSensorType::Param;
  else if (type == "ODOM")
    return CarmenSensorType::Odom;
  else if (type == "TRUEPOS")
    return CarmenSensorType::TruePos;
  else if (type == "RAWLASER1" || type == "RAWLASER2" ||
           type == "RAWLASER3" || type == "RAWLASER4" ||
           type == "RAWLASER5")
    return CarmenSensorType::RawLaser;
  else if (type == "ROBOTLASER1" || type == "ROBOTLASER2" ||
           type == "ROBOTLASER3" || type == "ROBOTLASER4" ||
           type == "ROBOTLASER5")
    return CarmenSensorType::RobotLaser;
  else if (type == "FLASER" || type == "RLASER")
    return CarmenSensorType::OldLaserWithPose;
  else if (type == "LASER3" || type == "LASER4" ||
           type == "LASER5")
    return CarmenSensorType::OldLaser;

  return CarmenSensorType::Unknown;
}

// Check if the laser scanner type is valid
inline bool IsCarmenLaserTypeValid(const int laserType)
{
  switch (static_cast<CarmenLaserType>(laserType)) {
    case CarmenLaserType::SICK_LMS:
    case CarmenLaserType::SICK_PLS:
    case CarmenLaserType::HOKUYO_URG:
    case CarmenLaserType::SIMULATED_LASER:
    case CarmenLaserType::SICK_S300:
    case CarmenLaserType::UNKNOWN_PROXIMITY_SENSOR:
      return true;
  }
  return false;
}

// Read the laser scanner type from the input stream
inline std::istream& operator>>(
  std::istream& inStream, CarmenLaserType& laserType)
{
  int value;

  if (!inStream)
    return inStream;
  if (!(inStream >> value))
    return inStream;
  if (!IsCarmenLaserTypeValid(value))
    inStream.setstate(std::ios_base::failbit);
  if (inStream)
    laserType = static_cast<CarmenLaserType>(value);
  return inStream;
}

// Check if the remission value type is valid
inline bool IsCarmenLaserRemissionTypeValid(const int remissionType)
{
  switch (static_cast<CarmenLaserRemissionType>(remissionType)) {
    case CarmenLaserRemissionType::REMISSION_NONE:
    case CarmenLaserRemissionType::REMISSION_DIRECT:
    case CarmenLaserRemissionType::REMISSION_NORMALIZED:
      return true;
  }
  return false;
}

// Read the remission value type from the input stream
inline std::istream& operator>>(
  std::istream& inStream, CarmenLaserRemissionType& remissionType)
{
  int value;

  if (!inStream)
    return inStream;
  if (!(inStream >> value))
    return inStream;
  if (!IsCarmenLaserRemissionTypeValid(value))
    inStream.setstate(std::ios_base::failbit);
  if (inStream)
    remissionType = static_cast<CarmenLaserRemissionType>(value);
  return inStream;
}

// Read the laser scanner configuration from the input stream
inline std::istream& operator>>(
  std::istream& inStream, CarmenLaserConfig& laserConfig)
{
  if (!inStream)
    return inStream;
  if (!(inStream >> laserConfig.mLaserType))
    return inStream;
  if (!(inStream >> laserConfig.mStartAngle))
    return inStream;
  if (!(inStream >> laserConfig.mFieldOfView))
    return inStream;
  if (!(inStream >> laserConfig.mAngularResolution))
    return inStream;
  if (!(inStream >> laserConfig.mMaximumRange))
    return inStream;
  if (!(inStream >> laserConfig.mAccuracy))
    return inStream;
  if (!(inStream >> laserConfig.mRemissionMode))
    return inStream;

  return inStream;
}

// Read the values from the input stream
inline bool CarmenReadValues(
  std::istream& inStream, std::vector<float>& values)
{
  int numOfValues;

  // Read the number of range values
  if (!inStream)
    return false;
  if (!(inStream >> numOfValues))
    return false;
  if (numOfValues < 0)
    return false;

  // Resize the vector
  values.clear();
  values.resize(static_cast<std::size_t>(numOfValues));

  // Read the range values
  for (int i = 0; i < numOfValues; ++i)
    if (!(inStream >> values[i]))
      return false;

  return true;
}

// Read the range values from the input stream
inline bool CarmenReadRangeValues(
  std::istream& inStream, std::vector<float>& ranges)
{
  return CarmenReadValues(inStream, ranges);
}

// Read the remission values from the input stream
inline bool CarmenReadRemissionValues(
  std::istream& inStream, std::vector<float>& remissions)
{
  return CarmenReadValues(inStream, remissions);
}

// Read the 2D pose from the input stream
inline bool CarmenReadPose2D(std::istream& inStream, Eigen::Vector3d& pose)
{
  if (!inStream)
    return false;
  if (!(inStream >> pose[0]))
    return false;
  if (!(inStream >> pose[1]))
    return false;
  if (!(inStream >> pose[2]))
    return false;

  return true;
}

#endif // CARMEN_TO_BAG_CARMEN_LASER_TYPES_HPP
