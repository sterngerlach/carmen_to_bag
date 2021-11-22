
// carmen_loader.hpp

#ifndef CARMEN_TO_BAG_CARMEN_LOADER_HPP
#define CARMEN_TO_BAG_CARMEN_LOADER_HPP

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/version.hpp>

#include <ros/ros.h>

#include "carmen_to_bag/carmen_laser_types.hpp"

struct CarmenLog
{
  // Find a parameter with the given name
  std::vector<CarmenParameterPtr>::const_iterator FindParameter(
    const std::string& paramName) const;
  // Find a message with the given message type
  std::vector<CarmenMessagePtr>::const_iterator FindMessage(
    const CarmenSensorType sensorType) const;
  // Find a message with the given sensor name
  std::vector<CarmenMessagePtr>::const_iterator FindMessage(
    const std::string& sensorName, const int sensorId = -1) const;

  // Check if a parameter with the given name exists
  inline bool ParameterExists(const std::string& paramName) const
  { return this->FindParameter(paramName) != this->mParameters.end(); }
  // Check if a message with the given message type exists
  inline bool MessageExists(const CarmenSensorType sensorType) const
  { return this->FindMessage(sensorType) != this->mMessages.end(); }
  // Check if a message with the given sensor name exists
  inline bool MessageExists(const std::string& sensorName,
                            const int sensorId = -1) const
  { return this->FindMessage(sensorName, sensorId) != this->mMessages.end(); }

  // Parameters
  std::vector<CarmenParameterPtr> mParameters;
  // Messages
  std::vector<CarmenMessagePtr> mMessages;
};

class CarmenLoader final
{
public:
  // Constructor
  CarmenLoader() = default;
  // Destructor
  ~CarmenLoader() = default;

  // Load Carmen log
  bool Load(const std::string& carmenFileName, CarmenLog& carmenLog);

  // Copy constructor (disabled)
  CarmenLoader(const CarmenLoader&) = delete;
  // Copy assignment operator (disabled)
  CarmenLoader& operator=(const CarmenLoader&) = delete;
  // Move constructor (disabled)
  CarmenLoader(CarmenLoader&&) = delete;
  // Move assignment operator (disabled)
  CarmenLoader& operator=(CarmenLoader&&) = delete;

  // Read parameter
  bool ReadParameter(std::istringstream& strStream,
                     CarmenParameter& parameter);
  // Read odometry message
  bool ReadOdometry(std::istringstream& strStream,
                    CarmenOdometryMessage& odometryMessage);
  // Read ground-truth pose message
  bool ReadTruePos(std::istringstream& strStream,
                   CarmenTruePosMessage& truePosMessage);
  // Read raw laser message
  bool ReadRawLaser(const std::string& sensorName,
                    std::istringstream& strStream,
                    CarmenLaserMessage& rawLaserMessage);
  // Read robot laser message
  bool ReadRobotLaser(const std::string& sensorName,
                      std::istringstream& strStream,
                      CarmenRobotLaserMessage& robotLaserMessage);
  // Read old laser message with pose
  bool ReadOldLaserWithPose(const std::string& sensorName,
                            std::istringstream& strStream,
                            CarmenRobotLaserMessage& robotLaserMessage);
  // Read old laser message
  bool ReadOldLaser(const std::string& sensorName,
                    std::istringstream& strStream,
                    CarmenLaserMessage& oldLaserMessage);

  // Guess the angle increment from the number of range values
  static float GuessAngleIncrement(const int numOfValues);
  // Guess the field-of-view from the number of range values
  static float GuessFieldOfView(const int numOfValues);
};

#endif // CARMEN_TO_BAG_CARMEN_LOADER_HPP
