// Used Windows 10 for the developmen
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include <array>
#include <memory>
#include <iostream>
#include <string>
#include <fstream>

# define TIME_STEP 64

// Defining csv and txt file locations
const std::string motionExecutionLocation = "../../MotionExecution.csv";
const std::string motionPlanLocation = "../../MotionPLan.txt";

const int TIME = 64;
// const int numSensors = 8;
const int numDistSensors = 3;
constexpr double maxWheelSpeed = 6.28;

// Name of default distance sensors (unused)
// const std::array<std::string, numSensors> sensor_names = {
//   "ps0",
//   "ps1",
//   "ps2",
//   "ps3",
//   "ps4",
//   "ps5",
//   "ps6",
//   "ps7",
// };

// Names of additional distance sensors
const std::array<std::string, numDistSensors> distSensorsNames = {
  "dsL",
  "dsF",
  "dsR",
};

using namespace webots;



// Helper Functions
// Obstacle checker for walls
bool checkObstacle(double dist) {
  return dist < 650; //825
}

// Calculates the position the robot will target
std::tuple<int, int, int> calcPosition(int key, int direc, int row, int column) {
  switch (key) {
    // Going straight
    case 'F':
      // Facing north
      if (direc == 0) {
        row--;
      }
      // Facing east
      if (direc == 1) {
        column++;
      }
      // Facing south
      if (direc == 2) {
        row++;
      }
      // Facing west
      if (direc == 3) {
        column--;
      }
      break;
    // Turning left
    case 'L':
      //std::cout << "I am on case L.\n";
      direc = direc -1;
      // If abs(direc) >= 4, reset
      if (direc < 0) {
        direc = direc + 4;
      }
      break;
    case 'R':
      //std::cout << "I am on case R.\n";
      direc++;
      // If abs(direc) >= 4, reset
      if (std::abs(direc > 3)) {
        direc = std::abs(direc) - 4;
      }
      break;
    default:
      break;
  }
  // Returning values for future calculations
  return std::make_tuple(direc, row, column);
}

// Prints the position and heading of the robot 
void printPosition(int stepCounter, int row, int column, char heading, char leftWallResult, char frontWallResult, char rightWallResult) {
  if (stepCounter >= 0 && stepCounter < 10) {
    std::cout << "[z5259006_MTRN4110_PhaseA] Step: 00" << stepCounter << ", Row: " << row << ", Column: " << column << ", Heading: " << heading;
    std::cout << ", Left Wall: " << leftWallResult << ", Front Wall: " << frontWallResult << ", Right Wall: " << rightWallResult << "\n";
  }
  if (stepCounter > 9 && stepCounter < 99) {
    std::cout << "[z5259006_MTRN4110_PhaseA] Step: 0" << stepCounter << ", Row: " << row << ", Column: " << column << ", Heading: " << heading;
    std::cout << ", Left Wall: " << leftWallResult << ", Front Wall: " << frontWallResult << ", Right Wall: " << rightWallResult << "\n";
  }
  if (stepCounter > 99) {
    std::cout << "[z5259006_MTRN4110_PhaseA] Step: " << stepCounter << ", Row: " << row << ", Column: " << column << ", Heading: " << heading;
    std::cout << ", Left Wall: " << leftWallResult << ", Front Wall: " << frontWallResult << ", Right Wall: " << rightWallResult << "\n";
  }
}

// Converts ASCII value pulled from commandString to int for using in direction compass calculations
int converASCIIToInt (int ASCIIValue) {
  int direc {0};
  if (ASCIIValue == 78) {
    direc = 0;
  }
  if (ASCIIValue == 69) {
    direc = 1;
  }
  if (ASCIIValue == 83) {
    direc = 2;
  }
  if (ASCIIValue == 87) {
    direc = 3;
  }
  return direc;
}

// Read in the text file
std::string readText() {
  std::string commandString;
  std::fstream commandFile; // Initiate commands variable as a file
  commandFile.open(motionPlanLocation,std::ios::in); // Opens the file and allows input 
  if (commandFile.is_open()) {
    getline(commandFile, commandString);
    commandFile.close();
  }
  return commandString;
}

// Write heading onto the csv file
std::tuple<char,char,char, char> convertDetectResults (bool isLeftWall, bool isFrontWall, bool isRightWall, int direc) {
  char heading;
  if (direc == 0) {
    heading = 'N';
  }
  if (direc == 1) {
    heading = 'E';
  }
  if (direc == 2) {
    heading = 'S';
  }
  if (direc == 3) {
    heading = 'W';
  }
  char leftWallResult = 'N';
  char rightWallResult = 'N';
  char frontWallResult = 'N';
  if (isLeftWall) {
    leftWallResult = 'Y';
  }
  if (isRightWall) {
    rightWallResult = 'Y';
  }
  if (isFrontWall) {
    frontWallResult = 'Y';
  }
  return std::make_tuple(leftWallResult, frontWallResult, rightWallResult, heading);
}

// IMPORTANT CONSTANTS
// Distance between each cell is 0.165 metres.
// Each wheel needs to roll approximately 8.23 radians to move one cell.
// Using this, we find true radius of wheel as: r = 0.02003843889 m (approximately).
// To turn 360 we turn both wheels to 8.92 radians in different directions.
// This means axle length is around 0.08937143745 m (HIGHLY SUS).

// Main Loop
int main(int argc, char **argv) {
  // Initiates the timestep
  webots::Robot robot;
  //int timeStep = robot.getBasicTimeStep(); // (Not Being Used)

  // Getting the Default PositionSensors (Not being Used)
  // std::array<std::unique_ptr<webots::DistanceSensor>, numSensors> sensors;
  // for (int i = 0; i < numSensors; i++) {
  //   sensors[i] = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor(sensor_names[i]));
  //   sensors[i]->enable(timeStep);
  // }

  // Getting the addition 3 Distance Sensors
  std::array<std::unique_ptr<webots::DistanceSensor>, numDistSensors> distSensors;
  for (int i = 0; i < numDistSensors; i++) {
    distSensors[i] = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor(distSensorsNames[i]));
    distSensors[i]->enable(TIME_STEP);
  }

  // Getting the Motors
  // Note: unique_ptrs are pointers that self destruct when they are not being used
  std::unique_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
  std::unique_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};
  leftMotor->setPosition(0);
  rightMotor->setPosition(0);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);

  // Getting Keyboard for input (Not being used)
  // std::unique_ptr<webots::Keyboard> keyboard{robot.getKeyboard()};
  // keyboard->enable(timeStep);

  // Getting the Inertial Unit (Not being used)
  // std::unique_ptr<webots::InertialUnit> IMU{robot.getInertialUnit("IMU")};
  // IMU->enable(timeStep);

  // Initializing variables
  //double roll {0};
  double leftTarPos {0}; // Where the left wheel is currently at
  double rightTarPos {0}; // Where the right wheel is currently at
  double leftCurPos {0}; // What the left wheel will attempt to move to
  double rightCurPos {0}; // What the right wheel will attempt to move to
  int row {0}; // What row the robot is in
  int column {0}; // What column the robot is in
  int direc {0}; // What direction the robot is heading
  std::string commandString = readText(); // Read in the motion commands
  int key; // What command is given to the robot (called key because keyboard was used in progression check)
  int i {0}; // Used to loop through commandString
  std::fstream csvFile; // Initiates the csv file for ouput
  int stepCounter {0}; // Counts how many actions the robot has performed
  bool isLeftWall; // detects a wall on the left
  bool isFrontWall; // detects a wall ahead
  bool isRightWall; // detects a wall on the right
  char leftWallResult; // Yes or no if it detects a wall on the left
  char rightWallResult; // Yes or no if it detects a wall ahead
  char frontWallResult; // Yes or no if it detects a wall on the right
  char heading; // Converts direc to char 
  std::array <double, 10> leftSensorValues {0,0,0,0,0,0,0,0,0,0}; // Creating array for 10 samples used in moving average calculation
  std::array <double, 10> frontSensorValues {0,0,0,0,0,0,0,0,0,0}; // Creating array for 10 samples used in moving average calculation
  std::array <double, 10> rightSensorValues {0,0,0,0,0,0,0,0,0,0}; // Creating array for 10 samples used in moving average calculation
  double leftAverage {0}; // Average of sensor readings
  double frontAverage {0}; // Average of sensor readings
  double rightAverage {0}; // Average of sensor readings
  double timeStopper {0}; // Variable used in timer loops
  double timeStopperTarget {0}; // Target for timeStopper to reach (acts as end of timer loop)
  int stringLength = commandString.length(); // Finds length of MotionPlan string

  // PROGRAM START

  // Open the csv file
  std::cout << "[z5259006_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt...\n";
  csvFile.open(motionExecutionLocation,std::ios::out);
  std::cout << "[z5259006_MTRN4110_PhaseA] Motion plan: " << commandString << "\n";
  std::cout << "[z5259006_MTRN4110_PhaseA] Executing motion plan...\n";

  // Write the headings
  csvFile<< "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
  
  // Get headings at initial position
  // Getting intial coordinates
  stepCounter = 0;
  row = commandString[0] - '0';
  column = commandString[1] - '0';
  direc = converASCIIToInt(commandString[2]);

  // Get 5 distance sensor values
  timeStopperTarget = timeStopper + 3;
  while (timeStopper < timeStopperTarget)  {
    for (int j = 0; j < 10; j++) {
      robot.step(TIME_STEP);
      leftSensorValues[j] = distSensors[0]->getValue();
      frontSensorValues[j] = distSensors[1]->getValue();
      rightSensorValues[j] = distSensors[2]->getValue();
      timeStopper += (double)TIME_STEP/128;
    }
  }

  leftAverage = (leftSensorValues[7]+leftSensorValues[8]+leftSensorValues[9])/(double)3;
  frontAverage = (frontSensorValues[7]+frontSensorValues[8]+frontSensorValues[9])/(double)3;
  rightAverage = (rightSensorValues[7]+rightSensorValues[8]+rightSensorValues[9])/(double)3;

  // Check if obstacle is within average
  isLeftWall = checkObstacle(leftAverage);
  isFrontWall = checkObstacle(frontAverage);
  isRightWall = checkObstacle(rightAverage);

  // Export the results into the csv file
  leftWallResult = std::get<0>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
  frontWallResult = std::get<1>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
  rightWallResult = std::get<2>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
  heading = std::get<3>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
  csvFile<< stepCounter << "," << row << "," << column << "," << heading << "," << leftWallResult << "," << frontWallResult << "," << rightWallResult << "\n";

  // Print position to console
  printPosition(stepCounter, row, column, heading, leftWallResult, frontWallResult, rightWallResult); 

  // Main loop:
  while (robot.step(TIME_STEP) != -1) {
    if (i == 0) {
      row = commandString[i] - '0';
      i++;
    }
    if (i == 1) {
      column = commandString[i] - '0';
      i++;
    }
    if (i == 2) {
      direc = converASCIIToInt(commandString[i]);
      i++;
    }
    if (i > 2 && i != stringLength+1) {
      switch (commandString[i]) {
        // Forwards
        case 'F':
          key = 'F';
          stepCounter++;

          // Sets the speed the robot will go
          leftMotor->setVelocity(0.75*maxWheelSpeed);
          rightMotor->setVelocity(0.75*maxWheelSpeed);
          // Obtain the current position of the robot
          leftCurPos = leftMotor->getTargetPosition();
          rightCurPos = rightMotor->getTargetPosition();
          // Set the position that we want the robot to go
          leftTarPos = leftCurPos + 8.23; rightTarPos = rightCurPos + 8.23;
          leftMotor->setPosition(leftTarPos);
          rightMotor->setPosition(rightTarPos);

          // Calculate position
          calcPosition(key, direc, row, column);
          direc = std::get<0>(calcPosition(key, direc, row, column));
          row = std::get<1>(calcPosition(key, direc, row, column));
          column = std::get<2>(calcPosition(key, direc, row, column));

          // Pause the robot for wall detection
          timeStopperTarget = timeStopper + 3;
          while (timeStopper < timeStopperTarget) {
            robot.step(TIME_STEP);
            timeStopper += (double)TIME_STEP/1000.0;
          }
          
          // Get 10 distance sensor values
          timeStopperTarget = timeStopper + 3;
          while (timeStopper < timeStopperTarget)  {
            for (int j = 0; j < 10; j++) {
              robot.step(TIME_STEP);
              leftSensorValues[j] = distSensors[0]->getValue();
              frontSensorValues[j] = distSensors[1]->getValue();
              rightSensorValues[j] = distSensors[2]->getValue();
              timeStopper += (double)TIME_STEP/128;
            }
          }

          // Find moving average of sensor readings out of 10 samples using last 3 samples
          leftAverage = (leftSensorValues[7]+leftSensorValues[8]+leftSensorValues[9])/(double)3;
          frontAverage = (frontSensorValues[7]+frontSensorValues[8]+frontSensorValues[9])/(double)3;
          rightAverage = (rightSensorValues[7]+rightSensorValues[8]+rightSensorValues[9])/(double)3;

          // Check if obstacle is within average
          isLeftWall = checkObstacle(leftAverage);
          isFrontWall = checkObstacle(frontAverage);
          isRightWall = checkObstacle(rightAverage);

          // Export the results into the csv file
          leftWallResult = std::get<0>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          frontWallResult = std::get<1>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          rightWallResult = std::get<2>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          heading = std::get<3>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          csvFile<< stepCounter << "," << row << "," << column << "," << heading << "," << leftWallResult << "," << frontWallResult << "," << rightWallResult << "\n";

          // Print position to console
          printPosition(stepCounter, row, column, heading, leftWallResult, frontWallResult, rightWallResult);

          i++;
          break;
        // Turning left
        case 'L':
          key = 'L';
          stepCounter++;

          // Sets the speed the robot will go
          leftMotor->setVelocity(0.15*maxWheelSpeed);
          rightMotor->setVelocity(0.15*maxWheelSpeed);
          // Obtain the current position of the robot
          leftCurPos = leftMotor->getTargetPosition();
          rightCurPos = rightMotor->getTargetPosition();
          // Set the position that we want the robot to go
          leftTarPos = leftCurPos - 2.23; rightTarPos = rightCurPos + 2.23;
          leftMotor->setPosition(leftTarPos);
          rightMotor->setPosition(rightTarPos);

          // Calculate position
          calcPosition(key, direc, row, column);
          direc = std::get<0>(calcPosition(key, direc, row, column));
          row = std::get<1>(calcPosition(key, direc, row, column));
          column = std::get<2>(calcPosition(key, direc, row, column));

          // Pause the robot for wall detection
          timeStopperTarget = timeStopper + 3;
          while (timeStopper < timeStopperTarget) {
            robot.step(TIME_STEP);
            timeStopper += (double)TIME_STEP/1000.0;
          }

          // Get 10 distance sensor values
          timeStopperTarget = timeStopper + 3;
          while (timeStopper < timeStopperTarget) {
            for (int k = 0; k < 10; k++) {
              robot.step(TIME_STEP);
              leftSensorValues[k] = distSensors[0]->getValue();
              frontSensorValues[k] = distSensors[1]->getValue();
              rightSensorValues[k] = distSensors[2]->getValue();
              timeStopper += (double)TIME_STEP/128;
              
            }
          }
          
          // Find moving average of sensor readings out of 10 samples using last 3 samples
          leftAverage = (leftSensorValues[7]+leftSensorValues[8]+leftSensorValues[9])/(double)3;
          frontAverage = (frontSensorValues[7]+frontSensorValues[8]+frontSensorValues[9])/(double)3;
          rightAverage = (rightSensorValues[7]+rightSensorValues[8]+rightSensorValues[9])/(double)3;

          // Check if obstacle is within average
          isLeftWall = checkObstacle(leftAverage);
          isFrontWall = checkObstacle(frontAverage);
          isRightWall = checkObstacle(rightAverage);

          // Export the results into the csv file
          leftWallResult = std::get<0>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          frontWallResult = std::get<1>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          rightWallResult = std::get<2>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          heading = std::get<3>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          csvFile<< stepCounter << "," << row << "," << column << "," << heading << "," << leftWallResult << "," << frontWallResult << "," << rightWallResult << "\n";

          // Print position to console
          printPosition(stepCounter, row, column, heading, leftWallResult, frontWallResult, rightWallResult);

          i++;
          break;
        // Turning right
        case 'R':
          key = 'R';
          stepCounter++;
          
          // Sets the speed the robot will go
          leftMotor->setVelocity(0.15*maxWheelSpeed);
          rightMotor->setVelocity(0.15*maxWheelSpeed);
          // Obtain the current position of the robot
          leftCurPos = leftMotor->getTargetPosition();
          rightCurPos = rightMotor->getTargetPosition();
          // Set the position that we want the robot to go
          leftTarPos = leftCurPos + 2.23; rightTarPos = rightCurPos - 2.23;
          leftMotor->setPosition(leftTarPos);
          rightMotor->setPosition(rightTarPos);

          // Calculate position
          calcPosition(key, direc, row, column);
          direc = std::get<0>(calcPosition(key, direc, row, column));
          row = std::get<1>(calcPosition(key, direc, row, column));
          column = std::get<2>(calcPosition(key, direc, row, column));

          // Pause the robot for wall detection
          timeStopperTarget = timeStopper + 3;
          while (timeStopper < timeStopperTarget) {
            robot.step(TIME_STEP);
            timeStopper += (double)TIME_STEP/1000.0;
          }

          // Get 10 distance sensor values
          timeStopperTarget = timeStopper + 3;
          while (timeStopper < timeStopperTarget) {
            for (int l = 0; l < 10; l++) {
              robot.step(TIME_STEP);
              leftSensorValues[l] = distSensors[0]->getValue();
              frontSensorValues[l] = distSensors[1]->getValue();
              rightSensorValues[l] = distSensors[2]->getValue();
              timeStopper += (double)TIME_STEP/128;
            }
          }
          
          // Find moving average of sensor readings out of 10 samples using last 3 samples
          leftAverage = (leftSensorValues[2]+leftSensorValues[3]+leftSensorValues[4])/(double)4;
          frontAverage = (frontSensorValues[2]+frontSensorValues[3]+frontSensorValues[4])/(double)4;
          rightAverage = (rightSensorValues[2]+rightSensorValues[3]+rightSensorValues[4])/(double)4;

          // Check if obstacle is within average
          isLeftWall = checkObstacle(leftAverage);
          isFrontWall = checkObstacle(frontAverage);
          isRightWall = checkObstacle(rightAverage);

          // Export the results into the csv file
          leftWallResult = std::get<0>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          frontWallResult = std::get<1>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          rightWallResult = std::get<2>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          heading = std::get<3>(convertDetectResults(isLeftWall,isFrontWall,isRightWall, direc));
          csvFile<< stepCounter << "," << row << "," << column << "," << heading << "," << leftWallResult << "," << frontWallResult << "," << rightWallResult << "\n";

          // Print position to console
          printPosition(stepCounter, row, column, heading, leftWallResult, frontWallResult, rightWallResult);

          i++;
          break;
        // Occurs when it reaches the end of the MotionPlan
        default:
          // Close the csv file (this only happens at the end)
          std::cout << "[z5259006_MTRN4110_PhaseA] Motion plan executed!\n";
          csvFile.close();
          i++;
          break;
      }
      
    }
    // If robot reaches the end of the MotionPlan, do nothing
    if (i == stringLength+1) {
      robot.step(TIME_STEP);
    }
  };

  return 0;
}