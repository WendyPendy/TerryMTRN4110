#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <fstream>
#include <vector>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>

constexpr double maxMotorSpeed = 6.28;  // rad/s
constexpr double wheel_radius = 0.0205;  // m
constexpr int num_distance_sensors = 3;
constexpr double obstacleDistance = 90.0;
#define TIME_STEP 64

// Helper functions
// Print instructions
void printInstructions() {
  std::cout << "Controls:\n";
  std::cout << "    W: Moves the robot forward one cell.\n";
  std::cout << "    A: Pivots the robot to the left 90 degrees.\n";
  std::cout << "    D: Pivots the robot to the right 90 degrees.\n";
  std::cout << "    F: Close the program.\n";
}

// Writes out to console
void writeToConsole(int row, int col,char heading) {
  std::cout<< "[Robobot] Row: " << row << ","<< " Column: "<< col << "," << " Heading: "
  << heading << std::endl;
  return;
}

int mod(int a, int b) { 
    return (a%b+b)%b; 
}

// Obtain the initial position and heading
std::string getInitialPos() {
  std::string initialPos;
  std::ifstream posFile("..\\..\\InitialPosition.txt");
  if (posFile.is_open()) {
    std::getline(posFile, initialPos);
    posFile.close();
  } else {
    std::cerr << "Cannot read file";
  }
  return initialPos;
}

// Movement function
auto go(webots::Motor &leftMotor,webots::Motor &rightMotor, double leftPos, double rightPos) {
  // Sets the speed the robot will go
  leftMotor.setVelocity(0.15*maxMotorSpeed);
  rightMotor.setVelocity(0.15*maxMotorSpeed);
  // Sets the position the robot will go
  leftMotor.setPosition(leftPos);
  rightMotor.setPosition(rightPos);
}

auto executeMove(char command,double leftMotor,double rightMotor, char bearing, int row, int col) {
  //returns position for motors and updates rows, cols and heading
      int bearingINT;
      
      switch(bearing) {
          case 'N':
              bearingINT = 0;
              break;
          case 'E':
              bearingINT = 1;
              break;
          case 'S':
              bearingINT  = 2;
              break;
          case 'W':
              bearingINT = 3;
              break;
      }
  //forward command
    if ( command == 'F') {
       leftMotor = leftMotor + 2.62586*M_PI;
       rightMotor = rightMotor + 2.62586*M_PI;
      switch(bearing) {
          case 'N':
              row = row - 1;
              break;
          case 'E':
              col = col + 1;
              break;
          case 'S':
              row  = row + 1;
              break;
          case 'W':
              col = col - 1;
              break;
      }
  
    }
    //left command
    else if (command == 'L') {
        leftMotor = leftMotor - 0.711175*M_PI;
        rightMotor = rightMotor +0.711175*M_PI;
        bearingINT = mod((bearingINT - 1), 4);
    }
    //right command
    else if (command == 'R') {
        leftMotor = leftMotor + 0.711175*M_PI;
        rightMotor = rightMotor - 0.711175*M_PI;
        bearingINT = mod((bearingINT + 1), 4);
    }
    switch(bearingINT) {
          case 0:
              bearing = 'N';
              break;
          case 1:
              bearing = 'E';
              break;
          case 2:
              bearing  = 'S';
              break;
          case 3:
              bearing = 'W';
              break;
      }
    struct motorPos {double leftMotor; double rightMotor; int row; int col; char bearing;};

    return motorPos{leftMotor,rightMotor,row,col,bearing};
  
}

int main() {
  // Initialise robot.
  webots::Robot robot;
  // Get pointers to our robot's motors.
  std::unique_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
  std::unique_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  leftMotor->setPosition(0);
  rightMotor->setPosition(0);

  // Initialize the keyboard
  int timeStep {static_cast<int>(robot.getBasicTimeStep())};
  std::unique_ptr<webots::Keyboard> keyboard{robot.getKeyboard()};
  keyboard->enable(timeStep);
  
  std::string initialPos;
  int row = 0;
  int col = 0;
  char heading = 'a';
  double leftPos = 0;
  double rightPos = 0;
  int key {0};
  int prevKey {0};

  std::cout << "Please input initial position in InitialPosition.txt and press R to continue.\n";
  while (robot.step(TIME_STEP) != -1) {
      prevKey = key;
      key = keyboard->getKey();
      if (prevKey != key && key != -1) {
        switch(key) {
          // Forwards movement
          case 'W':
          {
          // Move the robot forwards
          char convertedChar = 'F';
          auto motorPos = executeMove(convertedChar,leftPos,rightPos,heading,row,col);
          go(*leftMotor, *rightMotor, motorPos.leftMotor, motorPos.rightMotor);

          // Update variables
          leftPos = motorPos.leftMotor;
          rightPos = motorPos.rightMotor;
          row = motorPos.row;
          col = motorPos.col;
          heading = motorPos.bearing;

          // Write to console
          writeToConsole(row, col, heading);

          break;
          }
          // Turn left
          case 'A':
          {
          // Move the robot
          char convertedChar = 'L';
          auto motorPos = executeMove(convertedChar,leftPos,rightPos,heading,row,col);
          go(*leftMotor, *rightMotor, motorPos.leftMotor, motorPos.rightMotor);

          // Update variables
          leftPos = motorPos.leftMotor;
          rightPos = motorPos.rightMotor;
          row = motorPos.row;
          col = motorPos.col;
          heading = motorPos.bearing;

          // Write to console
          writeToConsole(row, col, heading);

          break;
          }
          // Turn right
          case 'D':
          {
          // Move the robot
          char convertedChar = 'R';
          auto motorPos = executeMove(convertedChar,leftPos,rightPos,heading,row,col);
          go(*leftMotor, *rightMotor, motorPos.leftMotor, motorPos.rightMotor);

          // Update variables
          leftPos = motorPos.leftMotor;
          rightPos = motorPos.rightMotor;
          row = motorPos.row;
          col = motorPos.col;
          heading = motorPos.bearing;

          // Write to console
          writeToConsole(row, col, heading);
          
          break;
          }
          // Read in the initial position text file
          case 'R':
          {
          initialPos = getInitialPos();
          row = initialPos[0] - '0';
          col = initialPos[1] - '0';
          heading = initialPos[2];
          std::cout << "[Robobot] Initial position read in!.\n";
          // Write to console
          writeToConsole(row, col, heading);
          printInstructions();
          break;
          }
          // Finish the program and output the maze into a maze text file
          case 'F':
          {
          exit(0);
          }
          // Default case when inappropriate key is pressed
          default:
          {
          char keychar = key;
          std::cout << "[Robobot] That is not a valid key: " << keychar << ".\n";
          break;
          }
        }
      }
  }
  
  return 0;
}