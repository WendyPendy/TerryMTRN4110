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

const std::array<std::string, num_distance_sensors> distanceSensorNames = {"dsL","dsF","dsR"};

// Helper functions
// Print instructions
void printInstructions() {
  std::cout << "Controls:\n";
  std::cout << "    W: Moves the robot forward one cell.\n";
  std::cout << "    A: Pivots the robot to the left 90 degrees.\n";
  std::cout << "    D: Pivots the robot to the right 90 degrees.\n";
  std::cout << "    P: Print the current explored maze.\n";
  std::cout << "    F: Export the explored maze and close the program.\n";
  std::cout << "Note: Before maze exploration begins, please rotate the robot 90 degrees"
            << " to its left or right to detect any potential walls behind it.\n";
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

// Export 2d map vector to a text file
void exportMap (std::vector<std::string> map) {
  std::fstream file;
  file.open("..\\..\\Map.txt", std::ios::out);
  for (int i = 0; i < int(map.size()); i++) {
    file << map[i] << "\n";
  }
  file.close();
}

// Updates the 2d map vector
std::vector<std::string> updateMap(std::vector<std::string> map, int row, int col, char heading, std::array<bool,3> detectedWalls) {
  std::vector<std::string> tempMap = map;
  int mapRow = 2*row + 1;
  int mapCol = 4*col + 2;
  // Start adding the walls
  // If the robot is facing North
  if (heading == 'N') {
    // Place detected walls onto the map
    // Check the left wall
    if (detectedWalls[0] == true) {
      tempMap[mapRow][mapCol-2] = '|';
    }
    // Check the front wall
    if (detectedWalls[1] == true) {
      tempMap[mapRow-1][mapCol-1] = '-';
      tempMap[mapRow-1][mapCol] = '-';
      tempMap[mapRow-1][mapCol+1] = '-';
    }
    // Check the right wall
    if (detectedWalls[2] == true) {
      tempMap[mapRow][mapCol+2] = '|';
    }
  }
  if (heading == 'E') {
    // Place detected walls onto the map
    // Check the left wall
    if (detectedWalls[0] == true) {
      tempMap[mapRow-1][mapCol-1] = '-';
      tempMap[mapRow-1][mapCol] = '-';
      tempMap[mapRow-1][mapCol+1] = '-';
    }
    // Check the front wall
    if (detectedWalls[1] == true) {
      tempMap[mapRow][mapCol+2] = '|';
    }
    // Check the right wall
    if (detectedWalls[2] == true) {
      tempMap[mapRow+1][mapCol-1] = '-';
      tempMap[mapRow+1][mapCol] = '-';
      tempMap[mapRow+1][mapCol+1] = '-';
    }
  }
  if (heading == 'S') {
    // Place detected walls onto the map
    // Check the left wall
    if (detectedWalls[0] == true) {
      tempMap[mapRow][mapCol+2] = '|';
    }
    // Check the front wall
    if (detectedWalls[1] == true) {
      tempMap[mapRow+1][mapCol-1] = '-';
      tempMap[mapRow+1][mapCol] = '-';
      tempMap[mapRow+1][mapCol+1] = '-';
    }
    // Check the right wall
    if (detectedWalls[2] == true) {
      tempMap[mapRow][mapCol-2] = '|';
    }
  }
  if (heading == 'W') {
    // Place detected walls onto the map
    // Check the left wall
    if (detectedWalls[0] == true) {
      tempMap[mapRow+1][mapCol-1] = '-';
      tempMap[mapRow+1][mapCol] = '-';
      tempMap[mapRow+1][mapCol+1] = '-';
    }
    // Check the front wall
    if (detectedWalls[1] == true) {
      tempMap[mapRow][mapCol-2] = '|';
    }
    // Check the right wall
    if (detectedWalls[2] == true) {
      tempMap[mapRow-1][mapCol-1] = '-';
      tempMap[mapRow-1][mapCol] = '-';
      tempMap[mapRow-1][mapCol+1] = '-';
    }
  }
  return tempMap;
}

// Print 2D String vector (FUNCTION IS PULLED FROM Terry z5259006 Phase B)
void print2DStringVector(std::vector<std::string> vec) {
  for (int i = 0; i < int(vec.size()); i++) {
    std:: cout << "[Robobot] " << vec[i] << "\n";
  }
}

// Obtain the maze template
std::vector<std::string> getMapTemplate() {
  std::string mapString;
  std::vector<std::string> map;
  std::ifstream posFile("..\\..\\mazeTemplate.txt");
  if (posFile.is_open()) {
    while (std::getline(posFile, mapString)) {
    // Output the text from the file
      map.push_back(mapString);
    }
    posFile.close();
  } else {
    std::cerr << "Cannot read file";
  }
  return map;
}
// Wall detection function
std::array<int, 3> detectWalls(webots::Robot &robot, std::array<std::unique_ptr<webots::DistanceSensor>, num_distance_sensors> &distanceSensors) {
  int aveLeft = 0;
  int aveRight = 0;
  int aveFront = 0;
  int sum = 0;
  std::array<int, 3> detectedWalls;
  double currTime = robot.getTime();
  robot.step(1);
  while(robot.getTime() < currTime + 3) {
      robot.step(1);
      aveLeft = distanceSensors[0]->getValue() + aveLeft;
      aveFront = distanceSensors[1]->getValue() + aveFront;
      aveRight = distanceSensors[2]->getValue() + aveRight;
      sum++;
  }
  aveLeft = aveLeft/sum;
  aveFront = aveFront/sum;
  aveRight = aveRight/sum;
  detectedWalls[0] = aveLeft;
  detectedWalls[1] = aveFront;
  detectedWalls[2] = aveRight;
  return detectedWalls;
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

int mod(int a, int b) { 
    return (a%b+b)%b; 
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

// Wall checking
bool checkWall (double distanceValue) {
    if (distanceValue < 850) {
        return true;
    } else {
        return false;
    }
}

// Writes out to console
void writeToConsole(int row, int col,char heading, bool left, bool front, bool right ) {
  std::string leftWall = "N";
  std::string rightWall = "N";
  std::string frontWall = "N";
  if (left == true) {
     leftWall = "Y";
  }
  if (right == true) {
     rightWall = "Y";
  }
  if (front == true) {
     frontWall = "Y";
  }
  
  std::cout<< "[Robobot] Row: " << row << ","<< " Column: "<< col << "," << " Heading: "
  << heading << "," << " Left Wall: "<< leftWall << "," << " Front Wall: "<< frontWall << "," <<" Right Wall: " << rightWall << std::endl;
  return;
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
  //from workshop code
  std::array<std::unique_ptr<webots::DistanceSensor>, num_distance_sensors> distanceSensors;
  for (int i = 0; i < num_distance_sensors; i++) {
      distanceSensors[i] = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor(distanceSensorNames[i]));
      distanceSensors[i]->enable(TIME_STEP);
  }

  // Initialize the keyboard
  int timeStep {static_cast<int>(robot.getBasicTimeStep())};
  std::unique_ptr<webots::Keyboard> keyboard{robot.getKeyboard()};
  keyboard->enable(timeStep);
  
  std::vector<std::string> map = getMapTemplate();
  std::string initialPos;
  int row = 0;
  int col = 0;
  char heading = 'a';
  double prevTime = 0;
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
          // Move the robot
          char convertedChar = 'F';
          auto motorPos = executeMove(convertedChar,leftPos,rightPos,heading,row,col);
          go(*leftMotor, *rightMotor, motorPos.leftMotor, motorPos.rightMotor);

          // Update variables
          leftPos = motorPos.leftMotor;
          rightPos = motorPos.rightMotor;
          row = motorPos.row;
          col = motorPos.col;
          heading = motorPos.bearing;

          // Halt the robot
          prevTime = robot.getTime();
          robot.step(1);
          while (robot.getTime() < prevTime + 8) {
              robot.step(1); 
          }

          // Detect the walls
          std::array<int, 3> detectedWalls = detectWalls (robot, distanceSensors);
          writeToConsole(row,col,heading,checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2]));
          std::array<bool, 3> detectedWallsBool = {checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2])};

          // Update the map
          map = updateMap(map, row, col, heading, detectedWallsBool);
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

          // Halt the robot
          prevTime = robot.getTime();
          robot.step(1);
          while (robot.getTime() < prevTime + 8) {
            robot.step(1); 
          }

          // Detect the walls
          std::array<int, 3> detectedWalls = detectWalls (robot, distanceSensors);
          writeToConsole(row,col,heading,checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2]));
          std::array<bool, 3> detectedWallsBool = {checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2])};
          
          // Update the map
          map = updateMap(map, row, col, heading, detectedWallsBool);
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
          
          // Halt the robot
          prevTime = robot.getTime();
          robot.step(1);
          while (robot.getTime() < prevTime + 8) {
            robot.step(1); 
          }

          // Detect the walls
          std::array<int, 3> detectedWalls = detectWalls (robot, distanceSensors);
          writeToConsole(row,col,heading,checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2]));
          std::array<bool, 3> detectedWallsBool = {checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2])};
          
          // Update the map
          map = updateMap(map, row, col, heading, detectedWallsBool);
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

          // Detect the walls
          std::array<int, 3> detectedWalls = detectWalls (robot, distanceSensors);
          writeToConsole(row,col,heading,checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2]));
          std::array<bool, 3> detectedWallsBool = {checkWall(detectedWalls[0]), checkWall(detectedWalls[1]), checkWall(detectedWalls[2])};
          
          // Update the map
          map = updateMap(map, row, col, heading, detectedWallsBool);
          printInstructions();
          break;
          }
          // Print the current maze
          case 'P':
          {
          print2DStringVector(map);
          break;
          }
          // Finish the program and output the maze into a maze text file
          case 'F':
          {
          exportMap(map);
          exit(0); 
          }
          default:
          {
          char keychar = key;
          std::cout << "[Robobots] That is not a valid key: " << keychar << ".\n";
          break;
          }
        }
      }
  }
  
  return 0;
}