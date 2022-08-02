// z5259006 Terry Perng, I coded this on Windows

#include <webots/Robot.hpp>
#include <array>
#include <string>
#include <fstream>
#include <tuple>
#include <queue>
#include <stack>
#include <algorithm>
#include <map> // Basically dictionaries in python

using namespace webots;
const std::string mapLocation = "../../Map.txt";
const std::string pathLocation = "../../PathPlan.txt";
const std::string outputLocation = "../../Output.txt";

// Helper functions
void writeToText(std::string shortestPath) {
  std::ofstream textFile("../../PathPlan.txt", std::ofstream::out);
  if (textFile.is_open()) {
    textFile << shortestPath;
  }
  textFile.close();
}

// ConverStringToVectorChar This was made because I made a mistake
std::vector<char> convertStringToVectorChar(std::string shortestPath) {
  std::vector<char> shortestPathChar;
  for (int i = 0; i < int(shortestPath.size()); i++) {
    shortestPathChar.push_back(shortestPath[i]);
  }
  return shortestPathChar;
}

// Print nodeDistances map
void printNodeDistances(std::map<char, int> nodeDistances) {
  for (auto itr = nodeDistances.begin(); itr != nodeDistances.end(); ++itr) {
    std::cout << "The current node is " << itr->first << " and it is " << itr->second << " units away from the start.\n" ;
  }
}

// Print pred map
void printPred(std::map<char, std::vector<char>> pred) {
  for (auto itr = pred.begin(); itr != pred.end(); ++itr) {
    std::cout << "The current node is " << itr->first << " and its predecessor node is";
      for (int i = 0; i < int(itr->second.size()); i++) {
        std::cout << " " <<  itr->second[i];
      }
  std::cout << ".\n";
  }
}

// Print coordinate map
void printCoordinates(std::map<char, std::vector<int>> coordinates) {
  for (auto itr = coordinates.begin(); itr != coordinates.end(); ++itr) {
    std::cout << "The current node is " << itr->first << " and its coordinates are";
      for (int i = 0; i < int(itr->second.size()); i++) {
        std::cout << " " <<  itr->second[i];
      }
  std::cout << ".\n";
  }
}

// Print 2D Char Vector (pred)
void print2DCharVector(std::vector<std::vector<char>> vec) {
  for (int i = 0; i < int(vec.size()); i++) {
    for (int j = 0; j < int(vec[i].size()); j++) {
      std::cout << vec[i][j];
    }
    std::cout << "\n";
  }
}

// Print 2D String vector
void print2DStringVector(std::vector<std::string> vec) {
  for (int i = 0; i < int(vec.size()); i++) {
    for (int j = 0; j < int(vec[i].size()); j++) {
      std::cout << vec[i][j];
    }
    std::cout << "\n";
  }
}

// Print out a 1D char vector (path)
void print1DCharVector(std::vector<char> vec) {
  for (int i = 0; i < int(vec.size()); i++) {
    std::cout << vec[i] << ".\n";
  }
}

// Print out a 1D string vector (distances)
void print1DStringVector(std::vector<std::string> vec) {
  for (int i = 0; i < int(vec.size()); i++) {
    std::cout << vec[i] << ".\n";
  }
}

// Add the start node to all possible paths because I cannot fix the error
void addStartNode(std::vector<std::vector<char>> &paths, char start) {
  for (int i = 0; i < int(paths.size()); i++) {
    paths[i].push_back(start);
  }
}

// Convert the tempMapVector into a diagram
void convertToDiagram(std::vector<std::string> nodedMapVector, std::vector<char> path, char start, char trueStart, std::ofstream &output) {
  std::cout << "\n";
  int stepCounter {0}; // Used to count the steps from the target to the 
  char charCounter = 'n';
  char currentNode = 'n'; // Current node of interest
  int firstDigit {0};
  int secondDigit {0};
  int thirdDigit {0};
  std::vector<std::string> originalMap = nodedMapVector;
  // Loop through the path
  for (int i = 0; i < int(path.size()); i++) {
    // For the first element in the path
    currentNode = path[i];
    // Find the currentNode in the nodedMapvector
    for (int k = 0; k < int(nodedMapVector.size()); k++) {
      for (int l = 0; l < int(nodedMapVector[k].size()); l++) {
        if (currentNode == nodedMapVector[k][l]) {
          // If we find the start, just use the 
          if (currentNode == start) {
            nodedMapVector[k][l] = trueStart;
            break;
          }
          // If the counter is single digit, use just the node
          if (stepCounter < 10) {
            charCounter = stepCounter + '0';
            nodedMapVector[k][l] = charCounter;
          }
          if (stepCounter >= 10) {
            firstDigit = ((stepCounter) - (stepCounter)%10)/10 + '0';
            secondDigit = (stepCounter)%10 + '0';
            nodedMapVector[k][l] = firstDigit;
            nodedMapVector[k][l+1] = secondDigit;
          }
          if (stepCounter >= 100) {
            firstDigit = (stepCounter - stepCounter%100)/100 + '0';
            secondDigit = ((stepCounter - firstDigit*100) - (stepCounter - firstDigit*100)%10)/10 + '0';
            thirdDigit = stepCounter - firstDigit*100 - secondDigit*10;
            nodedMapVector[k][l] = firstDigit;
            nodedMapVector[k][l+1] = secondDigit;
            nodedMapVector[k][l+2] = thirdDigit;
          }
          stepCounter++;
        }
      }
    }
  }
  // Once we have added the path, clear all the ASCII nodes
  for (int x = 0; x < int(nodedMapVector.size()); x++) {
    for (int y = 0; y < int(nodedMapVector[x].size()); y++) {
      // Clear any unwanted characters
      if (nodedMapVector[x][y] != '|' && nodedMapVector[x][y] != '-' && nodedMapVector[x][y] != '|'
      && nodedMapVector[x][y] != '^' && nodedMapVector[x][y] != 'v' && nodedMapVector[x][y] != '>'
      && nodedMapVector[x][y] != '<' && nodedMapVector[x][y] != '0' && nodedMapVector[x][y] != '1'
      && nodedMapVector[x][y] != '2' && nodedMapVector[x][y] != '3' && nodedMapVector[x][y] != '4'
      && nodedMapVector[x][y] != '5' && nodedMapVector[x][y] != '6' && nodedMapVector[x][y] != '7'
      && nodedMapVector[x][y] != '8' && nodedMapVector[x][y] != '9' && nodedMapVector[x][y] != ' ') {
        nodedMapVector[x][y] = ' ';
      }
    }
  }
  // Print out the final result
  for (int m = 0; m < int(nodedMapVector.size()); m++) {
    std::cout << "[z5259006_MTRN4110_PhaseB] "<< nodedMapVector[m] << "\n";
    output << "[z5259006_MTRN4110_PhaseB] "<< nodedMapVector[m] << "\n";
  }
  // Reset the noded map vector
  nodedMapVector = originalMap;
  // Reset the counter
  stepCounter = 0;
}

// Assign coordinates to ASCII
void convertToCoordinates(std::vector<std::string> nodedMapVector, std::map<char, std::vector<int>> &coordinates) {
  int horiItr {0};
  int vertItr {0};
  std::vector<int> coordinateVector;
  for (int vert = 1; vert < int(nodedMapVector.size()); vert += 2) { 
    for (int hori = 2; hori< int(nodedMapVector[vert].size()); hori += 4) {
      // NodedMapVector[vert][hori] represents a node as a char variable
      // Add the coordinates as a string
      coordinateVector.push_back(vertItr);
      coordinateVector.push_back(horiItr);
      coordinates.insert(std::pair<char, std::vector<int>> (nodedMapVector[vert][hori], coordinateVector));
      horiItr++;
      coordinateVector.clear();
    }
  horiItr = 0;
  vertItr++;
  }
}

// Find shortest path from directions vector
std::string findShortestPath(std::vector<std::string> directions, int &shortestPathInt) {
  std::string shortestPath;
  std::string newPath;
  int shortestPathCounter {0};
  int newPathCounter {0};
  for (int i = 0; i < int(directions.size()); i++) {
    if (i == 0) {
      shortestPath = directions[i];
      shortestPathInt = 0;
    }
    else {
      newPath = directions[i];
      // If the next path is shorter
      if (int(newPath.size()) < int(shortestPath.size())) {
        // Make the next path the shortest path
        shortestPath = newPath;
        shortestPathInt = i;
      }
      // If they are the same size
      if (int(newPath.size()) == int(shortestPath.size())) {
        // Loop through shortestPath and count how many turns
        for (int j = 0; j < int(shortestPath.size()); j++) {
          if (shortestPath[j] == 'L' || shortestPath[j] == 'R') {
            shortestPathCounter++;
          }
        }
        // Loop through the newPath and count how many turns
        for (int k = 0; k < int(newPath.size()); k++) {
          if (newPath[k] == 'L' || newPath[k] == 'R') {
            newPathCounter++;
          }
        }
        // If the newPath has fewer turns, make it the new shortestPath
        if (newPathCounter < shortestPathCounter) {
          shortestPath = newPath;
          shortestPathInt = i;
        }
      }
    }
  }
  return shortestPath;
}

// Convert paths to directions
void convertToDirection(std::vector<std::vector<char>> paths, char start, char target, char heading, std::vector<std::string> &directions, std::map<char, std::vector<int>> coordinates) {
  std::string singleDirection; // A single string of commands
  char startNode = start;
  char currentNode = start;
  char nextNode = 'n'; // a is a placeholder to initialize the variable
  char startHeading = heading;

  // Loop through all paths
  for (int i = 0; i < int(paths.size()); i++) {
    // Reverse the path
    reverse(paths[i].begin(),paths[i].end());
    // std::cout << "The paths are now: ";
    // for (int j = 0; j < int(paths[i].size()); j++) {
    //   std::cout << paths[i][j];
    // }
    // std::cout << ".\n";
    // Convert coordinates to char to append to singleDirection string
    char vertChar = coordinates[currentNode][0] + '0';
    char horiChar = coordinates[currentNode][1] + '0';
    // Append the vertical coordinate
    singleDirection.push_back(vertChar);
    // Append the horizontal coordinate
    singleDirection.push_back(horiChar);
    // Append the heading
    singleDirection.push_back(heading);
    // Loop through the path vector
    for (int k = 0; k < int(paths[i].size()); k++) {
      // Obtain the current node
      currentNode = paths[i][k];
      // If the currentNode is the target, stop
      if (currentNode == target) {
        break; 
      }
      // Obtain the next node if we are not at the end
      if (k < int(paths[i].size()) - 1) {
        nextNode = paths[i][k+1];
      }
      // If the next y axis coordinate is to the left the current y-axis coordinate
      if (coordinates[nextNode][1]== coordinates[currentNode][1] - 1) {
        if (heading == 'W') { // If heading is north (Where we want to be going) 
          // Go straight
          singleDirection.push_back('F');
        }
        if (heading == 'N') { // If heading is west (not where we want to be going)
          // Turn right
          singleDirection.push_back('L');
          // Go straight
          singleDirection.push_back('F');
          heading = 'W';
        }
        if (heading == 'S') {
          // Turn left
          singleDirection.push_back('R');
          // Go straight
          singleDirection.push_back('F');
          heading = 'W';
        }
      }
      // If the next y axis coordinate is to the rigth of the current y-axis coordinate
      if (coordinates[nextNode][1] == coordinates[currentNode][1] + 1) {
        if (heading == 'E') { // If heading is south (where we want to be going)
          // Go straight
          singleDirection.push_back('F');
        }
        if (heading == 'N') { // If heading is west (not where we want to be going)
          // Turn left
          singleDirection.push_back('R');
          // Go straight
          singleDirection.push_back('F');
          // Set the new direction
          heading = 'E';
        }
        if (heading == 'S') { // If heading is east (not where we want to be going)
          // Turn right
          singleDirection.push_back('L');
          // Go straight
          singleDirection.push_back('F');
          // Set the new direction
          heading = 'E';
        }
      }
      // If the next x coordinate is below the current x coordinate
      if (coordinates[nextNode][0] == coordinates[currentNode][0] + 1) {
        if (heading == 'S') { // If heading is east (where we want to be going)
          // Go straight
          singleDirection.push_back('F');
        }
        if (heading == 'E') { // If heading is north (not where we want to be going)
          // Turn right
          singleDirection.push_back('R');
          // Go straight
          singleDirection.push_back('F');
          // Set the new direction
          heading = 'S';
        }
        if (heading == 'W') { // If heading is south (not where we want to be going)
          // Turn left
          singleDirection.push_back('L');
          // Go straight
          singleDirection.push_back('F');
          // Set the new direction
          heading = 'S';
        }
      }
      // If the next x coordinate is to the north of the current x coordinate
      if (coordinates[nextNode][0] == coordinates[currentNode][0] - 1) {
        if (heading == 'N') { // If heading is west (where we want to be going)
          // Go straight
          singleDirection.push_back('F');
        }
        if (heading == 'E') { // If heading is north (not where we want to be going)
          // Turn left
          singleDirection.push_back('L');
          // Go straight
          singleDirection.push_back('F');
          // Set the new direction
          heading = 'N';
        }
        if (heading == 'W') { // If heading is south (not where we want to be going)
          // Turn right
          singleDirection.push_back('R');
          // Go straight
          singleDirection.push_back('F');
          // Set the new direction
          heading = 'N';
        }
      }
    }
    directions.push_back(singleDirection);
    // Clear the string for later use
    singleDirection.clear();
    // Reset nextNode for later use
    nextNode = 'n';
    // Reset heading for later use
    heading = startHeading;
    // Reset curretNode for later use
    currentNode = startNode;
  }
}

// Generate all possible paths
void generateAllPaths(std::map<char,std::vector<char>> pred, char start, char target, std::vector<std::vector<char>> &paths, std::vector<char> &path) {
  // If we reach the start
  if (target == start) {
    // Add the start to the path (NOTE THIS DOESN'T WORK, ADDED THE NODE OUTSIDE OF THIS FUNCTION)
    // path.push_back(target);
    // Finish the path
    // std::reverse(path.begin(), path.end());
    paths.push_back(path);
    // Stop the function
    return;
  }
  // For the predecessors at the target node
  for (char prev : pred[target]) {
    // Push back the current node
    path.push_back(target);
    // Recursive call the function for all predecessors of the current node
    generateAllPaths(pred, start, prev, paths, path);
    // Pop the current node investigated because we don't need it anymore
    path.pop_back();
  }
}

// Find a path from the pred vector
void findPath(std::vector<std::vector<char>> pred, char start, char target, std::vector<char> &path) {
  // Make the current node the target node
  char currentNode = target;
  // Add the target node to the back
  path.push_back(target);
  // If we are not at the start, find the predecessor node
  while (currentNode != start) {
    for (int i = 0; i < int(pred.size()); i++) {
      if (pred[i][0] == currentNode) {
        // Once we find the target, work backwards
        // Add the predecessor node to the vector
        path.push_back(pred[i][1]);
        // Change the current node to the predecessor node
        currentNode = pred[i][1];
      }
    }
  }
  // Once we are finished, reverse the vector
  reverse(path.begin(),path.end());
}


// Fill in the pred vector with ASCII values
std::vector<std::vector<char>> fillPred(std::vector<std::vector<char>> pred) {
  int u {65};
  for (int i = 0; i < 45; i++) {
    char aChar = u;
    pred[i].push_back(aChar);
    u++;
  }
  return pred;
}

// Read in the Map file
std::vector<std::string> readText(std::vector<std::string> mapVector) {
  std::string result; // The finished output 
  std::string commandString; // Line by line of the text file
  std::fstream commandFile; // Initiate commands variable as a file
  commandFile.open(mapLocation,std::ios::in); // Opens the file and allows input 
  while (std::getline(commandFile, commandString) && !commandString.empty()) { // Need while loop to continually getline
    mapVector.push_back(commandString);
  }
  return mapVector;
}

// Generates the path using BFS
bool addPredecessors(std::vector<std::vector<char>> adjVec, char start, char target, std::map<char, std::vector<char>> &pred, std::map<char, int> &nodeDistances) {
  bool check = false;
  char currentNode = start;
  std::queue<char> queue {};
  std::vector<char> visited{};
  visited.push_back(start);
  queue.push(start);
  int distance {0};
  nodeDistances.insert(std::pair<char,int> (start, distance));
  int currentDistanceInNode {0};
  std::vector<char> predList;
  predList.push_back('n');
  pred.insert(std::pair<char,std::vector<char>> (currentNode, predList));
  predList.clear();
  
  while (!queue.empty()) {
    // Get the current node
    currentNode = queue.front();
    // Pop it from queue
    queue.pop();
    
    // Loop through the adjVec list 
    for (auto i = 0; i < int(adjVec.size()); i++) {
      // Find the current node
      if (adjVec[i][0] == currentNode) {
        // Loop through the adjvec for the current node
        for (int j = 0; j < int(adjVec[i].size()); j++) {
          // See if the adjacent nodes have been visited
          for (int k = 0; k < int(visited.size()); k++) {
            if (visited[k] == adjVec[i][j]) {
              check = true;
            }
          }
          // If it has not been visited, add it to the list
          if (check == false) {
            // Mark adjacent node as visited
            visited.push_back(adjVec[i][j]);
            // Make a predecessor for the next node using the current node
            predList.push_back(currentNode);
            pred.insert(std::pair<char,std::vector<char>> (adjVec[i][j], predList));
            // Clear the predlist for future use
            predList.clear();
            // Find the distance from this node to the source by finding the currentNode in the nodeDistances map and adding one to its distance
            distance = nodeDistances[currentNode] + 1;
            nodeDistances.insert(std::pair<char, int> (adjVec[i][j],distance));
            // Push the next node to be inspected next
            queue.push(adjVec[i][j]);
            if (adjVec[i][j] == target) {
              return true;
            }
          }
          // If it has been visited, move on
          if (check == true) {
            // Find the current value in the distance map
            currentDistanceInNode = nodeDistances[adjVec[i][j]];
            distance = nodeDistances[currentNode] +1;
            // If the path we are on right now is faster than the existing path, make the distance the distance of the current path (wipe the old, longer one)
            if (distance < currentDistanceInNode) {
              // Delete the existing distance
              nodeDistances.erase(adjVec[i][j]);
              // Update the distance
              nodeDistances.insert(std::pair<char, int> (adjVec[i][j],distance));
              // Erase the existing pred list element
              pred.erase(adjVec[i][j]);
              // Update the pred list with new shortest path
              predList.push_back(currentNode);
              pred.insert(std::pair<char,std::vector<char>> (adjVec[i][j], predList));
              // Clear predList for future use
              predList.clear();
            }
            // If the predecessors are the same distance away, add both of them
            if (distance == currentDistanceInNode) {
              // Obtain the current predList in pred
              predList = pred[adjVec[i][j]];
              // Update the predList with the currentNode
              predList.push_back(currentNode);
              // Remove the existing element in pred
              pred.erase(adjVec[i][j]);
              // Update the pred with the new predList
              pred.insert(std::pair<char,std::vector<char>> (adjVec[i][j], predList));
              // Clear predList for future use  
              predList.clear();
            }
            check = false;
          }
        }
      }
    }
  }
  return false;
}

std::vector<std::vector<char>> makeAdjVec(std::vector<std::string> mapVector) {
  // The way this list works is the first element is the node that we are currently looking at
  // additional element is a node that the current node points to
  //std::cout << "I have entered the function.\n";
  std::vector<std::vector<char>> adjVec;
  std::vector<char> currentNodeVector;
  int horLength = mapVector[0].size();
  int verLength = mapVector.size();
  for (int i = 1 ; i < verLength; i += 2) {
    for (int j = 2; j < horLength; j += 4) {
      // Add current node to the adjacency list
      //std::cout << "I am about to add the current node to the adjacency list.\n";
      currentNodeVector.push_back(mapVector[i][j]);
      //std::cout << "I have added the current node to the adjacency list.\n";
      // Add top if top isn't '-' and we are not in the top row
      if (mapVector[i-1][j] != '-' && i-1 != 0) {
        currentNodeVector.push_back(mapVector[i-2][j]);
      }
      // Add bottom if bottom isn't '-' and we are not in the bottom row
      if (mapVector[i+1][j] != '-' && i+1 != 11) {
        currentNodeVector.push_back(mapVector[i+2][j]);
      }
      // Add left if left isn't '|' and we are not in the left most column
      if (mapVector[i][j-2] != '|' && j-2 != 0) {
        currentNodeVector.push_back(mapVector[i][j-4]);
      }
      if (mapVector[i][j+2] != '|' && j-2 != 36) {
        currentNodeVector.push_back(mapVector[i][j+4]);
      }
      adjVec.push_back(currentNodeVector);
      currentNodeVector.clear();
    }
  }
  return adjVec;
}

std::tuple<std::vector<std::string>, char, char, char, char> convertToNodes(std::vector<std::string> mapVector) {
  int horLength = mapVector[0].size();
  int verLength = mapVector.size();
  char start = 'n'; // n is a placeholder to initialize the variable
  char target = 'n'; // n is a placeholder to initialize the variable
  char heading = 'n'; // n is a placeholder to initialize the variable
  char trueStart = 'n'; // n is a placeholder to initialize the variable
  int iter {65}; // Iterates to label to nodes
  for (int i = 1 ; i < verLength; i += 2) { // Vertical component
    for (int j = 2; j < horLength; j += 4) { // Horizontal component
      // Dodge the '^' char because it overlaps with later code
      if (iter == 94) {
        iter++;
      }
      if (mapVector[i][j] == 'v') { // If starting location
        start = iter;
        trueStart = 'v';
        mapVector[i][j] = iter;
        iter++;
        heading = 'S';
      }
      if (mapVector[i][j] == '^') {
        trueStart = '^';
        start = iter;
        mapVector[i][j] = iter;
        iter++;
        heading = 'N';
      }
      if (mapVector[i][j] == '>') {
        trueStart = '>';
        start = iter;
        mapVector[i][j] = iter;
        iter++;
        heading = 'E';
      }
      if (mapVector[i][j] == '<') {
        trueStart = '<';
        start = iter;
        mapVector[i][j] = iter;
        iter++;
        heading = 'W';
      }
      if (mapVector[i][j] == 'x') {
        target = iter;
        mapVector[i][j] = iter;
        iter++;
      }
      if (mapVector[i][j] == ' ') {
        mapVector[i][j] = iter;
        iter++;
      }
    }
  }
  return std::make_tuple(mapVector, start, target, heading, trueStart);
}

int main(int argc, char **argv) {
  Robot *robot = new Robot();

  int timeStep = (int)robot->getBasicTimeStep();

  // Initializing Variables
  char target = 'n'; // n is a placeholder for initialization
  char start = 'n'; // n is a placeholder for initialization
  char heading  = 'n'; // n is a placeholder for initialization
  char trueStart = 'n'; // n is a placeholder to initialize the variable
  std::vector<std::vector<char>> adjVec; // The adjacency list, vector with vectors of chars inside telling you what which nodes are connected to the first element
  std::vector<std::string> mapVector; // 2D vector of the map
  std::vector<std::string> nodedMapVector; // 2d vector of map with nodes (to stop with rewriting in getting start and target chars)
  mapVector = readText(mapVector); // Get a vector of strings of the map
  std::map<char, std::vector<char>> pred; // Predecessor list
  std::map<char, int> nodeDistances; // List of nodes and their distance from start
  std::vector<std::vector<char>> paths; // Where all possible paths are stored
  std::vector<std::string> directions; // Converted paths into directions
  std::vector<char> path; // A singular possible path
  std::map<char, std::vector<int>> coordinates; // Coordinates of each node on a map for use in headings
  std::string shortestPath; // The absolute shortest path (my final answer)
  std::vector<std::string> tempMapVector; // Temporary map vector used to store mapVector and edit in convertToDiagram
  std::vector<char> shortestPathChar;
  int shortestPathInt {0};

  std::cout << "I have started the program.\n";
  // Opens the Output file
  std::ofstream output("../../Output.txt", std::ofstream::out);
  // Adds nodes to the map
  nodedMapVector = std::get<0>(convertToNodes(mapVector));

  // Converts the nodes to coordinates (I SHOULD I HAVE DONE THIS IN THE BEGINNING BUT I IMPLEMENTED LIKE 80% OF MY CODE USING THIS ASCII NODE METHOD AND NOW I JUST REALISED I NEED THE COORDINATES)
  convertToCoordinates(nodedMapVector, coordinates);
  // printCoordinates(coordinates);

  //Prints out the noded maze
  // for (int i = 0; i < int(mapVector.size()); i++) {
  //   std::cout << nodedMapVector[i] << "\n";
  // }

  // Get starting position, target location and heading
  start = std::get<1>(convertToNodes(mapVector));
  target = std::get<2>(convertToNodes(mapVector));
  heading = std::get<3>(convertToNodes(mapVector));
  trueStart = std::get<4>(convertToNodes(mapVector));
  // std::cout << "The start char is " << start << ", the target char is " << target << " and the heading is: " << heading << ".\n";

  // Creates adjacency vector
  adjVec = makeAdjVec(nodedMapVector);

  // Prints out adjacency vector
  // std::cout << "The adjacency vector is: \n";
  // print2DCharVector(adjVec);

  // Generate predecessors and distances between each nodes
  addPredecessors(adjVec, start, target, pred, nodeDistances);
  // printNodeDistances(nodeDistances);

  // Print the elements of the pred vector
  // std::cout << "The elements of the pred are: \n";
  // printPred(pred);

  // Generate all possible paths
  generateAllPaths(pred, start, target, paths, path);
  addStartNode(paths, start);
  // print2DCharVector(paths);

  // Convert the paths to directions
  convertToDirection(paths, start, target, heading, directions, coordinates);
  // print1DStringVector(directions);

  // Calculate the shortest possible path
  shortestPath = findShortestPath(directions, shortestPathInt);
  // std::cout << "The shortest path is: " << shortestPath << " and it's value in the vector is " << shortestPathInt << ".\n"; 

  // Convert the shortest path string to a vector of chars
  shortestPathChar = convertStringToVectorChar(shortestPath);

  // Begin printing everything
  std::cout << "[z5259006_MTRN4110_PhaseB] Reading in map from " << mapLocation << "...\n";
  output << "[z5259006_MTRN4110_PhaseB] Reading in map from " << mapLocation << "...\n";
  // Print out the map
  for (int i = 0; i < int(mapVector.size()); i++) {
    std::cout << "[z5259006_MTRN4110_PhaseB] " << mapVector[i] << "\n";
    output << "[z5259006_MTRN4110_PhaseB] " << mapVector[i] << "\n";
  }
  std::cout << "[z5259006_MTRN4110_PhaseB] Map read in!\n";
  output << "[z5259006_MTRN4110_PhaseB] Map read in!\n";
  std::cout << "[z5259006_MTRN4110_PhaseB] Finding shortest paths...\n";
  output << "[z5259006_MTRN4110_PhaseB] Finding shortest paths...\n";
  // Print all the paths
  for (int x = 0; x < int(paths.size()); x++) {
    std::cout << "[z5259006_MTRN4110_PhaseB] Path - " << x+1 << ":\n";
    output << "[z5259006_MTRN4110_PhaseB] Path - " << x+1 << ":\n";
    convertToDiagram(nodedMapVector, paths[x], start, trueStart, output);
  }
  std::cout << "[z5259006_MTRN4110_PhaseB] " << paths.size() << " shortest paths found!\n";
  output << "[z5259006_MTRN4110_PhaseB] " << paths.size() << " shortest paths found!\n";
  std::cout << "[z5259006_MTRN4110_PhaseB] Finding shortest path with least turns...\n";
  output << "[z5259006_MTRN4110_PhaseB] Finding shortest path with least turns...\n";
  convertToDiagram(nodedMapVector, paths[shortestPathInt], start, trueStart, output);
  std::cout << "[z5259006_MTRN4110_PhaseB] Shortest path with least turns found!\n";
  output << "[z5259006_MTRN4110_PhaseB] Shortest path with least turns found!\n";
  std::cout << "[z5259006_MTRN4110_PhaseB] Path Plan (" << int(shortestPath.size())-3 << " steps): " << shortestPath << "\n";
  output << "[z5259006_MTRN4110_PhaseB] Path Plan (" << int(shortestPath.size())-3 << " steps): " << shortestPath << "\n";
  std::cout << "[z5259006_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt...\n";
  output << "[z5259006_MTRN4110_PhaseB] Writing path plan to ../../PathPlan.txt...\n";
  writeToText(shortestPath);
  std::cout << "[z5259006_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!";
  output << "[z5259006_MTRN4110_PhaseB] Path plan written to ../../PathPlan.txt!";
  output.close();
  
  // NOTE TO SELF CHANGE PLACEHOLDER 'A'TO 
  while (robot->step(timeStep) != -1) {
    
  };

  delete robot;
  return 0;
}
