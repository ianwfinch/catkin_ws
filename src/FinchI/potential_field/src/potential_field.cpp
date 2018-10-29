#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <LinearMath/btQuaternion.h> // Needed to convert rotation ...
#include <LinearMath/btMatrix3x3.h>  // ... quaternion into Euler angles


struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t; // last received time
  
  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};
  
  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    btQuaternion q = btQuaternion(msg->pose.pose.orientation.x, \
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, \
      msg->pose.pose.orientation.w);
    btMatrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
  };
};


class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh, int robotID, int n, \
      double gx, double gy) : ID(robotID), numRobots(n), \
      goalX(gx), goalY(gy) {
    // Initialize random time generator
    srand(time(NULL));

    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, \
      &PotFieldBot::laserCallback, this);
    
    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    for (int i = 0; i < numRobots; i++) {
      pose.push_back(Pose());
    }
    for (int i = 0; i < numRobots; i++) {
      poseSubs.push_back(nh.subscribe("/robot_" + \
        boost::lexical_cast<std::string>(i) + \
        "/base_pose_ground_truth", 1, \
        &Pose::poseCallback, &pose[i]));
    }
  };


  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // TODO: parse laser data
    // (see http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html)













  };
  
  
  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
      // TODO: remove demo code, compute potential function, actuate robot
        //user defined
        float myLambda;
        float myAlpha;
        float myBeta;
        float myEpsilon;
        //temporary function length until i find out what it is
        //also change Fattr and Frepuls to arrays
        float Fattr = lambda*(length(Xgoal-Xrobot))^2;
        float Frepuls;
        if (di < dsafe+myEpsilon) {
            Frepuls = myAlpha/((di-dsafe)^2);
        } else if (di > dsafe+myAlpha && di < myBeta) {
            Frepuls = myAlpha/(myEpsilon^2);
        } else {
            Frepuls = 0;
        }

        //vector summation









    
      /* Demo code: print each robot's pose
      for (int i = 0; i < numRobots; i++) {
        std::cout << std::endl;
        std::cout << i << "        ";
        std::cout << "Pose: " << pose[i].x << ", " << pose[i].y << ", " << pose[i].heading << std::endl;
      }*/

      ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
      rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
  };

  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI/2;


protected:
  ros::Publisher commandPub; // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the current robot's laser scan topic
  std::vector<ros::Subscriber> poseSubs; // List of subscribers to all robots' pose topics
  std::vector<Pose> pose; // List of pose objects for all robots
  int ID; // 0-indexed robot ID
  int numRobots; // Number of robots, positive value
  double goalX, goalY; // Coordinates of goal
};


int main(int argc, char **argv) {
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;
  
  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    std::cout << "Usage: " << argv[0] << " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]" << std::endl;
    return EXIT_FAILURE;
  }
  
  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1])); // Initiate ROS node
  ros::NodeHandle n("robot_" + std::string(argv[1])); // Create named handle "robot_#"
  PotFieldBot robbie(n, robotID, numRobots, goalX, goalY); // Create new random walk object
  robbie.spin(); // Execute FSM loop

  return EXIT_SUCCESS;
};