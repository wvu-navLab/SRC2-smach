#include <ros/ros.h>

#include <task_planning/ForwardSearch.hpp>
#include <task_planning/Types.hpp>
#include <volatile_map/VolatileMap.h>
#include <volatile_map/Volatile.h>

volatile_map::VolatileMap create_volatile_map()
{
  float xmin = -50;
  float xmax = 50;
  float ymin = -50;
  float ymax = 50;

  volatile_map::VolatileMap VolatileMap;

  for (int i = 0; i < 2; i++)
  {
    volatile_map::Volatile vol;

    //position
    geometry_msgs::PointStamped position;

    //time stamp
    position.header.stamp = ros::Time::now();

    //volatile position
    float randx = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float randy = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    position.point.x = xmin + randx * (xmax - xmin);
    position.point.y = ymin + randy * (ymax - ymin);
    position.point.z = 0;
    vol.position = position;

    //volatile type
    if (i % 1 == 1)
      vol.type = "ice";
    if (i % 2 == 1)
      vol.type = "ethane";
    if (i % 3 == 1)
      vol.type = "methanol";
    if (i % 4 == 1)
      vol.type = "ammonia";
    if (i % 5 == 1)
      vol.type = "sulfur_dioxide";

    //add to map
    VolatileMap.vol.push_back(vol);
  }
  ROS_INFO_STREAM(VolatileMap);
  return VolatileMap;
}

std::vector<mac::Robot> create_robots()
{

  float xmin = -50;
  float xmax = 50;
  float ymin = -50;
  float ymax = 50;

  std::vector<mac::Robot> robots;
  //excavators
  mac::Robot robot;

  float randx = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
  float randy = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

  for (int i = 0; i < 2; ++i)
  {
    robot.id = i + 1;
    robot.type = mac::EXCAVATOR;
    robot.time_remaining = 1;
    robot.volatile_index = -1;
    robot.current_task = -1;
    robot.odom.pose.pose.position.x = 0; //xmin + randx * (xmax - xmin);
    robot.odom.pose.pose.position.y = 0; //ymin + randy * (ymax - ymin);
    robot.toggle_sleep = false;

    robots.push_back(robot);
  }
  //haulers
  for (int i = 0; i < 2; ++i)
  {
    robot.id = i + 1;
    robot.type = mac::HAULER;
    robot.time_remaining = 1;
    robot.volatile_index = -1;
    robot.bucket_contents = 0;
    robot.odom.pose.pose.position.x = 0; //xmin + randx * (xmax - xmin);
    robot.odom.pose.pose.position.y = 0; //ymin + randy * (ymax - ymin);
    robot.toggle_sleep = false;

    robots.push_back(robot);
  }

  return robots;
}

void print_joint_action(std::vector<mac::Action> joint_action)
{
  std::cout << "*********JOINT ACTION*********" << std::endl;
  int counter = 0;
  for (const auto &action : joint_action)
  {
    std::cout << "ACTION[" << counter << "]" << std::endl;

    std::cout << "robot_type: ";
    switch (action.robot_type)
    {
    case mac::SCOUT:
      std::cout << "SCOUT" << action.robot_type << std::endl;
      break;
    case mac::EXCAVATOR:
      std::cout << "EXCAVATOR" << action.robot_type << std::endl;
      break;
    case mac::HAULER:
      std::cout << "HAULER" << action.robot_type << std::endl;
      break;
    default:
      std::cout << "Invalid robot type: print_joint_action" << std::endl;
      break;
    }

    std::cout << "objective: " << action.objective.first << ", " << action.objective.second << std::endl;
    std::cout << "id: " << action.id << std::endl;

    if (action.robot_type == mac::EXCAVATOR)
    {
      switch (action.code)
      {
      case (int)mac::ACTION_EXCAVATOR_T::_initialize:
        std::cout << "TASK: EXCAVATOR INITIALIZE" << std::endl;
        break;
      case (int)mac::ACTION_EXCAVATOR_T::_planning:
        std::cout << "TASK: EXCAVATOR PLANNING" << std::endl;
        break;
      case (int)mac::ACTION_EXCAVATOR_T::_traverse:
        std::cout << "TASK: EXCAVATOR TRAVERSE" << std::endl;
        break;
      case (int)mac::ACTION_EXCAVATOR_T::_volatile_handler:
        std::cout << "TASK: EXCAVATOR VOL HANDLER" << std::endl;
        break;
      case (int)mac::ACTION_EXCAVATOR_T::_lost:
        std::cout << "TASK: EXCAVATOR LOST" << std::endl;
        break;
      case (int)mac::ACTION_EXCAVATOR_T::_in_progress:
        std::cout << "TASK: EXCAVATOR IN PROGRESS" << std::endl;
        break;
      }
    }

    if (action.robot_type == mac::HAULER)
    {
      switch (action.code)
      {
      case (int)mac::ACTION_HAULER_T::_initialize:
        std::cout << "TASK: HAULER INITIALIZE" << std::endl;
        break;
      case (int)mac::ACTION_HAULER_T::_planning:
        std::cout << "TASK: HAULER PLANNING" << std::endl;
        break;
      case (int)mac::ACTION_HAULER_T::_traverse:
        std::cout << "TASK: HAULER TRAVERSE" << std::endl;
        break;
      case (int)mac::ACTION_HAULER_T::_volatile_handler:
        std::cout << "TASK: HAULER VOL HANDLER" << std::endl;
        break;
      case (int)mac::ACTION_HAULER_T::_lost:
        std::cout << "TASK: HAULER LOST" << std::endl;
        break;
      case (int)mac::ACTION_HAULER_T::_hauler_dumping:
        std::cout << "TASK: HAULER DUMPING" << std::endl;
        break;
      case (int)mac::ACTION_HAULER_T::_in_progress:
        std::cout << "TASK: HAULER IN PROGRESS" << std::endl;
        break;
      default:
        std::cout << "Invalid code: print_joint_action" << std::endl;
        break;
      }
    }

    std::cout << "volatile_index: " << action.volatile_index << std::endl;
    std::cout << "toggle_sleep: " << action.toggle_sleep << std::endl;

    counter++;
  }
}

void print_sequence_of_joint_actions(std::vector<std::vector<mac::Action>> joint_actions)
{
  if (joint_actions.empty())
  {
    std::cout << "print_sequence_of_joint_actions:sequence of joint actions is empty." << std::endl; 
    return;
  }

  int counter = 0;
  for (auto &joint_action : joint_actions)
  {
    std::cout << "STEP[" << counter << "]" << std::endl;
    print_joint_action(joint_action);
    ++counter;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_forward_search");

  ros::NodeHandle nh("");

  std::cout << "main:: Initializing CostFunction..." << std::endl;
  const mac::CostFunction cf(mac::TIME_VOL_COLLECTED);
  mac::PlanningParams pp;
  pp.max_time = 5;  // seconds
  pp.max_depth = 5; //max depth to construct tree
  pp.timeout = 30;
  pp.wait_time = 30;      //seconds
  pp.max_v_scout = 5;     // m/s
  pp.max_v_excavator = 5; // m/s
  pp.max_v_hauler = 5;    // m/s
  pp.vol_handling_time_scout = 5;
  pp.vol_handling_time_excavator = 5;
  pp.vol_handling_time_hauler = 5;
  pp.homing_time_scout = 5;
  pp.homing_time_excavator = 5;
  pp.homing_time_hauler = 5;
  pp.dumping_time = 5;

  std::cout << "main: Initializing PlanningParams..." << std::endl;
  const mac::PlanningParams pp2 = pp;

  //dummy state
  std::cout << "main: Initializing State..." << std::endl;
  mac::State s;
  s.robots = create_robots();
  s.volatile_map = create_volatile_map();
  s.time = ros::Time::now();

  //run forward search
  std::cout << "main: Initializing ForwardSearch..." << std::endl;
  mac::ForwardSearch fs(cf, pp2);

  std::cout << "main: Running ForwardSearch::plan..." << std::endl;
  std::vector<mac::Action> joint_action;
  joint_action = fs.plan(s);

  std::cout << "main: printing joint action." << std::endl;
  print_joint_action(joint_action);

  std::cout << "main: printing sequence of joint actions." << std::endl;
  std::vector<std::vector<mac::Action>>
  best_seq_of_joint_actions = fs.get_best_sequence_of_joint_actions();
  print_sequence_of_joint_actions(best_seq_of_joint_actions);

  return 0;
}