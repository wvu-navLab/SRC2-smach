#include <ros/ros.h>

#include <task_planning/ForwardSearch.hpp>
#include <task_planning/Types.hpp>
#include <volatile_map/VolatileMap.h>
#include <volatile_map/Volatile.h>
#include <geometry_msgs/PointStamped.h>

volatile_map::VolatileMap create_volatile_map()
{
  float xmin = -50;
  float xmax = 50;
  float ymin = -50;
  float ymax = 50;

  volatile_map::VolatileMap VolatileMap;

  for (int i = 0; i < 10; i++)
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
    vol.collected = false;
    vol.dumped = false;

    //volatile type
    if (i % 1 == 0)
      vol.type = "ice";
    if (i % 2 == 0)
      vol.type = "ethane";
    if (i % 3 == 0)
      vol.type = "methanol";
    if (i % 4 == 0)
      vol.type = "ammonia";
    if (i % 5 == 0)
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
    robot.time_remaining = 0.0;
    robot.volatile_index = -1;
    robot.current_task = -1;
    robot.odom.pose.pose.position.x = 0; //xmin + randx * (xmax - xmin);
    robot.odom.pose.pose.position.y = 0; //ymin + randy * (ymax - ymin);
    geometry_msgs::PointStamped temp_plan;
    temp_plan.point.x = robot.odom.pose.pose.position.x;
    temp_plan.point.y = robot.odom.pose.pose.position.y;
    robot.plan.push_back(temp_plan);
    robot.toggle_sleep = false;

    robots.push_back(robot);
  }
  //haulers
  for (int i = 0; i < 2; ++i)
  {
    robot.id = i + 1;
    robot.type = mac::HAULER;
    robot.time_remaining = 0.0;
    robot.volatile_index = -1;
    robot.current_task = -1;
    robot.bucket_contents = 0;
    robot.odom.pose.pose.position.x = 0; //xmin + randx * (xmax - xmin);
    robot.odom.pose.pose.position.y = 0; //ymin + randy * (ymax - ymin);
    geometry_msgs::PointStamped temp_plan;
    temp_plan.point.x = robot.odom.pose.pose.position.x;
    temp_plan.point.y = robot.odom.pose.pose.position.y;
    robot.plan.push_back(temp_plan);
    robot.toggle_sleep = false;

    robots.push_back(robot);
  }

  return robots;
}



void test_propagate(mac::State s, std::vector<mac::Action> joint_action, mac::ForwardSearch fs)
{
  std::cout << "*****************************" << std::endl;
  std::cout << "****Testing Propagate********" << std::endl;
  std::cout << "*****************************" << std::endl;
  std::cout << "Current State:" << std::endl;
  fs.print_state(s);

  std::cout << "Joint Action:" << std::endl;

  std::cout << "Running propagate..." << std::endl;
  fs.print_joint_action(joint_action);

  mac::State sp = fs.propagate(s, joint_action);

  std::cout << "Next State:" << std::endl;
  fs.print_state(s);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_forward_search");

  ros::NodeHandle nh("");

  std::cout << "main:: Initializing CostFunction..." << std::endl;
  const mac::CostFunction cf(mac::TIME_VOL_COLLECTED);
  mac::PlanningParams pp;
  pp.max_time = 15;  // seconds
  pp.max_depth = 50; //max depth to construct tree
  pp.timeout = 30;
  pp.wait_time = 30;      //seconds
  pp.max_v_scout = 5;     // m/s
  pp.max_v_excavator = 5; // m/s
  pp.max_v_hauler = 5;    // m/s
  pp.vol_handling_time_scout = 5;
  pp.vol_handling_time_excavator = 145;
  pp.vol_handling_time_hauler = 150;
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

  //test propagate
  std::vector<std::vector<mac::Action>> jas = fs.get_actions_all_robots(s);
  std::vector<mac::Action> ja = jas[0];
  //test_propagate(s, ja, fs);
  // for (auto & ja: jas)
  // {
  //   std::cout << "//////////////" << std::endl;
  //   print_joint_action(ja);
  // }
  // std::cout << "duh saiz " << jas.size() << std::endl;

  //test forward search
  std::cout << "main: Running ForwardSearch::plan..." << std::endl;
  std::vector<mac::Action> joint_action;
  joint_action = fs.plan(s);
  std::cout << "check" << std::endl;
   fs.print_joint_action(joint_action);
   std::cout << "end" << std::endl;
  //print_volatile_map(s.volatile_map);

  // std::cout << "main: printing joint action." << std::endl;
  // print_joint_action(joint_action);

  // std::cout << "main: printing sequence of joint actions." << std::endl;
  // std::vector<std::vector<mac::Action>>
  // best_seq_of_joint_actions = fs.get_best_sequence_of_joint_actions();
  // print_sequence_of_joint_actions(best_seq_of_joint_actions);

  //-> we need to check if the propagate outputs make sense
  //-> then, make sure get_policy works properly
  //-> in get_actions_all_robots(const State &s), if only a single volatile is being pursued by excavator, then the only joint action is both haulers go to help, need to allow for either of the haulers to go
  //-> check possibility that if excavator collects before hauler is sent, that the hauler should still be sent.
  //-> 

  // forget about toggle. we're not doing it. we're just not. 

  return 0;
}