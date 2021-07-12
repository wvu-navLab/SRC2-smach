#include <task_planning/ForwardSearch.hpp>

namespace mac
{
  const char *ACTION_HAULER_T_STRINGS[] =
      {
          "_initialize",
          "_planning",
          "_traverse",
          "_volatile_handler",
          "_lost",
          "_hauler_dumping",
          "_in_progress"};

  const char *ACTION_EXCAVATOR_T_STRINGS[] =
      {
          "_initialize",
          "_planning",
          "_traverse",
          "_volatile_handler",
          "_lost",
          "_in_progress"};

  const char *ACTION_SCOUT_T_STRINGS[] =
      {
          "_initialize",
          "_planning",
          "_traverse",
          "_volatile_handler",
          "_lost",
          "_in_progress"};

  //////////////////////////////////////////////////
  ///////////////////////MATH///////////////////////
  //////////////////////////////////////////////////
  int ForwardSearch::get_robot_index(const State &s, int robot_type, int robot_id)
  {
    int index = -1;
    for (int i = 0; i < s.robots.size(); ++i)
    {
      if (s.robots[i].type == robot_type && s.robots[i].id == robot_id)
      {
        index = i;
      }
    }
    return index;
  }

  //////////////////////////////////////////////////
  ///////////////////////MATH///////////////////////
  //////////////////////////////////////////////////
  double ForwardSearch::dist(const std::vector<double> p1, const std::vector<double> p2)
  {
    if (p1.size() != p2.size())
    {
      std::cout << "Error! p1.size() != p2.size() for computing distance!\n";
      exit(1); //TODO: remove exit
    }
    double val = 0;
    for (int i = 0; i < p1.size(); i++)
    {
      double diff = p1[i] - p2[i];
      val = val + diff * diff;
    }
    val = std::sqrt(val);
    return val;
  }

  std::vector<std::vector<int>> ForwardSearch::cartesian_product(const std::vector<std::vector<int>> &v)
  {
    // std::cout << "input to cartesian_product:" << std::endl;
    // for (int j = 0; j < v.size(); j++)
    // {
    //   std::cout << "v[" << j << "]:";
    //   for (int i = 0; i < v[j].size(); i++)
    //   {
    //     std::cout << v[j][i] << " ";
    //   }
    //   std::cout << std::endl;
    // }

    std::vector<std::vector<int>> s = {{}};
    for (const auto &u : v)
    {
      std::vector<std::vector<int>> r;
      for (const auto &x : s)
      {
        for (const auto &y : u)
        {
          r.push_back(x);
          r.back().push_back(y);
        }
      }
      s = std::move(r);
    }
    return s;
  }

  ////////////////////////////////////////////////////
  ///////////////////////PRINTS///////////////////////
  ////////////////////////////////////////////////////

  void ForwardSearch::print_joint_action(std::vector<mac::Action> joint_action)
  {
    int counter = 0;
    for (const auto &action : joint_action)
    {
      std::cout << "ACTION[" << counter << "]" << std::endl;

      std::cout << "    robot_type: ";
      switch (action.robot_type)
      {
      case mac::SCOUT:
        std::cout << "SCOUT" << action.id << std::endl;
        break;
      case mac::EXCAVATOR:
        std::cout << "EXCAVATOR" << action.id << std::endl;
        break;
      case mac::HAULER:
        std::cout << "HAULER" << action.id << std::endl;
        break;
      default:
        std::cout << "Invalid robot type: print_joint_action" << std::endl;
        break;
      }

      std::cout << "    objective: " << action.objective.first << ", " << action.objective.second << std::endl;
      std::cout << "    id: " << action.id << std::endl;

      if (action.robot_type == mac::EXCAVATOR)
      {
        switch (action.code)
        {
        case (int)mac::ACTION_EXCAVATOR_T::_initialize:
          std::cout << "    TASK: EXCAVATOR INITIALIZE" << std::endl;
          break;
        case (int)mac::ACTION_EXCAVATOR_T::_planning:
          std::cout << "    TASK: EXCAVATOR PLANNING" << std::endl;
          break;
        case (int)mac::ACTION_EXCAVATOR_T::_traverse:
          std::cout << "    TASK: EXCAVATOR TRAVERSE" << std::endl;
          break;
        case (int)mac::ACTION_EXCAVATOR_T::_volatile_handler:
          std::cout << "    TASK: EXCAVATOR VOL HANDLER" << std::endl;
          break;
        case (int)mac::ACTION_EXCAVATOR_T::_lost:
          std::cout << "    TASK: EXCAVATOR LOST" << std::endl;
          break;
        case (int)mac::ACTION_EXCAVATOR_T::_in_progress:
          std::cout << "    TASK: EXCAVATOR IN PROGRESS" << std::endl;
          break;
        }
      }

      if (action.robot_type == mac::HAULER)
      {
        switch (action.code)
        {
        case (int)mac::ACTION_HAULER_T::_initialize:
          std::cout << "     TASK: HAULER INITIALIZE" << std::endl;
          break;
        case (int)mac::ACTION_HAULER_T::_planning:
          std::cout << "     TASK: HAULER PLANNING" << std::endl;
          break;
        case (int)mac::ACTION_HAULER_T::_traverse:
          std::cout << "     TASK: HAULER TRAVERSE" << std::endl;
          break;
        case (int)mac::ACTION_HAULER_T::_volatile_handler:
          std::cout << "     TASK: HAULER VOL HANDLER" << std::endl;
          break;
        case (int)mac::ACTION_HAULER_T::_lost:
          std::cout << "     TASK: HAULER LOST" << std::endl;
          break;
        case (int)mac::ACTION_HAULER_T::_hauler_dumping:
          std::cout << "     TASK: HAULER DUMPING" << std::endl;
          break;
        case (int)mac::ACTION_HAULER_T::_in_progress:
          std::cout << "     TASK: HAULER IN PROGRESS" << std::endl;
          break;
        default:
          std::cout << "Invalid code: print_joint_action" << std::endl;
          break;
        }
      }

      std::cout << "    volatile_index: " << action.volatile_index << std::endl;
      std::cout << "    toggle_sleep: " << action.toggle_sleep << std::endl;

      counter++;
    }
  }

  void ForwardSearch::print_sequence_of_joint_actions(std::vector<std::vector<mac::Action>> joint_actions)
  {
    if (joint_actions.empty())
    {
      std::cout << "print_sequence_of_joint_actions:sequence of joint actions is empty." << std::endl;
      return;
    }

    int counter = 0;
    for (auto &joint_action : joint_actions)
    {
      std::cout << "JOINT ACTION[" << counter << "]" << std::endl;
      print_joint_action(joint_action);
      ++counter;
    }
  }

  void ForwardSearch::print_volatile_map(volatile_map::VolatileMap volatile_map)
  {
    std::cout << "volatile_map:" << std::endl;
    int counter = 0;
    for (auto &v : volatile_map.vol)
    {

      std::cout << "  vol " << counter;
      std::cout << ": type=" << v.type;
      std::cout << ", x=" << v.position.point.x;
      std::cout << ", y=" << v.position.point.y;
      std::cout << ", collected=" << (int)v.collected;
      std::cout << ", duh-dumped=" << (int)v.dumped;
      std::cout << std::endl;
      ++counter;
    }
  }

  void ForwardSearch::print_robot(int robot_index, std::vector<mac::Robot> robots)
  {
    //robot
    mac::Robot robot = robots[robot_index];

    //map robot type and current task to strings for printing
    std::string robot_type;
    std::string current_task;
    switch (robot.type)
    {
    case mac::SCOUT:
      robot_type = "SCOUT";
      if (robot.current_task != -1)
      {
        current_task = mac::ACTION_SCOUT_T_STRINGS[robot.current_task];
      }
      else
      {
        current_task = "none";
      }
      break;
    case mac::EXCAVATOR:
      robot_type = "EXCAVATOR";
      if (robot.current_task != -1)
      {
        current_task = mac::ACTION_EXCAVATOR_T_STRINGS[robot.current_task];
      }
      else
      {
        current_task = "none";
      }
      break;
    case mac::HAULER:
      robot_type = "HAULER";
      if (robot.current_task != -1)
      {
        current_task = mac::ACTION_HAULER_T_STRINGS[robot.current_task];
      }
      else
      {
        current_task = "none";
      }
      break;
    default:
      std::cout << "Invalid robot type in print_robot" << std::endl;
      break;
    }

    std::cout << "  robot " << robot_index << ":" << std::endl;
    std::cout << "      id=" << robot.id << std::endl;
    std::cout << "      type=" << robot_type << std::endl;
    std::cout << "      time_remaining=" << robot.time_remaining << std::endl;
    std::cout << "      current_task=" << current_task << std::endl;
    std::cout << "      x=" << robot.odom.pose.pose.position.x << std::endl;
    std::cout << "      y=" << robot.odom.pose.pose.position.y << std::endl;
    std::cout << "      toggle_sleep=" << robot.toggle_sleep << std::endl;
  }

  void ForwardSearch::print_robots(std::vector<mac::Robot> robots)
  {
    for (int i = 0; i < robots.size(); i++)
    {
      print_robot(i, robots);
    }
  }

  void ForwardSearch::print_state(mac::State s)
  {
    std::cout << "///////////////////////////////" << std::endl;
    print_robots(s.robots);

    std::cout << "///////////////////////////////" << std::endl;
    print_volatile_map(s.volatile_map);

    std::cout << "///////////////////////////////" << std::endl;
    std::cout << "Time:" << s.time << std::endl;

    std::cout << "///////////////////////////////" << std::endl;
  }
}