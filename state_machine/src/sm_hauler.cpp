#include <state_machine/sm_hauler.hpp>


SmHauler::SmHauler() :
ac("move_base", true),
tf2_listener(tf_buffer),
move_base_state_(actionlib::SimpleClientGoalState::PREEMPTED)
{
  // Load Parameters
  node_name_ = "state_machine";
  if (ros::param::get("robot_name", robot_name_) == false)
  {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get("robot_id", robot_id_) == false)
  {
    ROS_FATAL("No parameter 'robot_id' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get("/num_excavators", num_excavators_) == false)
  {
    ROS_FATAL("No parameter 'num_excavators' specified");
    ros::shutdown();
    exit(1);
  }

  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_status_pub = nh.advertise<state_machine::RobotStatus>("state_machine/status", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("driving/cmd_vel", 1);
  driving_mode_pub = nh.advertise<std_msgs::Int64>("driving/driving_mode", 1);
  cmd_bin_pub = nh.advertise<std_msgs::Float64>("bin/command/position", 1);
  hauler_status_pub = nh.advertise<state_machine::HaulerStatus>("state_machine/hauler_status", 1);
  cmd_sensor_yaw_pub = nh.advertise<std_msgs::Float64>("sensor/yaw/command/position", 1);
  cmd_sensor_pitch_pub = nh.advertise<std_msgs::Float64>("sensor/pitch/command/position", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmHauler::localizedBaseCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmHauler::localizationCallback, this);
  driving_mode_sub = nh.subscribe("driving/driving_mode",1, &SmHauler::drivingModeCallback, this);
  laser_scan_sub = nh.subscribe("laser/scan",1, &SmHauler::laserCallback, this);
  planner_interrupt_sub = nh.subscribe("/task_planner/interrupt", 1, &SmHauler::plannerInterruptCallback, this);
  system_monitor_sub =nh.subscribe("system_monitor",1, &SmHauler::systemMonitorCallback, this);
  init_attitude_sub =nh.subscribe("/initial_attitude",1, &SmHauler::initialAttitudeCallback, this);
  for (int i=0; i<num_excavators_; i++)
  {
    std::string excavator_odom_topic;
    excavator_odom_topic = "/small_excavator_" + std::to_string(i+1) + "/localization/odometry/sensor_fusion";
    excavator_odom_subs.push_back(nh.subscribe(excavator_odom_topic, 1, &SmHauler::excavatorOdomCallback, this));

    std::string excavation_status_topic;
    excavation_status_topic = "/small_excavator_" + std::to_string(i+1) + "/state_machine/excavation_status";
    excavation_status_subs.push_back(nh.subscribe(excavation_status_topic, 1, &SmHauler::excavationStatusCallback, this));
  }

  // Clients
  clt_stop = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_rip = nh.serviceClient<driving_tools::RotateInPlace>("driving/rotate_in_place");
  clt_drive = nh.serviceClient<driving_tools::MoveForward>("driving/move_forward");
  clt_lights = nh.serviceClient<srcp2_msgs::SpotLightSrv>("spot_light");
  clt_power = nh.serviceClient<srcp2_msgs::SystemPowerSaveSrv>("system_monitor/power_saver");
  clt_brake = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_approach_base = nh.serviceClient<src2_approach_services::ApproachChargingStation>("approach_charging_station_service");
  clt_rover_static = nh.serviceClient<sensor_fusion::RoverStatic>("sensor_fusion/toggle_rover_static");
  clt_reset_position = nh.serviceClient<sensor_fusion::ResetPosition>("sensor_fusion/reset_position");
  clt_homing = nh.serviceClient<sensor_fusion::HomingUpdate>("homing");
  clt_homing_proc_plant = nh.serviceClient<sensor_fusion::HomingUpdateProcessingPlant>("homing_processing_plant");
  clt_sf_true_pose = nh.serviceClient<sensor_fusion::GetTruePose>("true_pose");
  clt_waypoint_checker = nh.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  clt_srcp2_brake_rover= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_task_planning = nh.serviceClient<task_planning::PlanInfo>("/task_planner/exc_haul");
  clt_approach_excavator = nh.serviceClient<src2_approach_services::ApproachExcavator>("approach_excavator_service");
  clt_approach_bin = nh.serviceClient<src2_approach_services::ApproachBin>("approach_bin_service");
  clt_location_of_bin = nh.serviceClient<range_to_base::LocationOfBin>("location_of_bin_service");
  clt_location_of_excavator = nh.serviceClient<range_to_base::LocationOfExcavator>("location_of_excavator_service");
  clt_find_excavator = nh.serviceClient<move_excavator::FindExcavator>("manipulation/find_excavator");
  clt_go_to_goal = nh.serviceClient<waypoint_nav::GoToGoal>("navigation/go_to_goal");
  clt_find_object = nh.serviceClient<src2_object_detection::FindObject>("/find_object");
  clt_dump_coordination = nh.serviceClient<task_planning::DumpCoordination>("/task_planner/dump_coordination");

  map_timer = ros::Time::now();
  wp_checker_timer=  ros::Time::now();
  laser_collision_timer = ros::Time::now();

  if(robot_id_ == 1)
  {
    // Local copy of the processing plant location
    proc_plant_bin_location_.x = 8.00;
    proc_plant_bin_location_.y = 8.00;
    proc_plant_bin_location_.z = 3.00;
  }
  else
  {
    // Local copy of the processing plant location
    proc_plant_bin_location_.x = 8.00;
    proc_plant_bin_location_.y = 12.00;
    proc_plant_bin_location_.z = 3.60;
  }
    
  front_of_bin_location_ = proc_plant_bin_location_;

  // Local copy of the charging station location
  charging_station_location_.x = 1.25;
  charging_station_location_.y = 9.00;
  charging_station_location_.z = 1.65;

  nav_msgs::Odometry temp_small_excavator_odom;
  state_machine::ExcavationStatus temp_small_excavator_status;
  for (int i=0; i<num_excavators_; i++)
  {
    small_excavators_odom_.push_back(temp_small_excavator_odom);
    small_excavators_status_.push_back(temp_small_excavator_status);
  }

  partner_excavator_id_ = robot_id_;
}

void SmHauler::run()
{
  ros::Rate loop_rate(5); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_have_true_pose: " << (int)flag_have_true_pose);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_interrupt_plan: " << (int)flag_interrupt_plan);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_arrived_at_waypoint: " << (int)flag_arrived_at_waypoint);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_localizing_volatile: " << (int)flag_localizing_volatile);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_dumping: " << (int)flag_dumping);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_recovering_localization: " << (int)flag_recovering_localization);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_brake_engaged: " << (int)flag_brake_engaged);
    ROS_INFO_STREAM("[" << robot_name_ << "] Flags: T,I,E,A,L,D,R,B");
    ROS_INFO_STREAM("[" << robot_name_ << "] State: " << (int)flag_have_true_pose << ","
                                                      << (int)flag_interrupt_plan << ","
                                                      << (int)flag_emergency << ","
                                                      << (int)flag_arrived_at_waypoint << ","
                                                      << (int)flag_localizing_volatile << ","
                                                      << (int)flag_dumping << ","
                                                      << (int)flag_recovering_localization << ","
                                                      << (int)flag_brake_engaged);
    //--------------------------------------------------------------------------------------
    //---------------------------------------------------------------------------------------------------------------------

    // State machine truth table ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    state_to_exec.clear();
    state_to_exec.resize(num_states,0);

    if(!flag_have_true_pose || !flag_spread_out)
    {
      state_to_exec.at(_initialize) = 1;
    }
    else if(flag_emergency || flag_wasted)
    {
      state_to_exec.at(_emergency) = 1;
    }
    else if(flag_interrupt_plan || (flag_arrived_at_waypoint && !flag_recovering_localization && !flag_localizing_volatile && !flag_dumping && !flag_brake_engaged))
    {
      state_to_exec.at(_planning) = 1;
    }
    else if(flag_arrived_at_waypoint && flag_recovering_localization && !flag_brake_engaged)
    {
      state_to_exec.at(_lost) = 1;
    }
    else if(flag_arrived_at_waypoint && flag_localizing_volatile && !flag_brake_engaged)
    {
      state_to_exec.at(_volatile_handler) = 1;
    }
    else if(flag_arrived_at_waypoint && flag_dumping && !flag_brake_engaged)
    {
      state_to_exec.at(_hauler_dumping) = 1;
    }
    else if(!flag_arrived_at_waypoint && !flag_brake_engaged)
    {
      state_to_exec.at(_traverse) = 1;
    }
    else
    {
      flag_interrupt_plan = true;
      flag_arrived_at_waypoint = true;
      flag_recovering_localization = false;
      flag_localizing_volatile = false;
      flag_dumping = false;
      flag_fallthrough_condition = true;
    }
    //---------------------------------------------------------------------------------------------------------------------

    // State machine execute states +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(state_to_exec.at(_initialize))
    {
      stateInitialize();
    }
    else if(state_to_exec.at(_planning))
    {
      statePlanning();
    }
    else if(state_to_exec.at(_traverse))
    {
      stateTraverse();
    }
    else if(state_to_exec.at(_volatile_handler))
    {
      stateVolatileHandler();
    }
    else if(state_to_exec.at(_lost))
    {
      stateLost();
    }
    else if(state_to_exec.at(_hauler_dumping))
    {
      stateDump();
    }
    else if(state_to_exec.at(_emergency))
    {
      stateEmergency();
    }
    else
    {
      ROS_FATAL_STREAM("[" << robot_name_ << "] " << "No state to execute");
      flag_fallthrough_condition = false;
    }
    // -------------------------------------------------------------------------------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// State function definitions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmHauler::stateInitialize()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Initialization State!");

  while (!clt_lights.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for Lights");
  }

  while (!clt_sf_true_pose.waitForExistence())
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waiting for TruePose service");
  }

  while (!clt_approach_excavator.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachExacavator service");
  }

  while (!clt_approach_base.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachChargingStation service");
  }

  while (!clt_approach_bin.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachExacavator service");
  }

  while (!clt_location_of_excavator.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for LocateExcavator service");
  }

  while (!clt_find_object.waitForExistence())
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waiting for FindObject service");
  }

  double progress = 0;

  Lights(20);

  SetPowerMode(true);

  Stop(0.1);
  Brake(100.0);
  RoverStatic(true);
  bool initialize_other_robot_attitude = false;
  if (robot_id_ == 1)
  {
    initialize_other_robot_attitude = true;
    GetTruePose(initialize_other_robot_attitude);
  }
  RoverStatic(false);
  if (flag_have_true_pose && !flag_spread_out)
  {
    SetPowerMode(false);
    Brake(0.0);
    ros::spinOnce();
    if (robot_id_ == 1)
    {
      RotateToHeading(5.0);
      Stop(0.1);
    }
    else
    {
      RotateToHeading(1.4);
      Stop(0.1);
    }
    DriveCmdVel(HAULER_MAX_SPEED,0,0,12);
    Stop(0.1);
    Brake(100.0);

    SetPowerMode(true);

    flag_spread_out = true;
    ClearCostmaps(5.0);
  }
  Brake(0.0);

  progress = 1.0;

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _initialize;
  sm_status_pub.publish(status_msg);
}

void SmHauler::statePlanning()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Planning State!");

  double progress = 0;

  CancelMoveBaseGoal();

  SetPowerMode(true);

  Plan();

  if (!no_objective)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"New objective.");
    goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

    SetPowerMode(false);

    Brake (0.0);
    RotateToHeading(goal_yaw_);
    Stop(0.1);
    BrakeRamp(100, 1, 0);
    Brake(0.0);

    SetPowerMode(true);

    // CheckWaypoint(3); // TODO: Check if they needed

    ClearCostmaps(5.0);
    double dx = goal_pose_.position.x - current_pose_.position.x;
    double dy = goal_pose_.position.y - current_pose_.position.y;
    double D = hypot(dx,dy);
    double dt = 87.5*(D/90);
    if(flag_localizing_volatile)
    {
      if(flag_first_volatile)
      {
        ros::Duration(dt+3).sleep();
      }
      else
      {
        ros::Duration(3).sleep();
      }
    }

    SetPowerMode(false);
    SetMoveBaseGoal();

    progress = 1.0;
  }
  else
  {
    ros::Duration(20).sleep();
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"No objective. Will ask again in 20s.");
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _planning;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateTraverse()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Traverse State");

  SetPowerMode(false);

  move_base_state_ = ac.getState();
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"MoveBase status: "<< move_base_state_.toString()
                      << ". Goal: (" << goal_pose_.position.x << "," << goal_pose_.position.y <<").");

  double progress = 0;

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 1.5)
  {

    CancelMoveBaseGoal();

    if(flag_approaching_side)
    {
      flag_approached_side = true;
      flag_approaching_side = false;
    }

    if(flag_approaching_front)
    {
      flag_approached_front = true;
      flag_approaching_front = false;
    }

    if(flag_dumping)
    {
      RotateToHeading(proc_plant_bin_location_.z);
      Stop(0.1);
    }

    flag_arrived_at_waypoint = true;
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Close to goal, getting new waypoint.");
  }
  else
  {
    if(move_base_state_ == actionlib::SimpleClientGoalState::ABORTED || move_base_state_ == actionlib::SimpleClientGoalState::LOST)
    {
      ROS_WARN_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"MoveBase status: "<< move_base_state_.toString());
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"MoveBase has failed to make itself useful.");

      // flag_arrived_at_waypoint = true;
      // flag_recovering_localization = false;
      // flag_localizing_volatile = false;

      // ClearCostmaps(5.0); // TODO: Check if they needed

      // move_base_fail_counter++;
      // Stop (0.1);
      // BrakeRamp(100, 1, 0);
      // Brake(0.0);

      // if(move_base_fail_counter > 5)
      // {
      //   ClearCostmaps(5.0);
      //   move_base_fail_counter = 0;
      // }

      ros::Duration timeoutMap(5.0);
      if (ros::Time::now() - map_timer > timeoutMap)
      {
        CancelMoveBaseGoal();
        ros::spinOnce();
        RotateToHeading(yaw_ + M_PI_2);
        Stop(0.1);
        ClearCostmaps(5.0);
        SetMoveBaseGoal();
      }

      ros::Duration timeoutWaypoint(480.0);
      if(move_base_state_ == actionlib::SimpleClientGoalState::PREEMPTED && ros::Time::now() - waypoint_timer > timeoutWaypoint)
      {
        CancelMoveBaseGoal();
        ros::spinOnce();
        RotateToHeading(yaw_ + M_PI_2);
        Stop(0.1);
        ClearCostmaps(5.0);
        SetMoveBaseGoal();
      }

    }

    // ros::Duration timeoutWaypointCheck(3.0);
    // if (ros::Time::now() - wp_checker_timer > timeoutWaypointCheck)
    // {
    //   CheckWaypoint(3);
    //   wp_checker_timer = ros::Time::now();
    // }

    // ros::Duration timeoutMap(90.0);
    // if (ros::Time::now() - map_timer > timeoutMap)
    // {
    //   ClearCostmaps(5.0);
    //   map_timer =ros::Time::now();
    // }

    // ros::Duration timeoutWaypoint(120);
    // if (ros::Time::now() - waypoint_timer > timeoutWaypoint )
    // {
    //   ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Waypoint reached timeout.");
    //   ClearCostmaps(5.0);
    // }
  }

  progress = distance_to_goal;

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _traverse;
  sm_status_pub.publish(status_msg);

}

void SmHauler::stateVolatileHandler()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Volatile Handling State!");

  double progress = 0;

  CancelMoveBaseGoal();

  SetPowerMode(true);

  flag_first_volatile = false;

  if(!flag_approached_side && partner_excavation_status_.found_parking_site.data)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Approaching the Excavator side are!");

    goal_pose_.position = partner_excavation_status_.parking_pose.position;
    if(partner_excavation_status_.parking_side.data == -1)
    {
      parking_left_offset = 0.0;
    }
    else
    {
      parking_left_offset = 0.4;
    }

    ClearCostmaps(5.0);

    SetPowerMode(false);

    SetMoveBaseGoal();

    flag_approaching_side = true;
    flag_arrived_at_waypoint = false;
    flag_localizing_volatile = true;

    parking_recovery_counter_ = 0;

    PublishHaulerStatus();

    return;
  }

  PublishHaulerStatus();

  if(!flag_full_bin && flag_approached_side)
  {
    if(!flag_approached_excavator && partner_excavation_status_.found_volatile.data)
    {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Approaching the Excavator!");

      tf2::Quaternion q(partner_excavation_status_.parking_pose.orientation.x,
                  partner_excavation_status_.parking_pose.orientation.y,
                  partner_excavation_status_.parking_pose.orientation.z,
                  partner_excavation_status_.parking_pose.orientation.w);

      double roll, pitch, yaw;

      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      SetPowerMode(false);

      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Rotating in the direction of the excavator. Goal yaw: " << yaw);
      RotateToHeading(yaw);
      Stop(0.1);
      // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Rotated to yaw: " << yaw_);

      flag_approached_excavator = ApproachExcavator(3, 3.0);

      SetPowerMode(true);

      CommandCamera(0,0,5);

      Stop(0.1);
      flag_located_excavator = false;
      flag_parked_hauler = false;
      PublishHaulerStatus();
    }

    if (!flag_parked_hauler && flag_approached_excavator)
    {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Parking Hauler.");

      bool goal_from_bucket = false;
      // Trying with computer vision

      flag_located_excavator = FindExcavator(10);
      goal_from_bucket = flag_located_excavator;
      if(!flag_located_excavator)
      {
        // Trying with laser
        flag_located_excavator = LocateExcavator();
        CommandCamera(0,0,0.1);
      }
      if(flag_located_excavator)
      {
        if(!goal_from_bucket)
        {
          ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Obtained goal from LaserScan");
          SetPowerMode(false);
          flag_parked_hauler = GoToWaypoint(1.5 + parking_left_offset, 1.0);
          SetPowerMode(true);
          Stop(0.1);
        }
        else
        {
          ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Obtained goal from Bucket detection");
          SetPowerMode(false);
          flag_parked_hauler = GoToWaypoint(1.3 + parking_left_offset, -0.3);
          SetPowerMode(true);
          Stop(0.1);
        }
        PublishHaulerStatus();
      }

      // Trying with object detection
      if(!flag_located_excavator)
      {
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Other methods failed, trying Approach again.");
        SetPowerMode(false);
        flag_located_excavator = ApproachExcavator(1 + parking_left_offset, 1.5);
        SetPowerMode(true);
        CommandCamera(0,0,0.1);
        Stop(0.1);
        flag_parked_hauler = true;
        PublishHaulerStatus();
      }

      Brake(500.0);
      ros::spinOnce();
      while(!flag_full_bin)
      {
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Bin is not full yet.");
        ros::spinOnce();
        ros::Duration(1.0).sleep();

        if(parking_recovery_counter_ > 4)
        {
          PublishHaulerStatus();
          break;
        }

        if(partner_excavation_status_.failed_to_find_hauler.data)
        {
          flag_approached_side = true;
          flag_approached_excavator = false;
          flag_located_excavator = false;
          flag_parked_hauler = false;

          parking_recovery_counter_++;
          PublishHaulerStatus();
          break;
        }
      }

      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Backing maneuver.");
      Brake(0.0);

      SetPowerMode(false);
      DriveCmdVel(-0.5,0,0,3);
      Stop (0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);
      SetPowerMode(true);
    }
  }

  if(flag_full_bin)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Full bin! Going to Dump State.");
    PublishHaulerStatus();

    // RESET ALL VOL HANDLING FLAGS
    flag_approaching_side = false;
    flag_approached_side = false;
    flag_approached_excavator = false;
    flag_located_excavator = false;
    flag_parked_hauler = false;

    // SET SMACH FLAGS TO DUMPING
    flag_interrupt_plan = false;
    flag_recovering_localization = false;
    flag_localizing_volatile = false;
    flag_arrived_at_waypoint = false;
    flag_dumping = true;

    SetPowerMode(false);

    goal_pose_.position = proc_plant_bin_location_;
    SetMoveBaseGoal();

    progress = 1.0;
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _volatile_handler;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateLost()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Lost State!");

  double progress = 1.0;

  CancelMoveBaseGoal();

  SetPowerMode(false);

  Stop (2.0);

  bool approachSuccess = ApproachChargingStation(3);

  BrakeRamp(100, 1, 0);

  if(approachSuccess)
  {
    bool homingSuccess = HomingUpdate(flag_need_init_landmark);
    // bool homingSuccess = true;
    if (homingSuccess)
    {
      progress = 1.0;
      flag_recovering_localization = false;
    }
    else
    {
      progress = -1.0;
    }

    Brake(0.0);
    if(robot_id_ == 1)
    {
      DriveCmdVel(-0.5,0.0,0.0,3);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      RotateToHeading(5.5);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      DriveCmdVel(0.5,0.0,0.0,10);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);
    }
    else
    {
      DriveCmdVel(-0.5,0.0,0.0,5);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      RotateToHeading(5.5);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      DriveCmdVel(0.5,0.0,0.0,10);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);
    }
    SetPowerMode(true);
    ClearCostmaps(5.0);
    BrakeRamp(100, 1, 0);
    Brake(0.0);
    SetPowerMode(false);
  }
  else
  {    
    SetPowerMode(true);
    ClearCostmaps(5.0);
    BrakeRamp(100, 1, 0);
    Brake(0.0);
    SetPowerMode(false);

    progress = -1.0;
  }
  Brake(0.0);

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _lost;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateEmergency()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Emergency Charging State!");

  CancelMoveBaseGoal();

  RotateToHeading(M_PI_2);
  Stop(0.1);

  SetPowerMode(true);

  double progress = 0;

  progress = power_level_/50;

  if(power_level_ > 50)
  {
    flag_emergency = false;
    SetPowerMode(false);
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _emergency;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateDump()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Dumping State!");

  CancelMoveBaseGoal();

  double progress = 0.0;
  
  if(!flag_approached_front)
  {
    flag_approached_front = FindBin();

    if(!flag_approached_front)
    {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Dumping. Approaching the front of the processing plant bin!");

      goal_pose_.position = front_of_bin_location_;

      ClearCostmaps(5.0);

      SetPowerMode(false);

      SetMoveBaseGoal();

      flag_approaching_front = true;
      flag_arrived_at_waypoint = false;
      flag_dumping = true;

      return;
    }
  }

  SetPowerMode(true);

  if(!flag_allowed_to_dump)
  {
    flag_allowed_to_dump = RequestDumping(true);
  }

  if(flag_allowed_to_dump)
  {
    SetPowerMode(false);
    // TODO: Approach Bin return false it's not the bin in front
    flag_dumped = ApproachBin(3);

    // localize bin after approaching bin
    if (flag_dumped)
    {
      // HomingUpdateProcessingPlant();

      Brake(0.0);

      DriveCmdVel(-0.5,0.0,0.0,8);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      RotateToHeading(4.2);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      DriveCmdVel(0.5,0.0,0.0,5);
      Stop(0.1);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      ClearCostmaps(5.0);
      BrakeRamp(100, 1, 0);
      Brake(0.0);

      progress = 1.0;

      PublishHaulerStatus();

      RequestDumping(false);
      flag_allowed_to_dump = false;

      // RESET ALL VOL HANDLING FLAGS
      flag_approaching_front = false;
      flag_approached_front = false;
      flag_approaching_side = false;
      flag_approached_side = false;
      flag_approached_excavator = false;
      flag_located_excavator = false;
      flag_parked_hauler = false;
      flag_full_bin = false;

      // SET SMACH FLAGS TO LOST
      flag_interrupt_plan = false;
      flag_recovering_localization = true;
      flag_localizing_volatile = false;
      flag_arrived_at_waypoint = true;
      flag_dumping = false;
    }
    else
    {
      Brake(0.0);
      progress = -1.0;
    }
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _hauler_dumping;
  sm_status_pub.publish(status_msg);
}

//------------------------------------------------------------------------------------------------------------------------

// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmHauler::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = (bool) msg->data;
  if (flag_localized_base)
  {
    ROS_WARN_STREAM_ONCE("Initial Localization Successful = " << (int)flag_localized_base);
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Waiting for Initial Localization  = " << (int)flag_localized_base);
  }
}

void SmHauler::systemMonitorCallback(const srcp2_msgs::SystemMonitorMsg::ConstPtr& msg)
{
  power_level_ = msg->power_level;
  power_rate_ = msg->power_rate;

  if (power_level_< 30)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " << "Power Level Warning: " << power_level_);

    flag_emergency = true;
  }
}


void SmHauler::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg)
{
  driving_mode=msg->data;
}

void SmHauler::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev_ = yaw_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

  double radius = hypot(current_pose_.position.x, current_pose_.position.y);

  if (abs(pitch_ * 180 / M_PI) > 10)
  {
    ROS_WARN_STREAM_THROTTLE(10, "Robot Climbing Up/Down! Pitch: " << pitch_ * 180 / M_PI);
    if (curr_max_speed_ != HAULER_MAX_SPEED*3/4)
    {
      SetMoveBaseSpeed(HAULER_MAX_SPEED*3/4);
      curr_max_speed_ = HAULER_MAX_SPEED*3/4;
    }

    if (abs(pitch_ * 180 / M_PI) > 27)
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Robot Cant Climb! Pitch: " << pitch_ * 180 / M_PI);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Commanding IMMOBILITY.");

      CancelMoveBaseGoal();
      Stop(0.05);
      Brake(200.0);
      Brake(0.0);

      DriveCmdVel(-0.3,0.0,0.0,3);
      Stop(0.1);
      Brake(100.0);
      Brake(0.0);

      int direction = (rand() % 2)>0? 1: -1;
      RotateToHeading(yaw_ + direction * M_PI_4);
      Stop(0.1);
      Brake(100.0);
      Brake(0.0);

      if(radius > CRATER_RADIUS)
      {
        goal_pose_.position.x = 0;
        goal_pose_.position.y = 0;
        flag_interrupt_plan = false;
        flag_emergency = false;
        flag_arrived_at_waypoint = false;
        flag_recovering_localization = true;
        flag_localizing_volatile = false;
      }

      ClearCostmaps(5.0);
      SetMoveBaseGoal();
    }
  }
  else
  {
    if (curr_max_speed_ != HAULER_MAX_SPEED)
    {
      SetMoveBaseSpeed(HAULER_MAX_SPEED);
      curr_max_speed_ = HAULER_MAX_SPEED;
    }
  }

  if (abs(roll_ * 180 / M_PI) > 10)
  {
    ROS_WARN_STREAM_THROTTLE(10, "Robot is Sideways! Roll: " << roll_ * 180 / M_PI);
    if (curr_max_speed_ != HAULER_MAX_SPEED*3/4)
    {
      SetMoveBaseSpeed(HAULER_MAX_SPEED*3/4);
      curr_max_speed_ = HAULER_MAX_SPEED*3/4;
    }

    if (abs(roll_ * 180 / M_PI) > 27)
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Robot Cant Climb! Roll: " << roll_ * 180 / M_PI);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Commanding IMMOBILITY.");

      CancelMoveBaseGoal();
      Stop(0.05);
      Brake(1000.0);
      Brake(0.0);

      DriveCmdVel(-0.3,0.0,0.0,3);
      Stop(0.1);
      Brake(100.0);
      Brake(0.0);

      int direction = (rand() % 2)>0? 1: -1;
      RotateToHeading(yaw_ + direction * M_PI_4);
      Stop(0.1);
      Brake(100.0);
      Brake(0.0);

      SetMoveBaseGoal();
    }
  }
  else
  {
    if (curr_max_speed_ != HAULER_MAX_SPEED)
    {
      SetMoveBaseSpeed(HAULER_MAX_SPEED);
      curr_max_speed_ = HAULER_MAX_SPEED;
    }
  }
}

void SmHauler::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<float> ranges = msg->ranges;
  std::sort (ranges.begin(), ranges.end());
  float min_range = 0;
  for (int i = 0; i < LASER_SET_SIZE; ++i)
  {
    min_range = min_range + ranges[i];
  }
  min_range = min_range/LASER_SET_SIZE;
  // ROS_INFO_STREAM_THROTTLE(2,"Minimum range average: " << min_range);

  if (min_range < LASER_THRESH)
  {
    if (ros::Time::now() - laser_collision_timer < ros::Duration(20))
    {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Close to wall.");
      counter_laser_collision_++;
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Counter laser:" << counter_laser_collision_);
    }
    else
    {
      counter_laser_collision_ = 0;
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Counter laser RESET!");
    }
    laser_collision_timer = ros::Time::now();
  }

  if (counter_laser_collision_ > LASER_COUNTER_THRESH)
  {
    counter_laser_collision_ =0;
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"LASER COUNTER > 20 ! Starting Recovery.");
    // immobilityRecovery(2);
  }
}

void SmHauler::excavationStatusCallback(const ros::MessageEvent<state_machine::ExcavationStatus const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const state_machine::ExcavationStatusConstPtr& msg = event.getMessage();

  char msg_excavator_ind = topic.c_str()[EXCAVATOR_STR_LOC];

  int msg_excavator_id = std::atoi(&msg_excavator_ind);

  small_excavators_status_[msg_excavator_id-1] = *msg;

  if (partner_excavator_id_ == msg_excavator_id)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Got new partner status.");
    partner_excavation_status_ = *msg;

    if(partner_excavation_status_.counter.data > previous_scoop_counter)
    {
      ExecuteShakeBin(1.0);
    }

    if(partner_excavation_status_.counter.data == -1)
    {
      flag_full_bin = true;
      previous_scoop_counter = 0;
      CommandCamera(0.0,0.0,1.0);
    }
  }
}

void SmHauler::excavatorOdomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const nav_msgs::OdometryConstPtr& msg = event.getMessage();

  char msg_excavator_ind = topic.c_str()[EXCAVATOR_STR_LOC];
  int msg_excavator_id = std::atoi(&msg_excavator_ind);

  small_excavators_odom_[msg_excavator_id-1] = *msg;

  if (partner_excavator_id_ == msg_excavator_id)
  {
    partner_excavator_location_ = msg->pose.pose.position;
  }
}

void SmHauler::activeCallback()
{
}

void SmHauler::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Goal done");
}

void SmHauler::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Got feedback");
}

void SmHauler::watchdogCallback(const localization_watchdog::WatchdogStatus::ConstPtr& msg)
{
  flag_wasted = msg->wasted.data;
  flag_immobile = msg->immobile.data;
}

void SmHauler::plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg)
{
  task_planning::PlanInfo srv_plan;
  srv_plan.request.replan.data = false;
  srv_plan.request.type.data = (uint8_t) mac::HAULER;
  srv_plan.request.id.data = (uint8_t) robot_id_;
  ROS_WARN_STREAM("[" << robot_name_ << "] " << " PLANNER INTERRUPT CALLBACK: robot.type " <<  (int) srv_plan.request.type.data << ", robot.id " << (int) srv_plan.request.id.data );

  if (clt_task_planning.call(srv_plan))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service Plan");
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call Plan service");
  }

  if(!(prev_srv_plan.response.objective.point.x == srv_plan.response.objective.point.x &&
  prev_srv_plan.response.objective.point.y == srv_plan.response.objective.point.y &&
  prev_srv_plan.response.code == srv_plan.response.code) &&
  (!flag_recovering_localization) && (!flag_dumping) && (!flag_localizing_volatile))
  {
    flag_interrupt_plan = true;
  }
  else
  {
    flag_interrupt_plan = false;
  }
}

void SmHauler::initialAttitudeCallback(const geometry_msgs::QuaternionConstPtr& msg)
{
  if(!flag_have_true_pose)
  {
    flag_have_true_pose = true;
  }
}

void SmHauler::CancelMoveBaseGoal()
{
  if(ac.getState() != actionlib::SimpleClientGoalState::LOST)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
    ac.waitForServer();
    ac.cancelGoal();
    ac.waitForResult(ros::Duration(0.25));
  }
}

void SmHauler::SetMoveBaseGoal()
{
  map_timer =ros::Time::now();
  waypoint_timer =ros::Time::now();
  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  SetPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Sending goal to MoveBase: (x,y): ("
                                      << move_base_goal.target_pose.pose.position.x << ","
                                      << move_base_goal.target_pose.pose.position.y << ").");
  ac.sendGoal(move_base_goal, boost::bind(&SmHauler::doneCallback, this,_1,_2), boost::bind(&SmHauler::activeCallback, this), boost::bind(&SmHauler::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));
}

void SmHauler::SetMoveBaseSpeed(double max_speed)
{
  // Update move_base max speed
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  double_param.name = "max_vel_x";
  double_param.value = max_speed;
  conf.doubles.push_back(double_param);

  srv_req.config = conf;

  if (ros::service::call("move_base/DWAPlannerROS_SRC/set_parameters", srv_req, srv_resp))
  {
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service to reconfigure MoveBase max speed to: "<< max_speed);
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service to reconfigure MoveBase (max speed).");
  }
}

void SmHauler::SetPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
  const double pitch = 0.0;
  const double roll = 0.0;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  poseGoal.target_pose.header.frame_id = robot_name_+"_odom";
  poseGoal.target_pose.pose.position.x = x;
  poseGoal.target_pose.pose.position.y = y;
  poseGoal.target_pose.pose.position.z = 0.0;
  poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
  poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
  poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
  poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmHauler::SetPowerMode(bool power_save)
{
  srcp2_msgs::SystemPowerSaveSrv srv_power;
  srv_power.request.power_save = power_save;
  if (clt_power.call(srv_power))
  {
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service PowerSaver");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service PowerSaver");
  }
}

void SmHauler::RotateToHeading(double desired_yaw)
{
  ros::Rate rateRotateToHeading(20);

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Starting yaw control.");

  double yaw_thres = 0.1;

  ros::spinOnce();

  double yaw_error = desired_yaw - yaw_;
  if (fabs(yaw_error) > M_PI)
  {
    if(yaw_error > 0)
    {
      yaw_error = yaw_error - 2*M_PI;
    }
    else
    {
      yaw_error = yaw_error + 2*M_PI;
    }
  }

  bool flag_heading_fail = false;
  ros::Time rotate_timer = ros::Time::now();
  ros::Duration timeoutHeading(45.0); // Timeout of 20 seconds

  while(fabs(yaw_error) > yaw_thres)
  {
    RotateInPlace(copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error),0.0);

    rateRotateToHeading.sleep();
    ros::spinOnce();

    yaw_error = desired_yaw - yaw_;
    if (fabs(yaw_error) > M_PI)
    {
      if(yaw_error > 0)
      {
        yaw_error = yaw_error - 2*M_PI;
      }
      else
      {
        yaw_error = yaw_error + 2*M_PI;
      }
    }
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Trying to control yaw to desired angles. Yaw error: "<<yaw_error);

    // ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"TIME: "<<ros::Time::now() - rotate_timer << ", TIMEOUT: " << timeoutHeading);

    if (ros::Time::now() - rotate_timer > timeoutHeading)
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Yaw Control Failed. Possibly stuck. Break control.");
      flag_heading_fail = true;
      break;
    }
  }

  if (flag_heading_fail)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Recovery action initiated in yaw control.");

    Stop(0.1);
    DriveCmdVel (-0.5, 0.0, 0.0, 4.0);
    Stop(0.1);
    BrakeRamp(100, 1, 0);
    Brake(0.0);

    flag_heading_fail = false;
  }
  else
  {
    Stop(0.1);
  }
}

void SmHauler::homingRecovery()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Starting Homing Recovery.");

  Lights(20);

  CancelMoveBaseGoal();
  Stop(0.1);
  BrakeRamp(100, 1, 0);
  Brake(0.0);

  DriveCmdVel(-0.3,-0.6, 0.0, 5.0);
  Stop(0.1);

  DriveCmdVel(0.0,0.0,-0.25,4.0);
  Stop(0.1);

  BrakeRamp(100, 1, 0);
  Brake(0.0);

  DriveCmdVel(0.6,0.0,0.0,4.5);
  Stop(0.1);
  BrakeRamp(100, 1, 0);
  Brake(0.0);
}

void SmHauler::immobilityRecovery(int type)
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Starting Immobility Recovery.");

  CancelMoveBaseGoal();
  Stop(0.1);
  Brake(100.0);
  Brake(0.0);

  DriveCmdVel(-0.5,0.0,0.0,3.0);
  Stop(0.1);
  BrakeRamp(100, 1, 0);
  Brake(0.0);

  DriveCmdVel(-0.3,-0.6, 0.0, 4.0);
  Stop(0.1);
  BrakeRamp(100, 1, 0);
  Brake(0.0);
}

void SmHauler::ClearCostmaps(double wait_time)
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Braking rover to clear the Map");
  BrakeRamp(100, 1, 0); // Give more time

  ROS_WARN_STREAM("[" << robot_name_ << "] " << "Move Base State: " << move_base_state_.toString());

  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("move_base/clear_costmaps",emptymsg))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " << "Called service to clear costmap layers.");
    ROS_WARN_STREAM("[" << robot_name_ << "] " << "Map Cleared");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed calling clear_costmaps service.");
  }

  Brake(0.0);
}

void SmHauler::GetTruePose(bool initialize_other_robot_attitude)
{
  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  srv_sf_true_pose.request.initialize = initialize_other_robot_attitude;
  if (clt_sf_true_pose.call(srv_sf_true_pose))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service TruePose");
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Status of SF True Pose: "<< (int) srv_sf_true_pose.response.success);
    flag_have_true_pose = true;
    flag_called_get_true_pose = true;
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call Pose Update service");
  }
}

void SmHauler::Lights(double intensity)
{
  // Turn on the Lights
  srcp2_msgs::SpotLightSrv srv_lights;
  srv_lights.request.range  = intensity;
  if (clt_lights.call(srv_lights))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service SpotLight");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service SpotLight");
  }
}

void SmHauler::RotateInPlace(double speed_ratio, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.speed_ratio  = speed_ratio;
  if (clt_rip.call(srv_turn))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call service RotateInPlace");
  }
}

void SmHauler::Stop(double time)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 20 seconds
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Commanded Stop.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub.publish(cmd_vel);
  }
}

void SmHauler::Brake(double intensity)
{
  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = intensity;
  if (clt_srcp2_brake_rover.call(srv_brake))
  {
    if (intensity < 0.01)
    {
      flag_brake_engaged =false;
    }
    else
    {
      flag_brake_engaged =true;
    }
    //ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service SRCP2 Brake. Engaged?: " << flag_brake_engaged);
  }
  else
  {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service Brake");
  }
}

void SmHauler::BrakeRamp(double max_intensity, double time, int aggressivity)
{
  double freq = 10;
  ros::Rate brake_rate(freq);
  int num_steps = (int) freq * time;
  if(aggressivity == 0)
  {
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Brake Ramp.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double intensity = (static_cast<double>(counter + 1)/(freq * time))*max_intensity;
      // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Brake intensity: " << intensity);
      Brake(intensity);
      brake_rate.sleep();
    }
  }
  else if (aggressivity == 1)
  {
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Brake Logistics Curve.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double multiplier = 2;
      double x = (static_cast<double>(counter + 1)/(freq * time)) * time * multiplier;
      double intensity =  max_intensity / (1 + exp(-x)) - max_intensity/2;
      // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Brake intensity: " << intensity);
      Brake(intensity);
      brake_rate.sleep();
    }
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Brake Full.");
    Brake(max_intensity);
    ros::Duration(time).sleep();
  }
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service SRCP2 Brake. Engaged? " << flag_brake_engaged);
}

void SmHauler::Drive(double speed_ratio, double time)
{
  driving_tools::MoveForward srv_drive;

  if (pitch_ > 0.0 && speed_ratio < 0.0) {
   speed_ratio = speed_ratio - (pitch_ / 0.52) * 0.3;
  }
  srv_drive.request.speed_ratio = speed_ratio;


  if (clt_drive.call(srv_drive))
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Drive");
    while(ros::Time::now() - start_time < ros::Duration(time))
    {
      std_msgs::Int64 mode;
      mode.data = 2;
      driving_mode_pub.publish(mode);
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service Drive");
  }
}


void SmHauler::DriveCmdVel(double vx, double vy, double wz, double time)
{
  if (pitch_ > 0.0 && vx < 0.0)
  {
   vx = vx + (pitch_ / 0.52) * vx;
   if (vx < -0.7){
     vx = -0.7;
   }
  }
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = wz;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 20 seconds
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Driving (DriveCmdVel).");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub.publish(cmd_vel);
  }
}

void SmHauler::CommandCamera(double yaw, double pitch, double time)
{
  std_msgs::Float64 cmd_yaw;
  std_msgs::Float64 cmd_pitch;

  cmd_yaw.data = yaw;
  cmd_pitch.data = pitch;
  cmd_sensor_yaw_pub.publish(cmd_yaw);
  cmd_sensor_pitch_pub.publish(cmd_pitch);
  ros::Duration(time).sleep();
}

void SmHauler::ExecuteShakeBin(double time)
{
  std_msgs::Float64 bin_pitch;


  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Shaking the bin!");

  for (int i = 0; i < 5; i++)
  {
    bin_pitch.data = 0.05;
    cmd_bin_pub.publish(bin_pitch);
    bin_pitch.data = 0.0;
    ros::Duration(time/5.0).sleep();
    cmd_bin_pub.publish(bin_pitch);
  }
}

void SmHauler::RoverStatic(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service RoverStatic. Turned on? " << (int) flag);
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service RoverStatic");
  }

}

void SmHauler::CheckWaypoint(int max_count)
{
  waypoint_checker::CheckCollision srv_wp_check;
  bool is_colliding = true;
  int counter = 0;
  while (is_colliding && counter < max_count)
  {
    srv_wp_check.request.x  = goal_pose_.position.x;
    srv_wp_check.request.y = goal_pose_.position.y;
    if (clt_waypoint_checker.call(srv_wp_check))
    {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if(is_colliding)
      {
        CancelMoveBaseGoal();

        BrakeRamp(100, 3.0, 0);

        Brake (0.0);

        flag_interrupt_plan = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service Waypoint Checker");
    }
    ros::spinOnce();
    counter=counter+1;
  }
}

bool SmHauler::GoToWaypoint(double distance_threshold, double y_offset_b)
{
  waypoint_nav::GoToGoal srv_gotoGoal;
  srv_gotoGoal.request.start = true;
  srv_gotoGoal.request.goal.position = partner_excavator_location_;
  srv_gotoGoal.request.side = partner_excavation_status_.parking_side.data;
  srv_gotoGoal.request.thresh = distance_threshold;
  srv_gotoGoal.request.y_offset_b = y_offset_b;
  srv_gotoGoal.request.timeOut = 30;

  if(clt_go_to_goal.call(srv_gotoGoal))
  {
    if (srv_gotoGoal.response.success)
    {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Parked near the excavator with GoToWaypoint.");
      return true;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Failed to park near excavator with GoToWaypoint.");
  }
  return false;
}

bool SmHauler::ApproachChargingStation(int max_count)
{
  src2_approach_services::ApproachChargingStation srv_approach_charging_station;
  srv_approach_charging_station.request.approach_charging_station.data = true;

  bool success = false;

  int count = 0;
  while(!success && count < max_count)
  {
    if (clt_approach_base.call(srv_approach_charging_station))
    {
      if(srv_approach_charging_station.response.success.data)
      {
        success = true;
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"ApproachChargingStation with classifier successful");
      }
      else
      {
        success = false;
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ApproachChargingStation with classifier NOT successful");
      }
    }
    else
    {
      success = false;
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call ApproachChargingStation service");
    }
    count = count + 1;
  }

  return success;
}

bool SmHauler::ApproachExcavator(int max_count, double distance_threshold)
{
  src2_approach_services::ApproachExcavator srv_approach_excavator;
  srv_approach_excavator.request.approach_excavator.data= true;
  srv_approach_excavator.request.distance_threshold.data= distance_threshold;
  bool success = false;
  int count = 0;
  while(!success && count<max_count)
  {
    if (clt_approach_excavator.call(srv_approach_excavator))
    {
      if(srv_approach_excavator.response.success.data)
      {
        success = true;
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. ApproachExcavator with classifier successful");
      }
      else
      {
        success = false;
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. ApproachExcavator with classifier NOT successful");
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Failed to call ApproachExcavator service");
    }
    count = count + 1;
  }
  geometry_msgs::PointStamped excavator_location = srv_approach_excavator.response.point;
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Excavator position estimated with Obj Detection. Point: " << excavator_location.point);

  return success;
}

bool SmHauler::FindBin()
{
  src2_approach_services::FindBin srv_find_bin;
  srv_find_bin.request.data.data = true;
  bool success = true;
  if (clt_find_bin.call(srv_find_bin))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service FindBin.");
    success = srv_find_bin.response.success.data;
    if(success)
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Not necessary to use an intermediate waypoint before dumping.");
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Sending an intermediate waypoint before dumping.");
    }
    front_of_bin_location_.x = srv_find_bin.response.x.data;
    front_of_bin_location_.y = srv_find_bin.response.y.data;
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call ApproachBin service");
  }

  return success;
}



bool SmHauler::ApproachBin(int max_count)
{
  src2_approach_services::ApproachBin srv_approach_bin;
  srv_approach_bin.request.approach_bin.data= true;
  bool success = false;
  int count = 0;
  while(!success && count < max_count)
  {
    if (clt_approach_bin.call(srv_approach_bin))
    {
      if(srv_approach_bin.response.success.data)
      {
        success = true;
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"ApproachBin with classifier successful");
      }
      else
      {
        success = false;
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ApproachBin with classifier NOT successful");
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call ApproachBin service");
    }
    count = count + 1;
  }

  return success;
}


bool SmHauler::LocateExcavator()
{
  range_to_base::LocationOfExcavator srv_location_of_excavator;
  srv_location_of_excavator.request.location_of_excavator.data=true;
  srv_location_of_excavator.request.center = -pitch_-.1;
  srv_location_of_excavator.request.offset = 0.025;

  bool success = false;

  if(clt_location_of_excavator.call(srv_location_of_excavator))
  {
    if(srv_location_of_excavator.response.success.data)
    {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Location of excavator reliable");
      partner_excavator_location_ = srv_location_of_excavator.response.excavator_location;
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Saving Excavator Location: " << partner_excavator_location_);

      success = true;
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Location of excavator not reliable");
    //  partner_excavator_location_ = srv_location_of_excavator.response.excavator_location;
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Estimated Bad Excavator Location: "<< srv_location_of_excavator.response.excavator_location);
      success = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Failed to call LocateExcavator service");
    success = false;
  }

  return success;
}

void SmHauler::CheckForCollision()
{
  ros::spinOnce();
  double D_proc_plant = hypot(x_proc_plant_ - current_pose_.position.x, y_proc_plant_ - current_pose_.position.y);
  bool flag_collision_proc_plant = D_proc_plant < 4.2;

  double D_repair_station = hypot(x_repair_station_ - current_pose_.position.x, y_repair_station_ - current_pose_.position.y);
  bool flag_collision_repair_station = D_repair_station < 4.2;

  if (flag_collision_proc_plant || flag_collision_repair_station)
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Detected that localization estimate is in collision. Setting up Homing.");
    flag_interrupt_plan = false;
    flag_emergency = false;
    flag_arrived_at_waypoint = true;
    flag_recovering_localization = true;
    flag_localizing_volatile = false;
    flag_dumping = false;
  }
}

void SmHauler::ResetPosition()
{
  sensor_fusion::ResetPosition srv_reset_position;
  geometry_msgs::Point reset_position;
  reset_position.x = 0.0;
  reset_position.y = 0.0;
  reset_position.z = 0.0;
  srv_reset_position.request.new_position = reset_position;

  if (clt_reset_position.call(srv_reset_position))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service ResetPosition.");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call ResetPosition Service.");
  }
}

bool SmHauler::RequestDumping(bool dump_request)
{
  task_planning::DumpCoordination srv_dump_coordination;
  srv_dump_coordination.request.dump_request = dump_request;

  if (dump_request)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Requesting to start Dumping.");
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Marking Dumping as complete.");
  }

  if (clt_dump_coordination.call(srv_dump_coordination))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service DumpCoordination.");

    if (srv_dump_coordination.response.request_accepted)
    {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Request ACCEPTED. Starting to Dump.");
      return true;
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Request -Wait for it- DENIED.");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call DumpCoordination Service.");
  }
  return false;
}

bool SmHauler::FindExcavator(double timeout)
{
  move_excavator::FindExcavator srv_find;

  srv_find.request.timeLimit = timeout;
  srv_find.request.side = partner_excavation_status_.parking_side.data;

  if (clt_find_excavator.call(srv_find))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation. Called service FindExcavator.");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Failed to call service FindExcavator.");
  }

  if (srv_find.response.success)
  {
    geometry_msgs::PointStamped bucket_point = srv_find.response.target;
    bucket_point.point.z -= 0.0;

    camera_link_to_odom = tf_buffer.lookupTransform(robot_name_+"_odom", robot_name_+"_left_camera_optical", ros::Time(0), ros::Duration(1.0));

    tf2::doTransform(bucket_point, bucket_point_, camera_link_to_odom);

    partner_excavator_location_ = bucket_point_.point;

    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Got the position of the Excavator's bucket. Point:" << partner_excavator_location_);

    CommandCamera(0,0,2);

    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation. Havent found the Excavator's bucket.");  // Look forward before starting to move again

    CommandCamera(0,0,2);

    return false;
  }
}

bool SmHauler::LocateBin()
{
  range_to_base::LocationOfBin srv_location_of_bin;
  srv_location_of_bin.request.location_of_bin.data=true;

  bool success = false;

  if(clt_location_of_bin.call(srv_location_of_bin))
  {
    if(srv_location_of_bin.response.success.data)
    {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Location of bin reliable");
      success = true;
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Location of bin not reliable");
      success = false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call LocateBin service");
    success = false;
  }

  return success;
}

bool SmHauler::HomingUpdate(bool init_landmark)
{
  sensor_fusion::HomingUpdate srv_homing;
  srv_homing.request.angle = pitch_ - 0.4; // pitch up is negative number
  srv_homing.request.initializeLandmark = init_landmark;

  bool success = false;

  if (clt_homing.call(srv_homing))
  {
    if(init_landmark && srv_homing.response.success)
    {
      base_location_ = srv_homing.response.base_location;
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Homing [Init] successful.");
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Saving Base Location "<<base_location_.x << "," << base_location_.y);
      success = true;
    }
    else if (!init_landmark && srv_homing.response.success)
    {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Homing [Update] successful.");
      success = true;
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Homing NOT successful.");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call Homing Service.");
  }
  return success;
}

bool SmHauler::HomingUpdateProcessingPlant()
{
  sensor_fusion::HomingUpdateProcessingPlant srv_homing_proc_plant;
  srv_homing_proc_plant.request.angle = pitch_ - 0.4; // pitch up is negative number
  srv_homing_proc_plant.request.initializeLandmark = false;

  bool success = false;

  if (clt_homing_proc_plant.call(srv_homing_proc_plant))
  {
    if (srv_homing_proc_plant.response.success)
    {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Homing [Update] successful.");
      success = true;
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Homing NOT successful.");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call Homing Service.");
  }
  return success;
}

void SmHauler::PublishHaulerStatus()
{
  state_machine::HaulerStatus msg;
  msg.hauler_id.data = robot_id_;
  msg.approaching_side.data = flag_approaching_side;
  msg.approached_side.data = flag_approached_side;
  msg.approached_excavator.data = flag_approached_excavator;
  msg.located_excavator.data = flag_located_excavator;
  msg.parked_hauler.data = flag_parked_hauler;
  msg.bin_full.data = flag_full_bin;
  msg.parking_recovery_counter.data = parking_recovery_counter_;
  msg.dumped.data = flag_dumped;

  hauler_status_pub.publish(msg);

  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Excavation. Publishing Hauler Status.");
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Hauler Status. hauler_id :" << (int) msg.hauler_id.data
                                              << ", approaching_side:" << (int) msg.approaching_side.data
                                              << ", approached_side:" << (int) msg.approached_side.data
                                              << ", approached_excavator:" << (int) msg.approached_excavator.data
                                              << ", located_excavator:" << (int) msg.located_excavator.data
                                              << ", parked_hauler:" << (int) msg.parked_hauler.data
                                              << ", bin_full:" << (int) msg.bin_full.data
                                              << ", parking_recovery_counter:" << (int) msg.parking_recovery_counter.data
                                              << ", dumped:" << (int) msg.dumped.data);
}

void SmHauler::Plan()
{
  task_planning::PlanInfo srv_plan;
  // if (!flag_interrupt_plan)
  // {
    srv_plan.request.replan.data = true;
  // }
  // else
  // {
  //   srv_plan.request.replan.data = false;
  // }
  srv_plan.request.type.data = (uint8_t) mac::HAULER;
  srv_plan.request.id.data = (uint8_t) robot_id_;
  // ROS_WARN_STREAM("[" << robot_name_ << "] " << "Planning. robot type: " <<  (int) srv_plan.request.type.data << ", id " << (int) srv_plan.request.id.data );

  if (clt_task_planning.call(srv_plan))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service Plan");
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call Plan service");
  }

  if (srv_plan.response.code.data !=255)
  {
    goal_pose_.position = srv_plan.response.objective.point;
    geometry_msgs::Quaternion quat;
    goal_pose_.orientation = quat;
    lost_recovery_counter_ = 0;
    // partner_excavator_id_ = srv_plan.response.id;
    no_objective = false;
  }
  else
  {
    no_objective = true;
    flag_interrupt_plan = false;
  }

  prev_srv_plan = srv_plan;

  switch (srv_plan.response.code.data)
  {
    case _initialize:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Initialize");
      // flag_have_true_pose = false;
      break;

    case _planning:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Planning");
      flag_interrupt_plan = true;
      flag_emergency = false;
      flag_arrived_at_waypoint = true;
      flag_recovering_localization = false;
      flag_localizing_volatile = false;
      break;

    case _traverse:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Traverse");
      flag_interrupt_plan = false;
      flag_emergency = false;
      flag_arrived_at_waypoint = false;
      flag_recovering_localization = false;
      flag_localizing_volatile = false;
      flag_dumping = false;
      break;

    case _volatile_handler:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Vol Handler");
      flag_interrupt_plan = false;
      flag_emergency = false;
      flag_arrived_at_waypoint = false;
      flag_recovering_localization = false;
      flag_localizing_volatile = true;
      flag_dumping = false;
      break;

    case _lost:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Homing");
      flag_interrupt_plan = false;
      flag_emergency = false;
      flag_arrived_at_waypoint = false;
      flag_recovering_localization = true;
      flag_localizing_volatile = false;
      flag_dumping = false;
      break;

    case _emergency:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Emergency");
      flag_interrupt_plan = false;
      flag_emergency = true;
      flag_arrived_at_waypoint = false;
      flag_recovering_localization = true;
      flag_localizing_volatile = false;
      flag_dumping = false;
      break;

    case _hauler_dumping:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Dumping");
      flag_interrupt_plan = false;
      flag_emergency = false;
      flag_arrived_at_waypoint = false;
      flag_recovering_localization = false;
      flag_localizing_volatile = false;
      flag_dumping = true;
      break;

    default:
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Nothing to do.");
      flag_interrupt_plan = true;
      flag_emergency = false;
      flag_arrived_at_waypoint = true;
      flag_recovering_localization = false;
      flag_localizing_volatile = false;
      flag_dumping = false;
      break;
  }

  // srv_plan.response.id;
  // flag_interrupt_plan = false;
}


//------------------------------------------------------------------------------------------------------------------------
