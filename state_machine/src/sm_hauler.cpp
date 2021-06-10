#include <state_machine/sm_hauler.hpp>


SmHauler::SmHauler() :
ac("move_base", true),
tf2_listener(tf_buffer),
move_base_state(actionlib::SimpleClientGoalState::PREEMPTED)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_status_pub = nh.advertise<state_machine::RobotStatus>("state_machine/status", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("driving/cmd_vel", 1);
  driving_mode_pub = nh.advertise<std_msgs::Int64>("driving/driving_mode", 1);
  cmd_dump_pub = nh.advertise<std_msgs::Float64>("bin/command/position", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmHauler::localizedBaseCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmHauler::localizationCallback, this);
  driving_mode_sub = nh.subscribe("driving/driving_mode",1, &SmHauler::drivingModeCallback, this);
  laser_scan_sub = nh.subscribe("laser/scan",1, &SmHauler::laserCallback, this);
  planner_interrupt_sub = nh.subscribe("/planner_interrupt", 1, &SmHauler::plannerInterruptCallback, this);

  // Clients
  clt_stop = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_rip = nh.serviceClient<driving_tools::RotateInPlace>("driving/rotate_in_place");
  clt_drive = nh.serviceClient<driving_tools::MoveForward>("driving/move_forward");
  clt_lights = nh.serviceClient<srcp2_msgs::SpotLightSrv>("spot_light");
  clt_brake = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_approach_base = nh.serviceClient<src2_approach_services::ApproachChargingStation>("approach_charging_station_service");
  clt_rover_static = nh.serviceClient<sensor_fusion::RoverStatic>("sensor_fusion/toggle_rover_static");
  clt_homing = nh.serviceClient<sensor_fusion::HomingUpdate>("homing");
  clt_sf_true_pose = nh.serviceClient<sensor_fusion::GetTruePose>("true_pose");
  clt_waypoint_checker = nh.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  clt_srcp2_brake_rover= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_task_planning = nh.serviceClient<task_planning::PlanInfo>("/task_planner_exc_haul");
  clt_approach_excavator = nh.serviceClient<src2_approach_services::ApproachExcavator>("approach_excavator_service");
  clt_approach_bin = nh.serviceClient<src2_approach_services::ApproachBin>("approach_bin_service");
  clt_location_of_bin = nh.serviceClient<range_to_base::LocationOfBin>("location_of_bin_service");

  map_timer = ros::Time::now();
  wp_checker_timer=  ros::Time::now();
  laser_collision_timer = ros::Time::now();

  // Local copy of the processing plant location
  proc_plant_bin_location_.x = 13.5;
  proc_plant_bin_location_.y = 9.0;
  proc_plant_bin_location_.z = 0.0;

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

}

void SmHauler::run()
{
  ros::Rate loop_rate(5); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_have_true_pose: " << (int)flag_have_true_pose);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_interrupt_plan: " << (int)flag_interrupt_plan);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_arrived_at_waypoint: " << (int)flag_arrived_at_waypoint);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_localizing_volatile: " << (int)flag_localizing_volatile);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_dumping: " << (int)flag_dumping);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_recovering_localization: " << (int)flag_recovering_localization);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_brake_engaged: " << (int)flag_brake_engaged);
    //---------------------------------------------------------------------------------------------------------------------

    // State machine truth table ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    state_to_exec.clear();
    state_to_exec.resize(num_states,0);

    if(!flag_have_true_pose)
    {
      state_to_exec.at(_initialize) = 1;
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
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Initialization State!\n");

  while (!clt_lights.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for Lights");
  }

  while (!clt_approach_base.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachChargingStation service");
  }

  while (!clt_sf_true_pose.waitForExistence())
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waiting for TruePose service");
  }

  while (!clt_approach_excavator.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachExacavator service");
  }

  while (!clt_approach_bin.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachExacavator service");
  }

  double progress = 0;

  Lights(20);

  if(robot_id_==1)
  {
    flag_need_init_landmark = true;

    bool approachSuccess = ApproachChargingStation(3);

    BrakeRamp(100, 3, 0);

    RoverStatic(true);

    GetTruePose();

    RoverStatic(false);

    if(approachSuccess)
    {
      bool homingSuccess = HomingUpdate(flag_need_init_landmark);
      if (homingSuccess)
      {
        progress = 1.0;
      }
      else
      {
        progress = -1.0;
      }
    }
    else
    {
      progress = -1.0;
      // TODO: SOMETHING
    }

    //Similar to initial homing, keep the localization good after homing.
    Brake(0.0);

    DriveCmdVel(-0.5,0.0,0.0,5);

    BrakeRamp(100, 1, 0);

    Brake(0.0);

    RotateInPlace(0.2, 3);

    BrakeRamp(100, 1, 0);

    Brake(0.0);

  }

  if(robot_id_ == 2)
  {
    Stop(2.0);

    Brake(100.0);

    RoverStatic(true);

    GetTruePose();

    RoverStatic(false);

    progress = 1.0;
  }

  ClearCostmaps();

  Brake(0.0);

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _initialize;
  sm_status_pub.publish(status_msg);
}

void SmHauler::statePlanning()
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Planning!\n");

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  Plan();

  goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

  Brake (0.0);

  RotateToHeading(goal_yaw_);

  BrakeRamp(100, 3.0, 0);

  CheckWaypoint(3);

  ClearCostmaps();

  BrakeRamp(100, 2, 0);

  Brake(0.0);

  if (!no_objective)
  {
    move_base_msgs::MoveBaseGoal move_base_goal;
    ac.waitForServer();
    setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Sending goal to MoveBase: " << move_base_goal);
    waypoint_timer = ros::Time::now();
    ac.sendGoal(move_base_goal, boost::bind(&SmHauler::doneCallback, this,_1,_2), boost::bind(&SmHauler::activeCallback, this), boost::bind(&SmHauler::feedbackCallback, this,_1));
    ac.waitForResult(ros::Duration(0.25));
    // flag_arrived_at_waypoint = false;
    // flag_localizing_volatile = true;
  }
  else
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"No objective\n");
  }

  double progress = 0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _planning;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateTraverse()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Traverse State\n");

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 6.0) // CHANGE TO 6.0
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Close to goal, getting new waypoint.");
    flag_arrived_at_waypoint = true;
  }

  ros::Duration timeOutWPCheck(3.0);
  if (ros::Time::now() - wp_checker_timer > timeOutWPCheck) {
    bool is_colliding = false;
    waypoint_checker::CheckCollision srv_wp_check;
    if (clt_waypoint_checker.call(srv_wp_check)) {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if (is_colliding) {
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Waypoint Unreachable. Sending to Planning");
        // flag_waypoint_unreachable = true;
      }
    }
    wp_checker_timer = ros::Time::now();
  }

  move_base_state = ac.getState();
  int mb_state =(int) move_base_state.state_;
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"MoveBase status: "<< mb_state);

  if(mb_state==5 || mb_state==7)
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"MoveBase has failed to make itself useful.");
    // flag_waypoint_unreachable= true;

    Stop (1.0);

    ClearCostmaps();

    BrakeRamp(100, 2, 0);

    Brake(0.0);
  }

  ros::Duration timeoutMap(90.0);

  if (ros::Time::now() - map_timer > timeoutMap)
  {
    std_srvs::Empty emptymsg;
    ros::service::call("move_base/clear_costmaps",emptymsg);
    map_timer =ros::Time::now();
    BrakeRamp(100, 2, 0); // Give more time
    Brake(0.0);

    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Rover is stopped to clear the Map");
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Move Base State: "<< mb_state);
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Map Cleared");
  }

  ros::Duration timeoutWaypoint(120);
  if (ros::Time::now() - waypoint_timer > timeoutWaypoint )
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waypoint Unreachable");
    // flag_waypoint_unreachable= true;
    Stop (1.0);
    ClearCostmaps();
    BrakeRamp(100, 2, 0);
    Brake(0.0);
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(1,"Remaining Time for Waypoint" << timeoutWaypoint - (ros::Time::now() - waypoint_timer));
  }

  double progress = 0;
  progress = distance_to_goal;

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _traverse;
  sm_status_pub.publish(status_msg);

}

void SmHauler::stateVolatileHandler()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Volatile Handling State!");

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  double progress = 0.0;
  flag_full_bin = false;

  // TODO: Get feedback from Excavator before approaching
  // Include callback to ExcavationStatus
  // According with ExcavationStatus we approach and park

  if(flag_approach_excavator)
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"STARTING APPROACH EXCAVATOR");

    bool approachSuccess = ApproachExcavator(3);

    if (approachSuccess)
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"APPROACH SUCCESS + CHANGING FLAGS ***********");

      // TODO: This is where we are gonna call the parallel parking
      // Include new method: SmHauler::ParallelParking that calls the service
      progress = 1.0;
      flag_approach_excavator = false;
      flag_full_bin = true;
    }
    else
    {
      // TODO: What do we do if approach fails?
      // Maybe send a new waypoint?
      progress = -1.0;
    }
  }

  // TODO: choose when to transition to dumping
  // We need to check what is a good condition. Some possibilities:
  // Five dumps?
  // Node that checks the srcp2_score
  // Node that checks the amount of material in the bin with the camera

  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"FULL BIN");
  flag_full_bin = true;
  Brake(0.0);
  if(flag_full_bin)
  {
    flag_interrupt_plan = false;
    flag_recovering_localization = false;
    flag_localizing_volatile = false;
    flag_arrived_at_waypoint = false;
    flag_dumping = true;

    goal_pose_.position = proc_plant_bin_location_;
    move_base_msgs::MoveBaseGoal move_base_goal;
    ac.waitForServer();
    setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Sending goal to MoveBase: " << move_base_goal);
    waypoint_timer = ros::Time::now();
    ac.sendGoal(move_base_goal, boost::bind(&SmHauler::doneCallback, this,_1,_2), boost::bind(&SmHauler::activeCallback, this), boost::bind(&SmHauler::feedbackCallback, this,_1));
    ac.waitForResult(ros::Duration(0.25));
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _volatile_handler;
  sm_status_pub.publish(status_msg);
}
void SmHauler::stateLost()
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"LOST STATE!\n");

  double progress = 1.0;

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  Stop (2.0);

  bool approachSuccess = ApproachChargingStation(3);

  BrakeRamp(100, 3, 0);

  if(approachSuccess)
  {
    bool homingSuccess = HomingUpdate(flag_need_init_landmark);
    if (homingSuccess)
    {
      progress = 1.0;
      flag_recovering_localization = false;
    }
    else
    {
      progress = -1.0;
    }
  }
  else
  {
    progress = -1.0;
    // TODO: SOMETHING
  }

  Brake(0.0);

  DriveCmdVel(-0.5,0.0,0.0,5);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  RotateInPlace(0.2, 3);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  ClearCostmaps();

  BrakeRamp(100, 2, 0);

  Brake(0.0);

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _lost;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateDump()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"DUMPING STATE!\n");

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  double progress = 0.0;

  bool approachSuccess = ApproachBin(3);

  // localize bin after approaching bin
  if (approachSuccess)
  {
    bool locateBinSuccess = LocateBin();
    if (locateBinSuccess)
    {
      progress = 1.0;
      flag_localizing_volatile = false;
      flag_dumping = false;

      Brake(0.0);

      DriveCmdVel(-0.5,0.0,0.0,5);

      BrakeRamp(100, 3, 0);

      Brake(0.0);

      RotateInPlace(0.2, 3);

      BrakeRamp(100, 3, 0);

      Brake(0.0);

      ClearCostmaps();

      BrakeRamp(100, 2, 0);

      Brake(0.0);

    }

  }
  else
  {
    progress = -1.0;
  }

  // approach closely
  // compare bin location w/ current location
  //  current_pose_
  // driving commands
  // call bin location + verify
  // dump action

  // double distx = current_pose_.position.x - srv_location_of_bin.response.position.x;
  // double disty = current_pose_.position.y - srv_location_of_bin.response.position.y;
  //
  // double dist  = std::pow((distx*distx + disty*disty), 0.5);
  //
  //
  // double dump_thresh = 0.5;
  //
  // if (dist < dump_thresh){

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
  if (flag_localized_base) {
    ROS_WARN_STREAM_ONCE("Initial Localization Successful = " << (int)flag_localized_base);

  }
  else {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Waiting for Initial Localization  = " << (int)flag_localized_base);
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
  ROS_INFO_STREAM_THROTTLE(2,"Minimum range average: " << min_range);

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
    immobilityRecovery(2);
  }
}

void SmHauler::setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
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

void SmHauler::plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg)
{
  task_planning::PlanInfo srv_plan;
  srv_plan.request.replan.data = false;
  srv_plan.request.type.data = mac::SCOUT;
  srv_plan.request.id.data = robot_id_;

  if (clt_task_planning.call(srv_plan))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service Plan");
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call service RotateInPlace");
  }

  if(!(prev_srv_plan.response.objective.point.x == srv_plan.response.objective.point.x &&
  prev_srv_plan.response.objective.point.y == srv_plan.response.objective.point.y &&
  prev_srv_plan.response.code == srv_plan.response.code))
  {
    flag_interrupt_plan = true;
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
  ros::Duration timeoutHeading(10.0); // Timeout of 20 seconds

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

     Stop(2.0);

     DriveCmdVel (-0.5, 0.0, 0.0, 4.0);

     BrakeRamp(100, 3, 0);

     Brake(0.0);
     // Stop(2.0);

  //  immobilityRecovery(); //TODO: Use this instead of Stop and Drive at line 714 and 716

    flag_heading_fail=false;
  }
  else
  {
    Stop(0.0);
  }
}

void SmHauler::homingRecovery()
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Starting Homing Recovery.");

  Lights(20);

  Stop(2.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  // Drive(-0.3, 3.0);
  DriveCmdVel(-0.3,-0.6, 0.0, 5.0);

  Stop(0.0);

  DriveCmdVel(0.0,0.0,-0.25,4.0);

  Stop(0.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  DriveCmdVel(0.6,0.0,0.0,4.5);

  Stop(0.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

}

void SmHauler::immobilityRecovery(int type)
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Starting Recovery.");

  Stop(2.0);

  Brake(100.0);

  Brake(0.0);

  DriveCmdVel(-0.5,0.0,0.0,3.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  DriveCmdVel(-0.3,-0.6, 0.0, 4.0);

  // Stop(3.0); //TODO: CMDvelZero try

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  // flag_mobility=true;

  // flag_waypoint_unreachable=true;


}

void SmHauler::ClearCostmaps()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed calling clear_costmaps service.");
  }
}

void SmHauler::GetTruePose()
{
  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose.call(srv_sf_true_pose))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service TruePose");
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Status of SF True Pose: "<< (int) srv_sf_true_pose.response.success);
    flag_have_true_pose = true;
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
  driving_tools::Stop srv_stop;
  srv_stop.request.enable  = true;
  if (clt_stop.call(srv_stop))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service Stop");
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
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service Brake");
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
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Brake FULL.");
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
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Drive Cmd Vel publisher.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub.publish(cmd_vel);
  }
}

void SmHauler::RoverStatic(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static.call(srv_rover_static))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service RoverStatic. Turned on? " << flag);
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
        Plan();

        goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

        Brake (0.0);

        RotateToHeading(goal_yaw_);

        BrakeRamp(100, 3.0, 0);

        Brake (0.0);
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
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"ApproachChargingStation with classifier successful");
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

bool SmHauler::ApproachExcavator(int max_count)
{
  src2_approach_services::ApproachExcavator srv_approach_excavator;
  srv_approach_excavator.request.approach_excavator.data= true;
  bool success = false;
  int count = 0;
  while(!success && count<max_count)
  {
    if (clt_approach_excavator.call(srv_approach_excavator))
    {
      if(srv_approach_excavator.response.success.data)
      {
        success = true;
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"ApproachExcavator with classifier successful");
      }
      else
      {
        success = false;
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ApproachExcavator with classifier NOT successful");
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call ApproachExcavator service");
    }
    count = count + 1;
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
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Homing NOT SUCCESSFUL.");
    }
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call Homing Service.");
  }
  return success;
}

void SmHauler::Plan()
{
  task_planning::PlanInfo srv_plan;
  if (!  flag_interrupt_plan)
  {
    srv_plan.request.replan.data = true;
  }
  else
  {
    srv_plan.request.replan.data = false;
  }
  srv_plan.request.type.data = mac::HAULER;
  srv_plan.request.id.data = robot_id_;

  if (clt_task_planning.call(srv_plan))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service Plan");
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call service RotateInPlace");
  }

  if (srv_plan.response.code.data !=-1)
  {
    goal_pose_.position = srv_plan.response.objective.point;
    geometry_msgs::Quaternion quat;
    goal_pose_.orientation = quat;
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
    flag_have_true_pose = false;
    break;

  case _planning:
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Planning");
    flag_interrupt_plan = true;
    flag_arrived_at_waypoint = true;
    flag_recovering_localization = false;
    flag_localizing_volatile = false;
    break;

  case _traverse:
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Traverse");
    flag_interrupt_plan = false;
    flag_arrived_at_waypoint = false;
    flag_recovering_localization = false;
    flag_localizing_volatile = false;
    break;

  case _volatile_handler:
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Vol Handler");
    flag_interrupt_plan = false;
    flag_arrived_at_waypoint = false;
    flag_recovering_localization = false;
    flag_localizing_volatile = true;
    break;

  case _lost:
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: Homing");
    flag_interrupt_plan = false;
    flag_arrived_at_waypoint = false;
    flag_recovering_localization = true;
    flag_localizing_volatile = false;
    break;

  default:
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Task Planner: No idea what Im doing");
    flag_interrupt_plan = true;
    flag_arrived_at_waypoint = true;
    flag_recovering_localization = false;
    flag_localizing_volatile = false;
    break;
  }

  // srv_plan.response.id;
  // flag_interrupt_plan = false;
}

//------------------------------------------------------------------------------------------------------------------------
