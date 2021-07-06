#include <state_machine/sm_scout.hpp>

SmScout::SmScout() :
ac("move_base", true),
move_base_state_(actionlib::SimpleClientGoalState::PREEMPTED)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_status_pub = nh.advertise<state_machine::RobotStatus>("state_machine/status", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("driving/cmd_vel", 1);
  driving_mode_pub = nh.advertise<std_msgs::Int64>("driving/driving_mode", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmScout::localizedBaseCallback, this);
  volatile_sensor_sub = nh.subscribe("volatile_sensor", 1, &SmScout::volatileSensorCallback, this);
  volatile_cmd_sub = nh.subscribe("volatile_map/cmd", 1, &SmScout::volatileCmdCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmScout::localizationCallback, this);
  driving_mode_sub =nh.subscribe("driving/driving_mode_",1, &SmScout::drivingModeCallback, this);
  laser_scan_sub =nh.subscribe("laser/scan",1, &SmScout::laserCallback, this);
  planner_interrupt_sub = nh.subscribe("/planner_interrupt", 1, &SmScout::plannerInterruptCallback, this);
  system_monitor_sub =nh.subscribe("system_monitor",1, &SmScout::systemMonitorCallback, this);

  // Clients
  clt_stop = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_rip = nh.serviceClient<driving_tools::RotateInPlace>("driving/rotate_in_place");
  clt_move_side = nh.serviceClient<driving_tools::MoveSideways>("driving/move_sideways");
  clt_turn_wheels_side = nh.serviceClient<driving_tools::TurnWheelsSideways>("driving/turn_wheels_sideways");
  clt_drive = nh.serviceClient<driving_tools::MoveForward>("driving/move_forward");
  clt_lights = nh.serviceClient<srcp2_msgs::SpotLightSrv>("spot_light");
  clt_brake = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_approach_base = nh.serviceClient<src2_approach_services::ApproachChargingStation>("approach_charging_station_service");
  clt_rover_static = nh.serviceClient<sensor_fusion::RoverStatic>("sensor_fusion/toggle_rover_static");
  clt_homing = nh.serviceClient<sensor_fusion::HomingUpdate>("homing");
  clt_sf_true_pose = nh.serviceClient<sensor_fusion::GetTruePose>("true_pose");
  clt_waypoint_checker = nh.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  clt_srcp2_brake_rover = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_task_planning = nh.serviceClient<task_planning::PlanInfo>("/task_planner_scout");

  map_timer = ros::Time::now();
  wp_checker_timer =  ros::Time::now();
  laser_collision_timer = ros::Time::now();

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

void SmScout::run()
{
  ros::Rate loop_rate(5); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_have_true_pose: " << (int)flag_have_true_pose);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_interrupt_plan: " << (int)flag_interrupt_plan);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_arrived_at_waypoint: " << (int)flag_arrived_at_waypoint);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_localizing_volatile: " << (int)flag_localizing_volatile);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_recovering_localization: " << (int)flag_recovering_localization);
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_brake_engaged: " << (int)flag_brake_engaged);
    //---------------------------------------------------------------------------------------------------------------------

    // State machine truth table ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    state_to_exec.clear();
    state_to_exec.resize(num_states,0);
    if(!flag_have_true_pose)
    {
      state_to_exec.at(_initialize) = 1;
    }
    else if(flag_interrupt_plan || (flag_arrived_at_waypoint && !flag_recovering_localization && !flag_localizing_volatile && !flag_brake_engaged))
    {
      state_to_exec.at(_planning) = 1;
    }
    else if(flag_localizing_volatile && !flag_brake_engaged) // Removed flag_volatile_detected
    {
      state_to_exec.at(_volatile_handler) = 1;
    }
    else if(flag_arrived_at_waypoint && flag_recovering_localization && !flag_brake_engaged)
    {
      state_to_exec.at(_lost) = 1;
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
    else
    {
      ROS_FATAL("No state to execute");
      flag_fallthrough_condition = false;
    }
    // -------------------------------------------------------------------------------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// State function definitions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmScout::stateInitialize()
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

  double progress = 0;

  Lights(20);

  Stop(0.1);

  Brake(100.0);

  RoverStatic(true);

  GetTruePose();

  RoverStatic(false);

  ClearCostmaps(5.0);

  Brake(0.0);

  progress = 1.0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int) _initialize;
  sm_status_pub.publish(status_msg);
}

void SmScout::statePlanning()
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Planning!\n");

  double progress = 0.0;

  CancelMoveBaseGoal();

  Plan();

  if (!no_objective) 
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"New objective.");
    goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

    Brake (0.0);

    RotateToHeading(goal_yaw_);

    BrakeRamp(100, 1, 0);

    Brake(0.0);

    // CheckWaypoint(3); // TODO: Check if they needed

    ClearCostmaps(5.0);

    SetMoveBaseGoal();

    progress = 1.0;
  }
  else
  {
    ros::Duration(2).sleep();
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"No objective. Will ask again in 20s.");
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int)_planning;
  sm_status_pub.publish(status_msg);
}

void SmScout::stateTraverse()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Traverse State\n");
  move_base_state_ = ac.getState();
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"MoveBase status: "<< move_base_state_.toString());

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Close to goal, getting new waypoint.");
    flag_arrived_at_waypoint = true;
  }
  else
  {
    if(move_base_state_ == actionlib::SimpleClientGoalState::ABORTED || move_base_state_ == actionlib::SimpleClientGoalState::LOST)
    {   
      ROS_WARN_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"MoveBase status: "<< move_base_state_.toString());
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"MoveBase has failed to make itself useful.");

      flag_arrived_at_waypoint = true;
      flag_recovering_localization = false;
      flag_localizing_volatile = false;

      // ClearCostmaps(5.0); // TODO: Check if they needed

      Stop (0.1);

      BrakeRamp(100, 1, 0);

      Brake(0.0);
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


  double progress = 0;
  progress = distance_to_goal;

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int)  _traverse;
  sm_status_pub.publish(status_msg);
}

void SmScout::stateVolatileHandler()
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"VOLATILE HANDLING STATE!\n");

  CancelMoveBaseGoal();

  ros::Duration timeoutVolatileHandling(120);

  if (!flag_volatile_honed)
  {
    Stop(0.1);

    BrakeRamp(100, 0.1, 0);

    Brake(0.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Turning wheels sideways.");
    TurnWheelsSideways(true, 2.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Moving sideways (Right).");
    MoveSideways(0.1, 10.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Moving sideways (Left).");
    MoveSideways(-0.1, 20.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Moving sideways (Right).");
    MoveSideways(0.1, 10.0);

    flag_volatile_honed = true;
  }
  else
  {
    Stop(0.1);

    DriveCmdVel(1, 0, 0, 3);

    Stop(0.1);

    BrakeRamp(100, 1.0, 0);

    Brake(0.0);

    SetMoveBaseSpeed(SCOUT_MAX_SPEED);

    flag_volatile_honed = false;
    flag_localizing_volatile = false;
    flag_interrupt_plan = true;
  }

  double progress = 0.0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int)  _volatile_handler;
  sm_status_pub.publish(status_msg);
}

void SmScout::stateLost()
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"LOST STATE!\n");

  double progress = 0.0;

  CancelMoveBaseGoal();

  Stop (0.1);

  bool approachSuccess = ApproachChargingStation(3);

  BrakeRamp(100, 1, 0);

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

  Stop(0.1);

  BrakeRamp(100, 1, 0);

  Brake(0.0);

  RotateInPlace(0.2, 3);

  Stop(0.1);

  BrakeRamp(100, 1, 0);

  Brake(0.0);

  ClearCostmaps(5.0);

  BrakeRamp(100, 1, 0);

  Brake(0.0);

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _lost;
  sm_status_pub.publish(status_msg);
}

//------------------------------------------------------------------------------------------------------------------------

// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmScout::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = (bool) msg->data;
  if (flag_localized_base)
  {
    ROS_WARN_STREAM_ONCE("Initial Localization Successful = " << (int) flag_localized_base);
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Waiting for Initial Localization  = " << (int) flag_localized_base);
  }
}

void SmScout::systemMonitorCallback(const srcp2_msgs::SystemMonitorMsg::ConstPtr& msg)
{
  power_level_ = msg->power_level;
  power_rate_ = msg->power_rate;

  if (power_level_< 20) 
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " << "Power Level Warning: " << power_level_);
    if (power_level_< 10) 
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Rover Mulfunction! Critical Power: " << power_level_);
    }
  }
  // ROS_INFO_STREAM("[" << robot_name_ << "] " << "Power Rate: " << power_rate_);
}

void SmScout::volatileSensorCallback(const srcp2_msgs::VolSensorMsg::ConstPtr& msg)
{
  // Update detected distance variables
  prev_vol_detected_dist_ = vol_detected_dist_;
  vol_detected_dist_ = msg->distance_to;

  // Update minimum detected distance
  min_vol_detected_dist_ = (vol_detected_dist_ < min_vol_detected_dist_)? vol_detected_dist_: min_vol_detected_dist_;

  // If detected = -1, out of range, reset.
  min_vol_detected_dist_ = ((vol_detected_dist_+1) < 0.001)? 30 : min_vol_detected_dist_;

  if (msg->distance_to > 0)
  {
    flag_volatile_detected = true;
  }
  else
  {
    flag_volatile_detected = false;
  }

  if (min_vol_detected_dist_<30)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " << "Minimum detected volatile distance: " << min_vol_detected_dist_);
  }

}

void SmScout::volatileCmdCallback(const std_msgs::Int64::ConstPtr& msg)
{
  SetMoveBaseSpeed(0.1);

  // Update move_base max speed
  if (msg->data == 2)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Minimum trajectory distance detected.");
    flag_localizing_volatile = true;
  }
  else
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Volatile detected.");
  }

}

void SmScout::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_ =msg->data;
}

void SmScout::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev_ = yaw_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

  if (abs(pitch_ * 180 / M_PI) > 10) 
  {
    ROS_WARN_STREAM_THROTTLE(10, "Robot Climbing Up! Pitch: " << pitch_ * 180 / M_PI);
    if (curr_max_speed_ != SCOUT_MAX_SPEED/2)
    {
      SetMoveBaseSpeed(SCOUT_MAX_SPEED/2);
      curr_max_speed_ = SCOUT_MAX_SPEED/2;
    }

    if (abs(pitch_ * 180 / M_PI) > 27) 
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Robot Cant Climb! Pitch: " << pitch_ * 180 / M_PI);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Commanding IMMOBILITY.");
      CancelMoveBaseGoal();
      Stop(0.0);
      BrakeRamp(100,1,0);
      Brake(0.0);
      DriveCmdVel(-1.0,0.0,0.0,3);
      Stop(0.1);
      SetMoveBaseGoal();
    }
  }
  else
  {
    if (curr_max_speed_ != SCOUT_MAX_SPEED)
    {
      SetMoveBaseSpeed(SCOUT_MAX_SPEED);
      curr_max_speed_ = SCOUT_MAX_SPEED;
    }
  }
  
  if (abs(roll_ * 180 / M_PI) > 10) 
  {
    ROS_WARN_STREAM_THROTTLE(10, "Robot is Sideways! Roll: " << roll_ * 180 / M_PI);
    if (curr_max_speed_ != SCOUT_MAX_SPEED/2)
    {
      SetMoveBaseSpeed(SCOUT_MAX_SPEED/2);
      curr_max_speed_ = SCOUT_MAX_SPEED/2;
    }

    if (abs(roll_ * 180 / M_PI) > 27) 
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Robot Cant Climb! Roll: " << roll_ * 180 / M_PI);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Commanding IMMOBILITY.");
      CancelMoveBaseGoal();
      Stop(0.0);
      BrakeRamp(100,1,0);
      Brake(0.0);
      DriveCmdVel(-1.0,0.0,0.0,3);
      Stop(0.1);
      SetMoveBaseGoal();
    }
  }
  else
  {
    if (curr_max_speed_ != SCOUT_MAX_SPEED)
    {
      SetMoveBaseSpeed(SCOUT_MAX_SPEED);
      curr_max_speed_ = SCOUT_MAX_SPEED;
    }
  }
}

void SmScout::activeCallback()
{
}

void SmScout::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
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
    immobilityRecovery(2);
  }
}

void SmScout::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Goal done");
}

void SmScout::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  //  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Got feedback");
}

void SmScout::plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg)
{
  task_planning::PlanInfo srv_plan;
  srv_plan.request.replan.data = false;
  srv_plan.request.type.data = (uint8_t) mac::SCOUT;
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
  prev_srv_plan.response.code == srv_plan.response.code))
  {
    flag_interrupt_plan = true;
  }
  else
  {
    flag_interrupt_plan = false;
  }
}

// Methods +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SmScout::CancelMoveBaseGoal()
{
  if(ac.getState() != actionlib::SimpleClientGoalState::LOST)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
    ac.waitForServer();
    ac.cancelGoal();
    ac.waitForResult(ros::Duration(0.25));
  }
}

void SmScout::SetMoveBaseGoal()
{
  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Sending goal to MoveBase: " << move_base_goal);
  waypoint_timer = ros::Time::now();
  ac.sendGoal(move_base_goal, boost::bind(&SmScout::doneCallback, this,_1,_2), boost::bind(&SmScout::activeCallback, this), boost::bind(&SmScout::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));
}


void SmScout::SetMoveBaseSpeed(double max_speed)
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
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service to reconfigure MoveBase max speed to: "<< max_speed);
  }
  else
  {    
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service to reconfigure MoveBase (max speed).");
  }
}

void SmScout::setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
  const double pitch = 0.0;
  const double roll = 0.0;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  poseGoal.target_pose.header.frame_id = robot_name_ + "_odom";
  poseGoal.target_pose.pose.position.x = x;
  poseGoal.target_pose.pose.position.y = y;
  poseGoal.target_pose.pose.position.z = 0.0;
  poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
  poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
  poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
  poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmScout::RotateToHeading(double desired_yaw)
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
  ros::Duration timeoutHeading(30.0); // Timeout of 20 seconds

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

    flag_heading_fail=false;
  }
  else
  {
    Stop(0.1);
  }
}

void SmScout::homingRecovery()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Starting Homing Recovery.");

  CancelMoveBaseGoal();

  Lights(20);

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

void SmScout::immobilityRecovery(int type)
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

void SmScout::ClearCostmaps(double wait_time)
{  
  ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Braking rover to clear the Map");
  BrakeRamp(100, 1, 0); // Give more time

  ROS_WARN_STREAM("[" << robot_name_ << "] " << "Move Base State: " << move_base_state_.toString());
  
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("move_base/clear_costmaps",emptymsg))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service to clear costmap layers.");
    ROS_WARN_STREAM("[" << robot_name_ << "] " << "Map Cleared");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed calling clear_costmaps service.");
  }
  
  Brake(0.0);
}

void SmScout::GetTruePose()
{
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

void SmScout::Lights(double intensity)
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

void SmScout::RotateInPlace(double speed_ratio, double time)
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

void SmScout::MoveSideways(double speed_ratio, double time)
{
  driving_tools::MoveSideways srv_move_side;
  srv_move_side.request.speed_ratio  = speed_ratio;
  if (clt_move_side.call(srv_move_side))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service MoveSideways");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call service MoveSideways");
  }
}

void SmScout::TurnWheelsSideways(bool start, double time)
{
  driving_tools::TurnWheelsSideways srv_turn_wheels_side;
  srv_turn_wheels_side.request.start  = start;
  if (clt_turn_wheels_side.call(srv_turn_wheels_side))
  {
    ROS_INFO_STREAM_THROTTLE(5,"[" << robot_name_ << "] " <<"Called service TurnWheelsSideways");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Failed to call service TurnWheelsSideways");
  }
}

void SmScout::Stop(double time)
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

void SmScout::Brake(double intensity)
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

void SmScout::BrakeRamp(double max_intensity, double time, int aggressivity)
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

void SmScout::Drive(double speed_ratio, double time)
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

void SmScout::DriveCmdVel(double vx, double vy, double wz, double time)
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

void SmScout::RoverStatic(bool flag)
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

void SmScout::CheckWaypoint(int max_count)
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

        BrakeRamp(100, 1.0, 0);

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

bool SmScout::ApproachChargingStation(int max_count)
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

bool SmScout::HomingUpdate(bool init_landmark)
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

void SmScout::Plan()
{
  task_planning::PlanInfo srv_plan;
  if (!flag_interrupt_plan)
  {
    srv_plan.request.replan.data = true;
  }
  else
  {
    srv_plan.request.replan.data = false;
  }
  srv_plan.request.type.data = (uint8_t) mac::SCOUT;
  srv_plan.request.id.data = (uint8_t) robot_id_;
  ROS_WARN_STREAM("[" << robot_name_ << "] " << "SMACH, PLAN: " <<  (int) srv_plan.request.type.data << " id " << (int) srv_plan.request.id.data );

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
