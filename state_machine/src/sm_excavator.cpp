#include <state_machine/sm_excavator.hpp>

SmExcavator::SmExcavator() :
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
  if (ros::param::get("/num_haulers", num_haulers_) == false)
  {
    ROS_FATAL("No parameter 'num_haulers' specified");
    ros::shutdown();
    exit(1);
  }

  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_status_pub = nh.advertise<state_machine::RobotStatus>("state_machine/status", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("driving/cmd_vel", 1);
  driving_mode_pub = nh.advertise<std_msgs::Int64>("driving/driving_mode", 1);
  excavation_status_pub = nh.advertise<state_machine::ExcavationStatus>("state_machine/excavation_status",1);
  sensor_yaw_pub = nh.advertise<std_msgs::Float64>("sensor/yaw/command/position", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmExcavator::localizedBaseCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmExcavator::localizationCallback, this);
  driving_mode_sub =nh.subscribe("driving/driving_mode",1, &SmExcavator::drivingModeCallback, this);
  laser_scan_sub =nh.subscribe("laser/scan",1, &SmExcavator::laserCallback, this);
  joint_states_sub = nh.subscribe("joint_states", 1, &SmExcavator::jointStateCallback, this);
  bucket_info_sub = nh.subscribe("scoop_info", 1, &SmExcavator::bucketCallback, this);
  goal_volatile_sub = nh.subscribe("manipulation/volatile_pose", 1, &SmExcavator::goalVolatileCallback, this);
  manipulation_cmd_sub = nh.subscribe("manipulation/cmd", 1, &SmExcavator::manipulationCmdCallback, this);
  planner_interrupt_sub = nh.subscribe("/planner_interrupt", 1, &SmExcavator::plannerInterruptCallback, this);
  for (int i=0; i<num_haulers_; i++)
  {
    std::string hauler_odom_topic;
    hauler_odom_topic = "/small_hauler_" + std::to_string(i+1) + "/localization/odometry/sensor_fusion";
    hauler_odom_subs.push_back(nh.subscribe(hauler_odom_topic, 1, &SmExcavator::haulerOdomCallback, this));

    std::string hauler_status_topic;
    hauler_status_topic = "/small_hauler_" + std::to_string(i+1) + "/state_machine/hauler_status";
    hauler_status_subs.push_back(nh.subscribe(hauler_status_topic, 1, &SmExcavator::haulerStatusCallback, this));
  }
  
  // Clients
  clt_stop = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_rip = nh.serviceClient<driving_tools::RotateInPlace>("driving/rotate_in_place");
  clt_move_side = nh.serviceClient<driving_tools::MoveSideways>("driving/move_sideways");
  clt_turn_wheels_side = nh.serviceClient<driving_tools::TurnWheelsSideways>("driving/turn_wheels_sideways");
  clt_drive = nh.serviceClient<driving_tools::MoveForward>("driving/move_forward");
  clt_lights = nh.serviceClient<srcp2_msgs::SpotLightSrv>("spot_light");
  clt_power = nh.serviceClient<srcp2_msgs::SystemPowerSaveSrv>("system_monitor/power_save");
  clt_brake = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_approach_base = nh.serviceClient<src2_approach_services::ApproachChargingStation>("approach_charging_station_service");
  clt_rover_static = nh.serviceClient<sensor_fusion::RoverStatic>("sensor_fusion/toggle_rover_static");
  clt_homing = nh.serviceClient<sensor_fusion::HomingUpdate>("homing");
  clt_sf_true_pose = nh.serviceClient<sensor_fusion::GetTruePose>("true_pose");
  clt_waypoint_checker = nh.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  clt_srcp2_brake_rover = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_home_arm = nh.serviceClient<move_excavator::HomeArm>("manipulation/home_arm");
  clt_extend_arm = nh.serviceClient<move_excavator::ExtendArm>("manipulation/extend_arm");
  clt_lower_arm = nh.serviceClient<move_excavator::LowerArm>("manipulation/lower_arm");
  clt_scoop = nh.serviceClient<move_excavator::Scoop>("manipulation/scoop");
  clt_after_scoop = nh.serviceClient<move_excavator::AfterScoop>("manipulation/after_scoop");
  clt_drop_volatile = nh.serviceClient<move_excavator::DropVolatile>("manipulation/drop_volatile");
  clt_forward_kin = nh.serviceClient<move_excavator::ExcavatorFK>("manipulation/excavator_fk");
  clt_go_to_pose = nh.serviceClient<move_excavator::GoToPose>("manipulation/go_to_pose");
  clt_find_hauler = nh.serviceClient<move_excavator::FindHauler>("manipulation/find_hauler");
  clt_where_hauler = nh.serviceClient<src2_object_detection::WhereToParkHauler>("/where_to_park_hauler");
  clt_task_planning = nh.serviceClient<task_planning::PlanInfo>("/task_planner_exc_haul");
  clt_vol_mark = nh.serviceClient<volatile_map::MarkCollected>("/volatile_map_mark_collected");

  map_timer = ros::Time::now();
  wp_checker_timer = ros::Time::now();
  laser_collision_timer = ros::Time::now();

  manipulation_timer = ros::Time::now();

  nav_msgs::Odometry temp_small_hauler_odom;
  state_machine::HaulerStatus temp_small_hauler_status;
  for (int i=0; i<num_haulers_; i++)
  {
    small_haulers_odom_.push_back(temp_small_hauler_odom);
    small_haulers_status_.push_back(temp_small_hauler_status);
  }
}

void SmExcavator::run()
{
  ros::Rate loop_rate(5); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_have_true_pose: " << (int)flag_have_true_pose);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_interrupt_plan: " << (int)flag_interrupt_plan);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_arrived_at_waypoint: " << (int)flag_arrived_at_waypoint);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_localizing_volatile: " << (int)flag_localizing_volatile);
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
    else if(flag_interrupt_plan || (flag_arrived_at_waypoint && !flag_recovering_localization && !flag_localizing_volatile && !flag_brake_engaged))
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
    // else
    // {
    //   stateVolatileHandler(); // temporary
    // }
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
void SmExcavator::stateInitialize()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Initialization State!\n");

  double progress = 0;

  while (!clt_lights.waitForExistence())
  {
      ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for Lights");
  }

  while (!clt_approach_base.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachChargingStation service");
  }

  while (!clt_home_arm.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for HomeArm service");
  }

  while (!clt_sf_true_pose.waitForExistence())
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waiting for TruePose service");
  }

  while (!clt_where_hauler.waitForExistence())
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waiting for WhereToParkHauler service");
  }

  Lights(20);

  ExecuteHomeArm(2,0);

  Stop(0.1);

  Brake(100.0);

  RoverStatic(true);

  GetTruePose();

  RoverStatic(false);

  ClearCostmaps(5.0);

  Brake(0.0);

  // flag_manipulation_interrupt = true; // TODO: Remove this later
  PublishExcavationStatus();

  // make sure we dont latch to a vol we skipped while homing
  progress = 1.0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _initialize;
  sm_status_pub.publish(status_msg);
}

void SmExcavator::statePlanning()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Planning!\n");

  double progress = 0;

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
    
    if(flag_localizing_volatile)
    {
      ros::Duration(15).sleep();
    }

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

void SmExcavator::stateTraverse()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Traverse State\n");
  move_base_state_ = ac.getState();
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"MoveBase status: "<< (std::string) move_base_state_.toString());

  double progress = 0;

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
  {
    CancelMoveBaseGoal();
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Close to goal, getting new waypoint.");
    flag_arrived_at_waypoint = true;
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

      // ClearCostmaps(5.0);  // TODO: Check if they needed

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

  progress = distance_to_goal;

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _traverse;
  sm_status_pub.publish(status_msg);
}

void SmExcavator::stateVolatileHandler()
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Volatile Handling State!");

  double progress = 0;

  CancelMoveBaseGoal();

  // If not interrupted, this will cancel move-base goal and 
  // manipulation will be enabled at first time 
  if(!flag_manipulation_enabled && !flag_manipulation_interrupt)
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Enabling Excavation State Machine.");
    Stop(0.1);
    BrakeRamp(100, 1, 0);
    Brake(0.0);

    SetHaulerParkingLocation();

    manipulation_timer = ros::Time::now();
    flag_manipulation_enabled = true;
  }
  
  // Then  the manipulation state-machine keeps being called unless timeout is reached or it is finished
  if (flag_manipulation_enabled && (ros::Time::now() - manipulation_timer) < ros::Duration(840))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation State Machine enabled.");
    ExcavationStateMachine();
  }
  else if (flag_manipulation_enabled && (ros::Time::now() - manipulation_timer) > ros::Duration(840))
  {
    // If time is over, it will cancel excavation
    CancelExcavation(false);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation State Machine interrupted by timeout.");
  }
  else
  {
    // If excavation gets disabled, it will cancel excavation
    // CancelExcavation(false);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation State Machine disabled.");
  }

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t)  _volatile_handler;
  sm_status_pub.publish(status_msg);
}

void SmExcavator::stateLost()
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"LOST STATE!\n");

  CancelMoveBaseGoal();

  double progress = 0.0;
  
  Stop (2.0);

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
void SmExcavator::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
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

void SmExcavator::systemMonitorCallback(const srcp2_msgs::SystemMonitorMsg::ConstPtr& msg)
{
  power_level_ = msg->power_level;
  power_rate_ = msg->power_rate;

  if (power_level_< 30) 
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " << "Power Level Warning: " << power_level_);

    Plan();

    RotateToHeading(M_PI_2);
    ros::Duration(120).sleep();
  }
}

void SmExcavator::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg){
  driving_mode=msg->data;
}

void SmExcavator::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev_ = yaw_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
  
  double radius = hypot(current_pose_.position.x, current_pose_.position.y);

  if(radius > CRATER_RADIUS)
  { 
    flag_interrupt_plan = true;
  }


  if (abs(pitch_ * 180 / M_PI) > 10) 
  {
    ROS_WARN_STREAM_THROTTLE(10, "Robot Climbing Up! Pitch: " << pitch_ * 180 / M_PI);
    if (curr_max_speed_ != EXCAVATOR_MAX_SPEED)
    {
      SetMoveBaseSpeed(EXCAVATOR_MAX_SPEED);
      curr_max_speed_ = EXCAVATOR_MAX_SPEED;
    }

    if (abs(pitch_ * 180 / M_PI) > 27) 
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Robot Cant Climb! Pitch: " << pitch_ * 180 / M_PI);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Commanding IMMOBILITY.");
      CancelMoveBaseGoal();
      Stop(0.1);
      Brake(100.0);
      Brake(0.0);
      DriveCmdVel(-0.5,0.0,0.0,3);
      Stop(0.1);
      SetMoveBaseGoal();
    }
  }
  else
  {
    if (curr_max_speed_ != EXCAVATOR_MAX_SPEED)
    {
      SetMoveBaseSpeed(EXCAVATOR_MAX_SPEED);
      curr_max_speed_ = EXCAVATOR_MAX_SPEED;
    }
  }
  
  if (abs(roll_ * 180 / M_PI) > 10) 
  {
    ROS_WARN_STREAM_THROTTLE(10, "Robot is Sideways! Roll: " << roll_ * 180 / M_PI);
    if (curr_max_speed_ != EXCAVATOR_MAX_SPEED)
    {
      SetMoveBaseSpeed(EXCAVATOR_MAX_SPEED);
      curr_max_speed_ = EXCAVATOR_MAX_SPEED;
    }

    if (abs(roll_ * 180 / M_PI) > 27) 
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Robot Cant Climb! Roll: " << roll_ * 180 / M_PI);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Commanding IMMOBILITY.");
      CancelMoveBaseGoal();
      Stop(0.1);
      Brake(100.0);
      Brake(0.0);
      DriveCmdVel(-0.5,0.0,0.0,3);
      Stop(0.1);
      SetMoveBaseGoal();
    }
  }
  else
  {
    if (curr_max_speed_ != EXCAVATOR_MAX_SPEED)
    {
      SetMoveBaseSpeed(EXCAVATOR_MAX_SPEED);
      curr_max_speed_ = EXCAVATOR_MAX_SPEED;
    }
  }

}

void SmExcavator::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
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
    counter_laser_collision_ = 0;
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"LASER COUNTER > 20 ! Starting Recovery.");
    // immobilityRecovery(2);
  }
}


void SmExcavator::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Goal done");
}

void SmExcavator::activeCallback()
{
}

void SmExcavator::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Got feedback");
}

void SmExcavator::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  // Find current angles and position
  int shoulder_yaw_joint_idx;
  int shoulder_pitch_joint_idx;
  int elbow_pitch_joint_idx;
  int wrist_pitch_joint_idx;
  int spawning_zone_joint_idx;

  // loop joint states
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "shoulder_yaw_joint") {
      shoulder_yaw_joint_idx = i;
    }
    if (msg->name[i] == "shoulder_pitch_joint") {
      shoulder_pitch_joint_idx = i;
    }
    if (msg->name[i] == "elbow_pitch_joint") {
      elbow_pitch_joint_idx = i;
    }
    if (msg->name[i] == "wrist_pitch_joint") {
      wrist_pitch_joint_idx = i;
    }
  }

  q1_pos_ = msg->position[shoulder_yaw_joint_idx];  // TODO: Get ids from the message names
  q2_pos_ = msg->position[shoulder_pitch_joint_idx];
  q3_pos_ = msg->position[elbow_pitch_joint_idx];
  q4_pos_ = msg->position[wrist_pitch_joint_idx];
}

void SmExcavator::bucketCallback(const srcp2_msgs::ExcavatorScoopMsg::ConstPtr &msg)
{
  flag_bucket_full = msg->volatile_clod_mass || msg->regolith_clod_mass;
  flag_has_volatile = msg->volatile_clod_mass;
}

void SmExcavator::goalVolatileCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  odom_to_arm_mount = tf_buffer.lookupTransform(robot_name_+"_arm_mount", robot_name_+"_odom", ros::Time(0), ros::Duration(1.0));
  tf2::doTransform(*msg, volatile_pose_, odom_to_arm_mount);

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation: Goal volatile updated. Pose:" << *msg);
}

void SmExcavator::manipulationCmdCallback(const std_msgs::Int64::ConstPtr &msg)
{
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: New command received:" << msg->data);
  switch (msg->data)
  {
  case STOP:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Enable interrupt.");
      ExecuteHomeArm(2,0);
      excavation_counter_ = 0;

      flag_found_hauler = false;
      flag_found_volatile = false;
      flag_manipulation_interrupt = true;
      flag_manipulation_enabled = false;
      flag_localizing_volatile = false;
    }
    break;
  case HOME_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Homing Arm.");
      ExecuteHomeArm(2,0);
      if(flag_bucket_full)
      {
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Extend.");
      }
      else
      {
        if (flag_found_volatile)
        {
          ExecuteLowerArm(2,0);
          ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Scoop.");
        }
        else
        {
          ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Search.");
        }
      }
    }
    break;
  case SEARCH_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Searching.");
      ExecuteLowerArm(2,0); 
      flag_found_volatile = ExecuteSearch();
      if(!flag_found_volatile)
      {
        flag_manipulation_enabled = false;
      }
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Home.");
    }
    break;
  case LOWER_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Lowering Arm.");
      ExecuteLowerArm(2,0); 
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Search.");

    }
    break;
  case SCOOP_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Scooping.");
      ExecuteScoop(2,0,volatile_heading_);
      flag_found_hauler = FindHauler(60);  // What if we don't find hauler/check if it is reacheable
      if (flag_found_hauler)
      {
        ExecuteAfterScoop(2,0);
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Home.");
      }
      else
      {
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Drop.");
      }
    }
    break;
  case EXTEND_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Extending Arm.");
      
      ExecuteExtendArm(10,0);

      geometry_msgs::PointStamped safe_point;
      safe_point.header = bin_point_.header;
      safe_point.point.x = 1.4;
      safe_point.point.y = 0.0;
      safe_point.point.z = 1.8;

      ExecuteGoToPose(5, 0, safe_point);
      
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Drop.");
    }
    break;
  case DROP_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Dropping.");

      ExecuteDrop(2,0,1);

      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Go to Home.");
    }
    break;
  case START:
    {
      flag_manipulation_enabled = false;
      flag_manipulation_interrupt = false;
      
      flag_interrupt_plan = false;
      flag_arrived_at_waypoint = true;
      flag_recovering_localization = false;
      flag_localizing_volatile = true;

      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ExcavationCMD: Disable interrupt!");
    }
    break;
  }
}

void SmExcavator::plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg)
{
  task_planning::PlanInfo srv_plan; 
  srv_plan.request.replan.data = false;
  srv_plan.request.type.data = (uint8_t) mac::EXCAVATOR;
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

void SmExcavator::haulerOdomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const nav_msgs::OdometryConstPtr& msg = event.getMessage();

  char msg_hauler_ind = topic.c_str()[HAULER_STR_LOC];
  int msg_hauler_id = std::atoi(&msg_hauler_ind);

  small_haulers_odom_[msg_hauler_id-1] = *msg;

  if (partner_hauler_id_ == msg_hauler_id)
  {
    partner_hauler_location_ = msg->pose.pose.position;
  } 
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Got small_hauler_1 odometry.");
}


void SmExcavator::haulerStatusCallback(const ros::MessageEvent<state_machine::HaulerStatus const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const state_machine::HaulerStatusConstPtr& msg = event.getMessage();

  char msg_hauler_ind = topic.c_str()[HAULER_STR_LOC];
  int msg_hauler_id = std::atoi(&msg_hauler_ind);

  small_haulers_status_[msg_hauler_id-1] = *msg;

  if (partner_hauler_id_ == msg_hauler_id)
  {
    partner_hauler_status_ = *msg;
    flag_hauler_ready = msg->hauler_ready.data;
  } 
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Got small_hauler_1 odometry.");
}

void SmExcavator::CancelMoveBaseGoal()
{
  if(ac.getState() != actionlib::SimpleClientGoalState::LOST)
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
    ac.waitForServer();
    ac.cancelGoal();
    ac.waitForResult(ros::Duration(0.25));
  }
}

void SmExcavator::SetMoveBaseGoal()
{
  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  SetPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Sending goal to MoveBase: " << move_base_goal);
  waypoint_timer = ros::Time::now();
  ac.sendGoal(move_base_goal, boost::bind(&SmExcavator::doneCallback, this,_1,_2), boost::bind(&SmExcavator::activeCallback, this), boost::bind(&SmExcavator::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));
}

void SmExcavator::SetMoveBaseSpeed(double max_speed)
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

void SmExcavator::SetPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
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

void SmExcavator::SetPowerMode(bool power_save)
{
  srcp2_msgs::SystemPowerSaveSrv srv_power;
  srv_power.request.power_save = power_save;
  if (clt_power.call(srv_power))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service PowerSaver");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service PowerSaver");
  }
}

void SmExcavator::RotateToHeading(double desired_yaw)
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
      flag_interrupt_plan = true;
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
    Stop(0.0);
  }
}

void SmExcavator::homingRecovery()
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

void SmExcavator::immobilityRecovery(int type)
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

void SmExcavator::ClearCostmaps(double wait_time)
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

void SmExcavator::GetTruePose()
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

void SmExcavator::Lights(double intensity)
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

void SmExcavator::RotateInPlace(double speed_ratio, double time)
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

void SmExcavator::MoveSideways(double speed_ratio, double time)
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

void SmExcavator::TurnWheelsSideways(bool start, double time)
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

void SmExcavator::Stop(double time)
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

void SmExcavator::Brake(double intensity)
{
  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = intensity;
  if (clt_srcp2_brake_rover .call(srv_brake))
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

void SmExcavator::BrakeRamp(double max_intensity, double time, int aggressivity)
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

void SmExcavator::Drive(double speed_ratio, double time)
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


void SmExcavator::DriveCmdVel(double vx, double vy, double wz, double time)
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
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Drive Cmd Vel publisher.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub.publish(cmd_vel);
  }
}

void SmExcavator::RoverStatic(bool flag)
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

void SmExcavator::ExecuteHomeArm(double duration, double wait_time)
{
  move_excavator::HomeArm srv;

  srv.request.heading = 0;
  srv.request.timeLimit = duration;

  if (clt_home_arm.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service HomeArm.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service HomeArm");
  }

}

void SmExcavator::ExecuteLowerArm(double duration, double wait_time)
{
  move_excavator::LowerArm srv;

  srv.request.heading = volatile_heading_;
  srv.request.timeLimit = duration;

  if (clt_lower_arm.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service LowerArm.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service LowerArm");
  }

}

void SmExcavator::ExecuteScoop(double duration, double wait_time, double heading)
{
  move_excavator::Scoop srv;

  srv.request.heading = heading;
  srv.request.timeLimit = duration;

  if (clt_scoop.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Scoop.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service Scoop");
  }
}

void SmExcavator::ExecuteAfterScoop(double duration, double wait_time)
{
  move_excavator::AfterScoop srv;

  srv.request.heading =  volatile_heading_;
  srv.request.timeLimit = duration;

  if (clt_after_scoop.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service AfterScoop.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service AfterScoop");
  }
}


void SmExcavator::ExecuteExtendArm(double duration, double wait_time)
{

  move_excavator::ExtendArm srv;

  srv.request.heading = relative_heading_;
  srv.request.timeLimit = duration;
  srv.request.range = relative_range_;

  if (clt_extend_arm.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service ExtendArm.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service ExtendArm");
  }
}

void SmExcavator::ExecuteDrop(double duration, double wait_time, int type)
{
  move_excavator::DropVolatile srv;

  srv.request.type = type;
  srv.request.timeLimit = duration;

  if (clt_drop_volatile.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Drop.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service Drop");
  }
}

void SmExcavator::ExecuteGoToPose(double duration, double wait_time, const geometry_msgs::PointStamped &point)
{
  move_excavator::GoToPose srv;

  srv.request.timeLimit = duration;
  srv.request.goal = point;

  if (clt_go_to_pose.call(srv))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service GoToPose.");
    ros::Duration(wait_time).sleep();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service GoToPose.");
  }
}

bool SmExcavator::ExecuteSearch()
{
  
  double v = 0.2; // motion speed
  double t = 5;   // time of motion

  std::vector<double> q1s {0.0, M_PI/3,  -M_PI/3}; // Search angles

  std::vector<double> directions {0.0, 1.0, 1.0, -1.0, -1.0}; // Directions
  std::vector<bool> wheelOrientations {false, false, true, true, false}; // True turns wheels sideways

  for (int j = 0; j < directions.size(); j++)
  {

    if (!wheelOrientations[j])
    {
      Drive(v* directions[j], fabs(directions[j]) * t);
    }
    else
    {
      MoveSideways(v * directions[j], fabs(directions[j]) * t);
    }

    Stop(0.1);
    BrakeRamp(100, 1, 0);
    Brake(0.0);

    for (int i = 0; i < q1s.size(); i++) // Change the angle of arm to search
    {
      // srv_scoop.request.heading = q1s[i];
      //ROS_INFO(" i: %d q1s[i]: %f", i, srv_scoop.request.heading);

      ExecuteScoop(5,3, q1s[i]);
      ros::spinOnce();

      if (flag_has_volatile)
      {
        ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Excavation: Found volatile!");

        volatile_heading_ = q1s[i];

        ExecuteDrop(2,3,0);
        ExecuteHomeArm(2,0);
        ros::spinOnce();

        return true;
      }
      else // did not find volatile
      {
        ExecuteDrop(2,3,0);
        ros::spinOnce();

        ExecuteAfterScoop(2,3); // This is to remove from the ground
        ros::Duration(2.0).sleep();
        ros::spinOnce();
      }
    }

    ExecuteHomeArm(2,0); 

    if (!wheelOrientations[j])
    {
      Drive(-v * directions[j], fabs(directions[j]) * t);
    }
    else
    {
      MoveSideways(-v * directions[j], fabs(directions[j]) * t);
    }

    Stop(0.1);
    BrakeRamp(100, 1, 0);
    Brake(0.0);

  }

  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Did not find volatile!"); 
  
  return false;
}


bool SmExcavator::FindHauler(double timeout)
{
  move_excavator::FindHauler srv_find;

  srv_find.request.timeLimit = timeout;
  srv_find.request.side = relative_side_;
  if (clt_find_hauler.call(srv_find))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service FindHauler.");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service FindHauler.");
  }

  if (srv_find.response.success)
  {
    geometry_msgs::PointStamped bin_point = srv_find.response.target;
    // bin_point.point.x += 0.1;

    camera_link_to_arm_mount = tf_buffer.lookupTransform(robot_name_+"_arm_mount", robot_name_+"_left_camera_optical", ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(bin_point, bin_point_, camera_link_to_arm_mount);

    bin_point_.point.z += 0.6;
  
    relative_range_ = hypot(bin_point_.point.x-0.7, bin_point_.point.y);
    relative_heading_ = atan2(bin_point_.point.y, bin_point_.point.x-0.7);

    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Excavation: Target bin updated. Point:" << bin_point_);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Havent found the Hauler bin.");
    bin_point_.point.x = 1.4;
    bin_point_.point.y = 0.0;
    bin_point_.point.z = 1.8;

    relative_range_ = hypot(bin_point_.point.x-0.7, bin_point_.point.y);
    relative_heading_ = atan2(bin_point_.point.y, bin_point_.point.x-0.7);
    return false;
  }
}

void SmExcavator::GetForwardKinematics(double timeout)
{
  move_excavator::ExcavatorFK srv;
  motion_control::ArmGroup q;
  q.q1 = q1_pos_;
  q.q2 = q2_pos_;
  q.q3 = q3_pos_;
  q.q4 = q4_pos_;

  srv.request.joints = q;

  bool success = clt_forward_kin.call(srv);
  eePose_ = srv.response.eePose;
}

bool SmExcavator::ApproachChargingStation(int max_count)
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

void SmExcavator::CheckWaypoint(int max_count)
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

        BrakeRamp(100, 1, 0);

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

bool SmExcavator::HomingUpdate(bool init_landmark)
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

void SmExcavator::SetHaulerParkingLocation()
{
  src2_object_detection::WhereToParkHauler srv_where_hauler;

  srv_where_hauler.request.data.data = true;

  if (clt_where_hauler.call(srv_where_hauler))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service WhereToParkHauler.");

    relative_side_ = srv_where_hauler.response.side; // 1 is left and -1 is right
    hauler_parking_pose_ = srv_where_hauler.response.pose;
    flag_found_parking_site = true;
    ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Where to park hauler: " << hauler_parking_pose_);
    PublishExcavationStatus();
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service WhereToParkHauler.");
  }
}

void SmExcavator::MarkCollectedVolatile(bool success)
{
  volatile_map::MarkCollected srv_vol_mark;

  srv_vol_mark.request.collected = success;
  srv_vol_mark.request.attempted = true;
  srv_vol_mark.request.vol_index = goal_vol_index_;

  if (clt_vol_mark.call(srv_vol_mark))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service MarkCollected.");
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service MarkCollected.");
  }
}


void SmExcavator::ExcavationStateMachine()
{
  switch (excavation_state_)
  {
    case HOME_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Homing Arm.");
      
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Bucket full? " << (int) flag_bucket_full);
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Found volatile? " << (int) flag_found_volatile);

      ExecuteHomeArm(5,2);

      if(flag_bucket_full)
      {        
        ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Going to Extend.");
        excavation_state_ = EXTEND_MODE;
      }
      else
      {
        if (flag_found_volatile)
        {
          ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Going to Scoop.");
          ExecuteLowerArm(5,0);
          excavation_state_ = SCOOP_MODE;
        }
        else
        {
          ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Going to Search.");
          excavation_state_ = SEARCH_MODE;
        } 
      }
    }
    break;
    case SEARCH_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Searching.");

      ExecuteLowerArm(5,0); 
      flag_found_volatile = ExecuteSearch();
      
      if(flag_found_volatile)
      {
        PublishExcavationStatus();
        
        // Wait until hauler gets ready 
        while(!flag_hauler_ready && (ros::Time::now() - manipulation_timer) < ros::Duration(840))
        {
          ros::Duration(1.0).sleep();
          ros::spinOnce();
        }
      }
      else
      {
        PublishExcavationStatus();
        // If the excavator does not find volatiles after Searching,
        // it will cancel excavation
        CancelExcavation(false);
      }
      excavation_state_ = HOME_MODE;
    }
    break;
    case LOWER_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Lowering Arm.");

      ExecuteLowerArm(5,0); 
      excavation_state_ = SEARCH_MODE;
    }
    break;
    case SCOOP_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Scooping.");

      //TODO: Turn on power saving

      ExecuteScoop(5,0,volatile_heading_);
      if(!flag_found_hauler)
      {
        flag_found_hauler = FindHauler(120);
      }
      
      PublishExcavationStatus();

      // Look forward before starting to move again
      std_msgs::Float64 sensor_yaw;
      sensor_yaw.data = 0.0;
      sensor_yaw_pub.publish(sensor_yaw);

      if (flag_found_hauler)
      {
        // If the hauler was found with the service, it will scoop material and go to Home
        ExecuteAfterScoop(5,2);
        excavation_state_ = HOME_MODE;
      }
      else
      {
        // If not it will drop the material below the terrain and ask Hauler to approach
        ExecuteDrop(5,0,0);
        ROS_ERROR_STREAM("[" << robot_name_ << "] " << "Excavation: Waiting for Hauler");

        flag_hauler_ready = false;
        
        // Wait until hauler gets ready 
        while(!flag_hauler_ready && (ros::Time::now() - manipulation_timer) < ros::Duration(840))
        {
          ros::Duration(1.0).sleep();
          ros::spinOnce();
        }
      }
    }
    break;
    case EXTEND_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Extending Arm.");

      ExecuteExtendArm(10,1);
      
      excavation_state_ = DROP_MODE;
    }
    break;
    case DROP_MODE:
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Dropping Bucket Content.");

      ExecuteDrop(5,0,1);

      geometry_msgs::PointStamped safe_point;
      safe_point.header = bin_point_.header;
      safe_point.point.x = 1.4;
      safe_point.point.y = 0.0;
      safe_point.point.z = 1.8;

      // Put the Arm in a safe position before Home
      ExecuteGoToPose(5,1,safe_point); 

      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Scoop counter: " << excavation_counter_);

      if (excavation_counter_++ > MAX_EXCAVATION_COUNTER)
      {
        // If the excavator digs five times 
        // it will cancel excavation
        CancelExcavation(true);
      }
      excavation_state_ = HOME_MODE;
    }
    break;
  }
}

void SmExcavator::CancelExcavation(bool success)
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Excavation: Canceling excavation:" << (int) success);

  // Reset all important variables
  volatile_heading_ = 0.0;
  relative_heading_ = 0.0;
  relative_range_ = 1.7;

  // Reset Exc Smach flags
  flag_found_hauler = false;
  flag_found_volatile = false;
  flag_found_parking_site = false;

  // Reset Exc Counter
  excavation_counter_ = 0;

  // Put arm in Home position
  ExecuteHomeArm(5,0);

  // Set flags to leave Volatile Handling State
  flag_manipulation_enabled = false;
  flag_localizing_volatile = false;

  //TODO: Turn off power saving
  MarkCollectedVolatile(success);
}

void SmExcavator::PublishExcavationStatus()
{
  state_machine::ExcavationStatus msg;

  msg.excavator_id.data = robot_id_;
  msg.state.data = excavation_state_;  
  msg.bucket_full.data = flag_bucket_full;
  msg.found_parking_site.data = flag_found_parking_site;
  msg.parking_pose = hauler_parking_pose_;
  msg.found_volatile.data = flag_found_volatile;
  msg.found_hauler.data = flag_found_hauler;
  msg.counter.data = excavation_counter_;
  msg.progress.data = excavation_counter_/MAX_EXCAVATION_COUNTER;

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Publishing Excavation State Machine Status." << msg);

  excavation_status_pub.publish(msg);

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Publishing Excavation State Machine Status.");
}

void SmExcavator::Plan()
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
  srv_plan.request.type.data = (uint8_t) mac::EXCAVATOR;
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
    partner_hauler_id_ = 1; //TODO: Tell Jareds we need partner ID
    goal_vol_index_ = srv_plan.response.volatile_index.data;
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
