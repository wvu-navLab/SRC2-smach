#include <state_machine/sm_scout.hpp>

SmScout::SmScout() :
ac("move_base", true),
move_base_state(actionlib::SimpleClientGoalState::PREEMPTED)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_status_pub = nh.advertise<state_machine::RobotStatus>("state_machine/status", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("driving/cmd_vel", 1);
  driving_mode_pub = nh.advertise<std_msgs::Int64>("driving/driving_mode", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmScout::localizedBaseCallback, this);
  // mobility_sub = nh.subscribe("/state_machine/mobility_scout", 1, &SmScout::mobilityCallback, this);
  waypoint_unreachable_sub = nh.subscribe("state_machine/waypoint_unreachable", 1, &SmScout::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("state_machine/arrived_at_waypoint", 1, &SmScout::arrivedAtWaypointCallback, this);
  volatile_sensor_sub = nh.subscribe("volatile_sensor", 1, &SmScout::volatileSensorCallback, this);
  volatile_cmd_sub = nh.subscribe("volatile_map/cmd", 1, &SmScout::volatileCmdCallback, this);
  localization_failure_sub = nh.subscribe("state_machine/localization_failure", 1, &SmScout::localizationFailureCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmScout::localizationCallback, this);
  driving_mode_sub =nh.subscribe("driving/driving_mode_",1, &SmScout::drivingModeCallback, this);
  laser_scan_sub =nh.subscribe("laser/scan",1, &SmScout::laserCallback, this);
  planner_interrupt_sub = nh.subscribe("/planner_interrupt", 1, &SmScout::plannerInterruptCallback, this);

  // Clients
  clt_wp_gen = nh.serviceClient<waypoint_gen::GenerateWaypoint>("navigation/generate_goal");
  clt_wp_start = nh.serviceClient<waypoint_gen::StartWaypoint>("navigation/start");
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

  // Service
  srv_mobility = nh.advertiseService("state_machine/mobility_service",&SmScout::setMobility, this);

  driving_mode_ = 0;
  waypoint_type_ = 0;

  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();
  laser_collision_timer = ros::Time::now();
  map_timer = ros::Time::now();
  wp_checker_timer =  ros::Time::now();

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
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_have_true_pose: " << (int)flag_have_true_pose);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_interrupt_plan: " << (int)flag_interrupt_plan);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_arrived_at_waypoint: " << (int)flag_arrived_at_waypoint);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_localizing_volatile: " << (int)flag_localizing_volatile);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_recovering_localization: " << (int)flag_recovering_localization);
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"flag_brake_engaged: " << (int)flag_brake_engaged);
    //---------------------------------------------------------------------------------------------------------------------

    // // Conditional flag logic (Preemptive conditions) +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // if((vol_detected_dist_>0.0) && !flag_localizing_volatile && !flag_localization_failure && flag_have_true_pose)
    // {
    //   flag_arrived_at_waypoint = true;
    //   flag_waypoint_unreachable = false;
    // }
    // if(flag_localization_failure && !flag_recovering_localization)
    // {
    //   flag_arrived_at_waypoint = true;
    //   flag_waypoint_unreachable = false;
    // }
    // //---------------------------------------------------------------------------------------------------------------------

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
    else if(flag_volatile_detected && flag_localizing_volatile && !flag_brake_engaged)
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


    // State execution ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // int state_to_exec_count = 0;
    // for(int i=0; i<state_to_exec.size(); i++)
    // {
    //   if(state_to_exec.at(i))
    //   {
    //     state_to_exec_count++;
    //   }
    // }

    // if(!flag_fallthrough_condition || state_to_exec_count == 1)
    // {
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
    // }
    // else
    // {
    //   ROS_WARN_STREAM("[" << robot_name_ << "] " <<"State fallthrough, flag_fallthrough_condition = %i, state_to_exec_count = " << (int) flag_fallthrough_condition, state_to_exec_count);
    //   flag_fallthrough_condition = false;
    // }
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

  Lights(20);

  while (!clt_approach_base.waitForExistence())
  {
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Waiting for ApproachChargingStation service");
  }

  Stop(2.0);

  Brake(100.0);

  while (!clt_sf_true_pose.waitForExistence())
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waiting for TruePose service");
  }

  RoverStatic(true);
  
  GetTruePose();

  RoverStatic(false);

  ClearCostmaps();

  Brake(0.0);

  // make sure we dont latch to a vol we skipped while homing
  vol_detected_dist_ = -1.0;

  flag_arrived_at_waypoint = true;
  flag_recovering_localization = false;
  flag_localizing_volatile = false;

  double progress = 1.0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int) _initialize;
  sm_status_pub.publish(status_msg);
}

void SmScout::statePlanning()
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

  // Brake (100.0);
  BrakeRamp(100, 3.0, 0);
  // check waypoint

  waypoint_checker::CheckCollision srv_wp_check;
  bool is_colliding = true;
  int counter = 0;
  while (is_colliding && counter<3)
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

  //ClearCostmaps();
  BrakeRamp(100, 2, 0);
  Brake(0.0);

if (!no_objective) {
  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Sending goal to MoveBase: " << move_base_goal);
  waypoint_timer = ros::Time::now();
  ac.sendGoal(move_base_goal, boost::bind(&SmScout::doneCallback, this,_1,_2), boost::bind(&SmScout::activeCallback, this), boost::bind(&SmScout::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));
  flag_arrived_at_waypoint = false;
}
else
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"no_objective\n");
}

  double progress = 1.0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int) _planning;
  sm_status_pub.publish(status_msg);
}

void SmScout::stateTraverse()
{
  ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Traverse State\n");

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
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
        flag_arrived_at_waypoint = true;
        flag_recovering_localization = false;
        flag_localizing_volatile = false;
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
    flag_arrived_at_waypoint = true;
    flag_recovering_localization = false;
    flag_localizing_volatile = false;

    Stop (1.0);

    //ClearCostmaps(); //TODO Check if they needed
    BrakeRamp(100, 2, 0);
    Brake(0.0);

  }

  ros::Duration timeoutMap(90.0);

  if (ros::Time::now() - map_timer > timeoutMap)
  {
    // std_srvs::Empty emptymsg;
    // ros::service::call("move_base/clear_costmaps",emptymsg);
    map_timer =ros::Time::now();
    // BrakeRamp(100, 2, 0); // Give more time
    // Brake(0.0);

    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Rover is stopped to clear the Map");
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Move Base State: "<< mb_state);
    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Map Cleared");
  }

  // ros::Duration timeoutWaypoint(120);
  // if (ros::Time::now() - waypoint_timer > timeoutWaypoint )
  // {
  //   ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Waypoint Unreachable");
  //   flag_waypoint_unreachable= true;
  //   Stop (1.0);
  //   ClearCostmaps();
  //   BrakeRamp(100, 2, 0);
  //   Brake(0.0);
  // }
  // else
  // {
  //   ROS_ERROR_STREAM_THROTTLE(1,"Remaining Time for Waypoint" << timeoutWaypoint - (ros::Time::now() - waypoint_timer));
  // }

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

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ros::Duration timeoutVolatileHandling(120);

  if (!flag_volatile_honed)
  {
    Stop(0.1);

    BrakeRamp(100, 0.1, 0);

    Brake(0.0);

    detection_timer = ros::Time::now();
  }

  if (vol_detected_dist_ < VOL_FOUND_THRESH || ros::Time::now() - detection_timer > timeoutVolatileHandling || flag_volatile_honed)
  {
    Stop(0.1);

    DriveCmdVel(1, 0, 0, 3);

    BrakeRamp(100, 0.5, 0);

    Brake(0.0);

    flag_localizing_volatile = false;
    flag_volatile_honed = false;
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "max_vel_x";
    double_param.value = 1.07;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    if (ros::service::call("move_base/DWAPlannerROS_SRC/set_parameters", srv_req, srv_resp))
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Called service to reconfigure MoveBase (increase max speed).");
    }
  }
  else
  {

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"SCOUT:Turning wheels sideways.");
    TurnWheelsSideways(true, 10.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"SCOUT:Moving sideways (Right).");
    MoveSideways(0.1, 10.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"SCOUT:Moving sideways (Left).");
    MoveSideways(-0.1, 20.0);

    ROS_WARN_STREAM("[" << robot_name_ << "] " <<"SCOUT:Moving sideways (Right).");
    MoveSideways(0.1, 10.0);

    flag_volatile_honed = true;
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

  double progress = 1.0;

  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  Stop (2.0);

  Lights (20);

  // Approach Base Station
  src2_approach_services::ApproachChargingStation srv_approach_base;
  srv_approach_base.request.approach_charging_station.data= true;
  bool approachSuccess = false;
  int homingRecoveryCount=0;
  while(!approachSuccess && homingRecoveryCount<3){
      if (clt_approach_base.call(srv_approach_base))
      {
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service ApproachChargingStation");
        ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Success finding the Base? "<< srv_approach_base.response.success.data);
        if(!srv_approach_base.response.success.data){
        homingRecovery();
        }
        else
        {
          approachSuccess=true;
        }
    }

    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service ApproachChargingStation");
    }
    homingRecoveryCount=homingRecoveryCount+1;
  }


  // Brake (100.0);
  BrakeRamp(100, 3, 0);
  if(approachSuccess)
  {
    // Homing - Measurement Update
    sensor_fusion::HomingUpdate srv_homing;
    // ros::spinOnce();

    srv_homing.request.angle = pitch_ + .4; // pitch up is negative number
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Requesting Angle for LIDAR "<<srv_homing.request.angle);
    srv_homing.request.initializeLandmark = flag_need_init_landmark;
    if (clt_homing.call(srv_homing))
    {
      ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service Homing [Update]");
      if(srv_homing.request.initializeLandmark && srv_homing.response.success)
      {
          base_location_ = srv_homing.response.base_location;
          ROS_WARN_STREAM("[" << robot_name_ << "] " <<"Saving Base Location  "<<base_location_.x << "," << base_location_.y);
      }
      // flag_localization_failure=false;
      // flag_arrived_at_waypoint = true;
      // flag_completed_homing = true;
      if(srv_homing.response.success)
      {
        flag_recovering_localization = false;
        flag_need_init_landmark = false;
        progress = 1.0;
      }
    }
    else
    {
      ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed to call service Homing [Update]");
      progress = -1.0;
    }
  }
  else
  {
    progress = -1.0;
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<" Homing Attempt Failed, Just Moving On For Now");
  }

  Lights (20);

  //Similar to initial homing, keep the localization good after homing.
  Brake(0.0);

  DriveCmdVel(-0.5,0.0,0.0,5);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  RotateInPlace(0.2, 3);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  //ClearCostmaps();
  BrakeRamp(100, 2, 0);
  Brake(0.0);


  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (int)  _lost;
  sm_status_pub.publish(status_msg);
}

//------------------------------------------------------------------------------------------------------------------------

// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmScout::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = msg->data;
  if (flag_localized_base) {
    ROS_WARN_STREAM_ONCE("Initial Localization Successful = " << (int)flag_localized_base);

  }
  else {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Waiting for Initial Localization  = " << (int)flag_localized_base);
  }
}

// void SmScout::mobilityCallback(const std_msgs::Int64::ConstPtr& msg)
// {
// flag_mobility = msg->data;
// if (flag_mobility == 0) {
//   ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"ROVER IMMOBILIZATION!  = " << (int) flag_mobility);
//   immobilityRecovery(1);
// } else {
//   ROS_WARN_STREAM_ONCE("Rover is traversing = " << (int) flag_mobility);
// }
// }

void SmScout::waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable = msg->data;
}

void SmScout::arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint = msg->data;
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
  if (min_vol_detected_dist_<30){
  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Minimum volatile detected." << min_vol_detected_dist_);
}
}

void SmScout::volatileCmdCallback(const std_msgs::Int64::ConstPtr& msg)
{
  // Update move_base max speed
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  double_param.name = "max_vel_x";
  double_param.value = 0.1;
  conf.doubles.push_back(double_param);

  srv_req.config = conf;

  if (ros::service::call("move_base/DWAPlannerROS_SRC/set_parameters", srv_req, srv_resp))
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Called service to reconfigure MoveBase (decrease max speed).");
  }

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

void SmScout::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
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

void SmScout::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone = true;
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Goal done");
}

void SmScout::activeCallback()
{
    // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Goal went active");
}

void SmScout::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  //  ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Got feedback");
}

void SmScout::plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg)
{
    // flag_interrupt_plan = msg->data;
  // ROS_INFO_STREAM("[" << robot_name_ << "] " <<"EXCAVATOR: Interrupt flag updated." << *msg);
}

// Methods +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

  //********************************************************************************************************
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

  flag_heading_fail=false;
  ros::Time start_time = ros::Time::now();
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

    // ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"TIME: " << ros::Time::now() - start_time << ", TIMEOUT: " << timeoutHeading);

    if (ros::Time::now() - start_time > timeoutHeading)
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

void SmScout::homingRecovery()
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

void SmScout::immobilityRecovery(int type)
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

  flag_mobility=true;

  flag_waypoint_unreachable=true;


}

void SmScout::ClearCostmaps()
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

void SmScout::GetTruePose()
{
  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose.call(srv_sf_true_pose))
  {
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"EXCAVATOR: Called service TruePose");
    ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Status of SF True Pose: "<< (int) srv_sf_true_pose.response.success);
    flag_have_true_pose = true;
  }
  else
  {
    ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"EXCAVATOR: Failed  to call service Pose Update");
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

// void SmScout::ToggleDetector(bool flag)
// {
//   volatile_handler::ToggleDetector srv_vol_detect;
//   srv_vol_detect.request.on  = flag;
//   if (clt_vol_detect_.call(srv_vol_detect))
//   {
//     ROS_INFO_STREAM("[" << robot_name_ << "] " <<"Called service ToggleDetector. Turned on? " << flag);
//   }
//   else
//   {
//     ROS_ERROR_STREAM("[" << robot_name_ << "] " <<"Failed  to call service ToggleDetector");
//   }
// }

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

bool SmScout::setMobility(state_machine::SetMobility::Request &req, state_machine::SetMobility::Response &res)
{
  ROS_ERROR_STREAM("[" << robot_name_ << "] " <<" GOT MOBILITY IN SM"<< req.mobility);
  flag_mobility = req.mobility;
  immobilityRecovery(1);
  //ros::Duration(2).sleep();
  res.success = true;
  return true;
}


void SmScout::Plan()
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

  if (srv_plan.response.code.data !=0 )
  {
    goal_pose_.position = srv_plan.response.objective.point;
    geometry_msgs::Quaternion quat;
    goal_pose_.orientation = quat;
    no_objective = false;
  }
  else
  {
    no_objective = true;
  }

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
  flag_interrupt_plan = false;
}

//------------------------------------------------------------------------------------------------------------------------
