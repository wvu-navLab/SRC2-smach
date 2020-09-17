#include <state_machine/sm_rd1.hpp>

SmRd1::SmRd1() :
ac("/scout_1/move_base", true),
move_base_state_(actionlib::SimpleClientGoalState::PREEMPTED)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_pub = nh.advertise<std_msgs::Int64>("/state_machine/state", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/scout_1/driving/cmd_vel", 1);
  pub_driving_mode_ = nh.advertise<std_msgs::Int64>("/scout_1/driving/driving_mode", 1);
  // Subscribers
  localized_base_sub = nh.subscribe("/state_machine/localized_base_scout", 1, &SmRd1::localizedBaseCallback, this);
  // mobility_sub = nh.subscribe("/state_machine/mobility_scout", 1, &SmRd1::mobilityCallback, this);
  waypoint_unreachable_sub = nh.subscribe("/state_machine/waypoint_unreachable", 1, &SmRd1::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("/state_machine/arrived_at_waypoint", 1, &SmRd1::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("/state_machine/volatile_detected", 1, &SmRd1::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("/state_machine/volatile_recorded", 1, &SmRd1::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("/state_machine/localization_failure", 1, &SmRd1::localizationFailureCallback, this);
  localization_sub  = nh.subscribe("/scout_1/localization/odometry/sensor_fusion", 1, &SmRd1::localizationCallback, this);
  driving_mode_sub =nh.subscribe("/scout_1/driving/driving_mode",1, &SmRd1::drivingModeCallback, this);
  laserscan_sub =nh.subscribe("/scout_1/laser/scan",1, &SmRd1::laserCallback, this);
  // Clients
  clt_wp_gen_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("/scout_1/navigation/generate_goal");
  clt_wp_start_ = nh.serviceClient<waypoint_gen::StartWaypoint>("/scout_1/navigation/start");
  clt_wp_nav_set_goal_ = nh.serviceClient<waypoint_nav::SetGoal>("/scout_1/navigation/set_goal");
  clt_wp_nav_interrupt_ = nh.serviceClient<waypoint_nav::Interrupt>("/scout_1/navigation/interrupt");
  clt_stop_ = nh.serviceClient<driving_tools::Stop>("/scout_1/driving/stop");
  clt_rip_ = nh.serviceClient<driving_tools::RotateInPlace>("/scout_1/driving/rotate_in_place");
  clt_drive_ = nh.serviceClient<driving_tools::MoveForward>("/scout_1/driving/move_forward");
  clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>("/scout_1/volatile/report");
  clt_vol_detect_ = nh.serviceClient<volatile_handler::ToggleDetector>("/scout_1/volatile/toggle_detector");
  clt_lights_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("/scout_1/toggle_light");
  clt_brake_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/scout_1/brake_rover");
  clt_approach_base_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/scout_1/approach_base_station");
  clt_rover_static_ = nh.serviceClient<sensor_fusion::RoverStatic>("/scout_1/sensor_fusion/toggle_rover_static");
  clt_homing_ = nh.serviceClient<sensor_fusion::HomingUpdate>("/scout_1/homing");
  clt_sf_true_pose_ = nh.serviceClient<sensor_fusion::GetTruePose>("/scout_1/true_pose");
  clt_waypoint_checker_ = nh.serviceClient<waypoint_checker::CheckCollision>("/scout_1/waypoint_checker");
  clt_srcp2_brake_rover_= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/scout_1/brake_rover");

  setMobilityService_ = nh.advertiseService("/state_machine/mobility_service_scout",&SmRd1::setMobility_, this);

  driving_mode_=0;
  waypoint_type_ =0;
  need_to_initialize_landmark=true;

  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();
  last_time_laser_collision_ = ros::Time::now();
  map_timer = ros::Time::now();
  wp_checker_timer=  ros::Time::now();
}



void SmRd1::run()
{
  ros::Rate loop_rate(5); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    ROS_INFO("flag_localized_base: %i", flag_localized_base);
    ROS_INFO("flag_mobility: %i",flag_mobility);
    ROS_INFO("flag_have_true_pose: %i",flag_have_true_pose);
    ROS_INFO("flag_waypoint_unreachable: %i",flag_waypoint_unreachable);
    ROS_INFO("flag_arrived_at_waypoint: %i",flag_arrived_at_waypoint);
  //  ROS_INFO("volatile_detected_distance: %f",volatile_detected_distance);
  //  ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    ROS_INFO("flag_volatile_recorded: %i",flag_volatile_recorded);
    ROS_INFO("flag_volatile_unreachable: %i",flag_volatile_unreachable);
    ROS_INFO("flag_localization_failure: %i",flag_localization_failure);
    ROS_INFO("flag_brake_engaged: %i",flag_brake_engaged);
    ROS_INFO("flag_fallthrough_condition: %i",flag_fallthrough_condition);
    //---------------------------------------------------------------------------------------------------------------------


    // Conditional flag logic (Preemptive conditions) +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if((volatile_detected_distance>0.0) && !flag_localizing_volatile && !flag_localization_failure && flag_have_true_pose)
    {
      flag_arrived_at_waypoint = true;
      flag_waypoint_unreachable = false;
    }
    if(flag_localization_failure && !flag_recovering_localization)
    {
      flag_arrived_at_waypoint = true;
      flag_waypoint_unreachable = false;
    }
    //---------------------------------------------------------------------------------------------------------------------


    // State machine truth table ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    state_to_exec.clear();
    state_to_exec.resize(num_states,0);
    if(!flag_have_true_pose && (flag_arrived_at_waypoint || flag_waypoint_unreachable))
    {
      state_to_exec.at(_initialize) = 1;
    }
    else if((flag_arrived_at_waypoint || flag_waypoint_unreachable) && (flag_localization_failure || flag_recovering_localization))
    {
      state_to_exec.at(_lost) = 1;
    }
    else if((flag_arrived_at_waypoint || flag_waypoint_unreachable) && (volatile_detected_distance==-1.0) && !flag_localizing_volatile && !flag_brake_engaged)
    {
      state_to_exec.at(_planning) = 1;
    }
    else if((!flag_arrived_at_waypoint && !flag_waypoint_unreachable) && !flag_brake_engaged)
    {
      state_to_exec.at(_traverse) = 1;
    }
    else if(((volatile_detected_distance>=0)  || flag_localizing_volatile) && !flag_brake_engaged)
    {
      state_to_exec.at(_volatile_handler) = 1;
    }
    else
    {
      flag_arrived_at_waypoint = true;
      flag_waypoint_unreachable = false;
      flag_localizing_volatile = false;
      flag_fallthrough_condition = true;
    }
    //---------------------------------------------------------------------------------------------------------------------


    // State execution ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    int state_to_exec_count = 0;
    for(int i=0; i<state_to_exec.size(); i++)
    {
      if(state_to_exec.at(i))
      {
        state_to_exec_count++;
      }
    }

    if(!flag_fallthrough_condition || state_to_exec_count == 1)
    {
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
      }
    }
    else
    {
      ROS_WARN("State fallthough, flag_fallthrough_condition = %i, state_to_exec_count = %i",flag_fallthrough_condition, state_to_exec_count);
    }
    // -------------------------------------------------------------------------------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// State function definitions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmRd1::stateInitialize()
{
  ROS_WARN("Initialization State!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  ToggleDetector(false);

  while (!clt_lights_.waitForExistence())
  {
      ROS_WARN("SCOUT: Waiting for Lights");
  }
  Lights("0.8");

  while (!clt_approach_base_.waitForExistence())
  {
    ROS_WARN("SCOUT: Waiting for ApproachBaseStation service");
  }

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  bool approachSuccess = false;
  int homingRecoveryCount = 0;
  while(!approachSuccess && homingRecoveryCount<3){
      if (clt_approach_base_.call(srv_approach_base))
      {
        ROS_INFO("SCOUT: Called service ApproachBaseStation");
        ROS_INFO_STREAM("Success finding the Base? "<< srv_approach_base.response.success.data);
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
      ROS_ERROR("SCOUT: Failed  to call service ApproachBaseStation");
    }
    homingRecoveryCount=homingRecoveryCount+1;
  }

  Stop(2.0);

  Brake(100.0);

  while (!clt_sf_true_pose_.waitForExistence())
  {
    ROS_ERROR("SCOUT: Waiting for TruePose service");
  }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose_.call(srv_sf_true_pose))
  {
    ROS_INFO("SCOUT: Called service TruePose");
    ROS_INFO_STREAM("Status of SF True Pose: "<< srv_sf_true_pose.response.success);
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service Pose Update");
  }

  RoverStatic(true);

  if(approachSuccess){
  // Homing - Initialize Base Station Landmark
  while (!clt_homing_.waitForExistence())
  {
      ROS_WARN("SCOUT: Waiting for Homing Service");
  }
  sensor_fusion::HomingUpdate srv_homing;
  ros::spinOnce();

  srv_homing.request.angle = pitch_ + .4; // pitch up is negative number
  ROS_ERROR("Requesting Angle for LIDAR %f",srv_homing.request.angle);
  srv_homing.request.initializeLandmark = need_to_initialize_landmark;
  if (clt_homing_.call(srv_homing))
  {
    ROS_INFO("SCOUT: Called service HomingUpdate");
    if(srv_homing.response.success){
    base_location_ = srv_homing.response.base_location;
    need_to_initialize_landmark = false;
  }else{
    ROS_ERROR(" Initial Homing Fail, Starting Without Base Location");
  }
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service HomingUpdate");
  }
}
else{
  ROS_ERROR(" Initial Homing Fail, Starting Without Base Location");
}

  RoverStatic(false);

  Lights("0.6");

  // Minimal Maneuvers to keep the localization good and get rid of BaseStation obstacle before generating initial path.
  Brake(0.0);

  DriveCmdVel(-0.5,0.0,0.0,4);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  RotateInPlace(0.2, 3);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  ToggleDetector(true);

  ClearCostmaps();


  waypoint_gen::StartWaypoint srv_wp_start;
  srv_wp_start.request.start  = true;

  if (clt_wp_start_.call(srv_wp_start))
  {
    ROS_INFO("Starting Waypoint Gen");
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service Waypoint Start");
  }


  // make sure we dont latch to a vol we skipped while homing
  volatile_detected_distance = -1.0;
  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_pub.publish(state_msg);
}

void SmRd1::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint = false;
  if(waypoint_type_==1)
  {
    flag_waypoint_unreachable=false;
  }

  ROS_INFO("SCOUT: Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  while (!clt_wp_gen_.waitForExistence())
  {
    ROS_ERROR("SCOUT: Waiting for Waypoint Gen service");
  }
  waypoint_gen::GenerateWaypoint srv_wp_gen;

  srv_wp_gen.request.start  = true;
  if(flag_completed_homing)
  {
    flag_completed_homing = false;
    srv_wp_gen.request.next  = true;
  }
  else if(flag_waypoint_unreachable)
  {
    flag_waypoint_unreachable=false;
    srv_wp_gen.request.next  = true;
  }

  else
  {
  srv_wp_gen.request.next  = false;
  }

  if (clt_wp_gen_.call(srv_wp_gen))
  {
    ROS_INFO("SCOUT: Called service Generate Waypoint");
    goal_pose_ = srv_wp_gen.response.goal;
    waypoint_type_ = srv_wp_gen.response.type;
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service Generate Waypoint");
  }
  ROS_INFO_STREAM("SCOUT: WP Generation: Goal pose: " << goal_pose_);

  goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

  Brake (0.0);

  RotateToHeading(goal_yaw_);

  // Brake (100.0);
  BrakeRamp(100, 3, 0);
  // check waypoint

  waypoint_checker::CheckCollision srv_wp_check;
  bool is_colliding = true;
  int counter = 0;
  while (is_colliding && counter<3)
  {
    srv_wp_check.request.x  = goal_pose_.position.x;
    srv_wp_check.request.y = goal_pose_.position.y;
    if (clt_waypoint_checker_.call(srv_wp_check))
    {
      ROS_INFO("SCOUT: Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if(is_colliding)
      {
        ROS_INFO("SCOUT: Waypoint Unreachable. Getting new waypoint");
        srv_wp_gen.request.start  = true;
        srv_wp_gen.request.next  = true;
        if (clt_wp_gen_.call(srv_wp_gen))
        {
          ROS_INFO("SCOUT: Called service Generate Waypoint");
          goal_pose_ = srv_wp_gen.response.goal;
	        waypoint_type_ = srv_wp_gen.response.type;

          goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

          Brake (0.0);

          RotateToHeading(goal_yaw_);

          BrakeRamp(100, 3, 0);
          Brake (0.0);
        }
        else
        {
          ROS_ERROR("SCOUT: Failed to call service Generate Waypoint");
        }
      }
    }
    else
    {
      ROS_ERROR("SCOUT: Failed to call service Waypoint Checker");


  }
  ros::spinOnce();
  counter=counter+1;
}
  ClearCostmaps();

  // Stop (2.0);

  Brake (0.0);

  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("SCOUT: Sending goal to MoveBase: " << move_base_goal);
  waypoint_timer_ = ros::Time::now();
  ac.sendGoal(move_base_goal, boost::bind(&SmRd1::doneCallback, this,_1,_2), boost::bind(&SmRd1::activeCallback, this), boost::bind(&SmRd1::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));

  std_msgs::Int64 state_msg;
  state_msg.data = _planning;
  sm_state_pub.publish(state_msg);
}
void SmRd1::stateTraverse()
{
  ROS_WARN("Traverse State\n");

  if(flag_localized_base && !flag_have_true_pose)
  {
    ROS_INFO("SCOUT: Localized but don't have true pose.");
    flag_have_true_pose = true;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }
  if(flag_volatile_recorded)
  {
    ROS_INFO("SCOUT: Volatile recorded.");
    volatile_detected_distance = -1.0;
    flag_localizing_volatile = false;
    flag_volatile_recorded = false;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }
  if(flag_recovering_localization && !flag_localization_failure)
  {
    ROS_INFO("SCOUT: Recovering localization or failure in localization.");
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
    flag_recovering_localization = false;
  }

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
  {
    ROS_INFO("SCOUT: Close to goal, getting new waypoint.");
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
    if(waypoint_type_ ==0 )
    {
      flag_localization_failure = false;
    }
    else
    {
      ROS_INFO(" Reached a Waypoint Designated for Localization Update Type : %f ",waypoint_type_ );
      flag_localization_failure = true;
      waypoint_type_=0; // just to account for triggered homing update
    }
  }

  ros::Duration timeOutWPCheck(3.0);
  if (ros::Time::now() - wp_checker_timer > timeOutWPCheck) {
    bool is_colliding = false;
    waypoint_checker::CheckCollision srv_wp_check;
    if (clt_waypoint_checker_.call(srv_wp_check)) {
      ROS_INFO("SCOUT: Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if (is_colliding) {
        ROS_INFO("SCOUT: Waypoint Unreachable. Sending to Planning");
        flag_waypoint_unreachable = true;
      }
    }
    wp_checker_timer = ros::Time::now();
  }



  move_base_state_ = ac.getState();
  int mb_state =(int) move_base_state_.state_;
  ROS_WARN_STREAM("MoveBase status: "<< mb_state);

  if(mb_state==5 || mb_state==7)
  {
    ROS_ERROR("SCOUT: MoveBase has failed to make itself useful.");
    flag_waypoint_unreachable= true;

    Stop (1.0);

    ClearCostmaps();
  }

  ros::Duration timeoutMap(30.0);

  if (ros::Time::now() - map_timer > timeoutMap)
  {
    std_srvs::Empty emptymsg;
    ros::service::call("/scout_1/move_base/clear_costmaps",emptymsg);
    map_timer =ros::Time::now();
    ROS_WARN("Map Cleared");
  }

  ros::Duration timeoutWaypoint(120);
  if (ros::Time::now() - waypoint_timer_ > timeoutWaypoint )
  {
    ROS_ERROR("Waypoint Unreachable");
    flag_waypoint_unreachable= true;
    Stop (1.0);
    ClearCostmaps();
  }
  else
  {
    ROS_ERROR_STREAM_THROTTLE(1,"Remaining Time for Waypoint" << timeoutWaypoint - (ros::Time::now() - waypoint_timer_));
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_pub.publish(state_msg);

}

void SmRd1::stateVolatileHandler()
{

  ROS_WARN("Volatile Handling State!");
}

void SmRd1::stateLost()
{
  ROS_ERROR("LOST STATE!\n");
  flag_recovering_localization = false;

  flag_localizing_volatile = false;
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  ROS_INFO("SCOUT: Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));


  Stop (2.0);

  // if(pow(pow(base_location_.x - current_pose_.position.x,2)+pow(base_location_.y - current_pose_.position.y,2),.5)>10.0){
  //
  //
  // ROS_INFO_STREAM("Defining goal from base location");
  //
  // goal_yaw_ = atan2(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);
  //
  // RotateToHeading(goal_yaw_);
  //
  // move_base_msgs::MoveBaseGoal move_base_goal;
  // ac.waitForServer();
  // setPoseGoal(move_base_goal, base_location_.x, base_location_.y, goal_yaw_);
  // ROS_INFO_STREAM("SCOUT: Sending goal to MoveBase: " << move_base_goal);
  // ac.sendGoal(move_base_goal, boost::bind(&SmRd1::doneCallback, this,_1,_2), boost::bind(&SmRd1::activeCallback, this), boost::bind(&SmRd1::feedbackCallback, this,_1));
  // ac.waitForResult(ros::Duration(0.25));
  // // set as a waypoint type 1 so it will come back here.
  // waypoint_type_=1;
  // return;
  // }
  Lights ("0.8");


  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  bool approachSuccess = false;
  int homingRecoveryCount=0;
  while(!approachSuccess && homingRecoveryCount<3){
      if (clt_approach_base_.call(srv_approach_base))
      {
        ROS_INFO("SCOUT: Called service ApproachBaseStation");
        ROS_INFO_STREAM("Success finding the Base? "<< srv_approach_base.response.success.data);
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
      ROS_ERROR("SCOUT: Failed  to call service ApproachBaseStation");
    }
    homingRecoveryCount=homingRecoveryCount+1;
  }


  // Brake (100.0);
  BrakeRamp(100, 3, 0);
  if(approachSuccess){
  // Homing - Measurement Update
  sensor_fusion::HomingUpdate srv_homing;
  ros::spinOnce();

  srv_homing.request.angle = pitch_ + .4; // pitch up is negative number
  ROS_INFO("Requesting Angle for LIDAR %f",srv_homing.request.angle);
  srv_homing.request.initializeLandmark = need_to_initialize_landmark;
  if (clt_homing_.call(srv_homing))
  {
    ROS_INFO("SCOUT: Called service Homing [Update]");
    if(srv_homing.request.initializeLandmark && srv_homing.response.success){
        base_location_ = srv_homing.response.base_location;
        ROS_WARN("SCOUT: Saving Base Location %f %f",base_location_.x, base_location_.y);
    }
    flag_localization_failure=false;
    flag_arrived_at_waypoint = true;
    flag_completed_homing = true;
    if(srv_homing.response.success){
      need_to_initialize_landmark=false;
    }
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service Homing [Update]");
  }

}
else{
  ROS_ERROR(" Homing Attempt Failed, Just Moving On For Now");

}

  Lights ("0.6");

  //Similar to initial homing, keep the localization good after homing.
  Brake(0.0);

  DriveCmdVel(-0.5,0.0,0.0,4);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  RotateInPlace(0.2, 3);

  BrakeRamp(100, 3, 0);

  Brake(0.0);


  ToggleDetector(true);

  ClearCostmaps();

  // make sure we dont latch to a vol we skipped while homing
  volatile_detected_distance = -1.0;

  //flag_completed_homing = true;
  std_msgs::Int64 state_msg;
  state_msg.data = _lost;
  sm_state_pub.publish(state_msg);
}
//------------------------------------------------------------------------------------------------------------------------


// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmRd1::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = msg->data;
  if (flag_localized_base) {
    ROS_WARN_ONCE("Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

// void SmRd1::mobilityCallback(const std_msgs::Int64::ConstPtr& msg)
// {
// flag_mobility = msg->data;
// if (flag_mobility == 0) {
//   ROS_ERROR("ROVER IMMOBILIZATION!  = %i", flag_mobility);
//   immobilityRecovery(1);
// } else {
//   ROS_WARN_ONCE("Rover is traversing = %i", flag_mobility);
// }
// }

void SmRd1::waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable = msg->data;
}

void SmRd1::arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint = msg->data;
}

void SmRd1::volatileDetectedCallback(const std_msgs::Float32::ConstPtr& msg)
{

  prev_volatile_detected_distance = volatile_detected_distance;
  volatile_detected_distance = msg->data;

}

void SmRd1::volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_volatile_recorded = msg->data;
  ROS_INFO_STREAM("Record: " << flag_volatile_recorded);
  if (flag_volatile_recorded == true)
  {
          volatile_detected_distance = -1.0;
  }else
  {
    not_detected_timer = ros::Time::now();
    ROS_INFO_STREAM("NAO" << not_detected_timer.toSec());
  }
}

void SmRd1::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
}
void SmRd1::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_=msg->data;
}
void SmRd1::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev_ = yaw_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

}

void SmRd1::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<float> ranges = msg->ranges;
  std::sort (ranges.begin(), ranges.end());
  float min_range = 0;
  for (int i = 0; i < LASER_SET_SIZE; ++i)
  {
    min_range = min_range + ranges[i];
  }
  min_range = min_range/LASER_SET_SIZE;
  ROS_INFO_STREAM_THROTTLE(2,"SCOUT: Minimum range average: " << min_range);

  if (min_range < LASER_THRESH)
  {
    if (ros::Time::now() - last_time_laser_collision_ < ros::Duration(20))
    {
      ROS_WARN("SCOUT: Close to wall.");
      counter_laser_collision_++;
      ROS_INFO_STREAM("SCOUT: Counter laser:" << counter_laser_collision_);
    }
    else
    {
      counter_laser_collision_ = 0;
      ROS_INFO_STREAM("SCOUT: Counter laser RESET!");
    }
    last_time_laser_collision_ = ros::Time::now();
  }

  if (counter_laser_collision_ > LASER_COUNTER_THRESH)
  {
    counter_laser_collision_ =0;
    ROS_ERROR("SCOUT: LASER COUNTER > 20 ! Starting Recovery.");
    immobilityRecovery(2);
  }
}

void SmRd1::setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    poseGoal.target_pose.header.frame_id = "scout_1_tf/odom";
    poseGoal.target_pose.pose.position.x = x;
    poseGoal.target_pose.pose.position.y = y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmRd1::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_ = true;
    // ROS_INFO("Goal done");
}
void SmRd1::activeCallback()
{
    // ROS_INFO("Goal went active");
}
void SmRd1::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  //  ROS_INFO("Got feedback");
}

void SmRd1::RotateToHeading(double desired_yaw)
{
  ros::Rate raterotateToHeading(20);

  ROS_INFO("Starting yaw control.");

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
  ros::Duration timeoutHeading(10.0); // Timeout of 20 seconds

  while(fabs(yaw_error) > yaw_thres)
  {
    RotateInPlace(copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error),0.0);

    raterotateToHeading.sleep();
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
    // ROS_INFO_STREAM("Trying to control yaw to desired angles. Yaw error: "<<yaw_error);

    // ROS_ERROR_STREAM("TIME: "<<ros::Time::now() - start_time << ", TIMEOUT: " << timeoutHeading);

    if (ros::Time::now() - start_time > timeoutHeading)
    {
      ROS_ERROR("Yaw Control Failed. Possibly stuck. Break control.");
      flag_heading_fail = true;
      break;
    }
  }

  if (flag_heading_fail)
  {
    ROS_WARN("Recovery action initiated in yaw control.");

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

void SmRd1::homingRecovery()
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Homing Recovery.");

  Lights("0.6");

  Stop(2.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  // Drive(-0.3, 3.0);
  DriveCmdVel(-0.3,-0.6, 0.0, 4.0);

  Stop(0.0);

  RotateToHeading(yaw_ - M_PI/4);

  Stop(0.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

  DriveCmdVel(0.7,0.0,0.0,4.0);

  Stop(0.0);

  BrakeRamp(100, 3, 0);

  Brake(0.0);

}

void SmRd1::immobilityRecovery(int type)
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Recovery.");

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

void SmRd1::ClearCostmaps()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/scout_1/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/scout_1/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("SCOUT: Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR("SCOUT: Failed calling clear_costmaps service.");
  }
}

void SmRd1::Lights(std::string intensity)
{
  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = intensity;
  if (clt_lights_.call(srv_lights))
  {
    ROS_INFO("SCOUT: Called service ToggleLight");
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service ToggleLight");
  }
}

void SmRd1::RotateInPlace(double throttle, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = throttle;
  if (clt_rip_.call(srv_turn))
  {
    ROS_INFO_THROTTLE(5,"SCOUT: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("SCOUT: Failed to call service RotateInPlace");
  }
}

void SmRd1::Stop(double time)
{
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    ROS_INFO("SCOUT: Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service Stop");
  }
}

void SmRd1::Brake(double intensity)
{
  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = intensity;
  if (clt_srcp2_brake_rover_.call(srv_brake))
  {
    if (intensity < 0.01)
    {
      flag_brake_engaged =false;
    }
    else
    {
      flag_brake_engaged =true;
    }
    //ROS_INFO_STREAM("SCOUT: Called service SRCP2 Brake. Engaged?: " << flag_brake_engaged);
  }
  else
  {
      ROS_ERROR("SCOUT: Failed  to call service Brake");
  }
}

void SmRd1::BrakeRamp(double max_intensity, double time, int aggressivity)
{
  double freq = 10;
  ros::Rate brake_rate(freq);
  int num_steps = (int) freq * time;
  if(aggressivity == 0)
  {
    // ROS_INFO("Brake Ramp.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double intensity = (static_cast<double>(counter + 1)/(freq * time))*max_intensity;
      // ROS_INFO_STREAM("Brake intensity: " << intensity);
      Brake(intensity);
      brake_rate.sleep();
    }
  }
  else if (aggressivity == 1)
  {
    // ROS_INFO("Brake Logistics Curve.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double multiplier = 2;
      double x = (static_cast<double>(counter + 1)/(freq * time)) * time * multiplier;
      double intensity =  max_intensity / (1 + exp(-x)) - max_intensity/2;
      // ROS_INFO_STREAM("Brake intensity: " << intensity);
      Brake(intensity);
      brake_rate.sleep();
    }
  }
  else
  {
    ROS_INFO("Brake FULL.");
    Brake(max_intensity);
    ros::Duration(time).sleep();
  }
  ROS_INFO_STREAM("SCOUT: Called service SRCP2 Brake. Engaged? " << flag_brake_engaged);
}

void SmRd1::Drive(double throttle, double time)
{
  driving_tools::MoveForward srv_drive;

  if (pitch_ > 0.0 && throttle < 0.0) {
   throttle = throttle - (pitch_ / 0.52) * 0.3;
  }
  srv_drive.request.throttle = throttle;


  if (clt_drive_.call(srv_drive))
  {
    ros::Time start_time = ros::Time::now();
    ROS_INFO("SCOUT: Called service Drive");
    while(ros::Time::now() - start_time < ros::Duration(time))
    {
      std_msgs::Int64 mode;
      mode.data = 2;
      pub_driving_mode_.publish(mode);
    }
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service Drive");
  }
}


void SmRd1::DriveCmdVel(double vx, double vy, double wz, double time)
{
  if (pitch_ > 0.0 && vx < 0.0)
  {
   vx = vx + (pitch_ / 0.52) * vx;
  }
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = wz;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 20 seconds
  ROS_ERROR("Drive Cmd Vel publisher.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub.publish(cmd_vel);
  }
}

void SmRd1::ToggleDetector(bool flag)
{
  volatile_handler::ToggleDetector srv_vol_detect;
  srv_vol_detect.request.on  = flag;
  if (clt_vol_detect_.call(srv_vol_detect))
  {
    ROS_INFO_STREAM("SCOUT: Called service ToggleDetector. Turned on? " << flag);
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service ToggleDetector");
  }
}

void SmRd1::RoverStatic(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static_.call(srv_rover_static))
  {
    ROS_INFO_STREAM("SCOUT: Called service RoverStatic. Turned on? " << flag);
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service RoverStatic");
  }

}

bool SmRd1::setMobility_(state_machine::SetMobility::Request &req, state_machine::SetMobility::Response &res){
  ROS_ERROR(" GOT MOBILITY IN SM %d", req.mobility);
  flag_mobility = req.mobility;
  immobilityRecovery(1);
  //ros::Duration(2).sleep();
  res.success = true;
  return true;
}

//------------------------------------------------------------------------------------------------------------------------
