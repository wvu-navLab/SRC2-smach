#include <state_machine/sm_rd2.hpp>

SmRd2::SmRd2() :
ac("/excavator_1/move_base", true),
move_base_state_(actionlib::SimpleClientGoalState::LOST)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_pub = nh.advertise<std_msgs::Int64>("/state_machine/state", 1);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/excavator_1/driving/cmd_vel", 1);
  // Subscribers
  localized_base_sub = nh.subscribe("/state_machine/localized_base_excavator", 1, &SmRd2::localizedBaseCallback, this);
  mobility_sub = nh.subscribe("/state_machine/mobility_excavator", 10, &SmRd2::mobilityCallback, this);
  waypoint_unreachable_sub = nh.subscribe("/state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("/state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("/state_machine/volatile_detected", 1, &SmRd2::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("/state_machine/volatile_recorded", 1, &SmRd2::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("/state_machine/localization_failure", 1, &SmRd2::localizationFailureCallback, this);
  localization_sub  = nh.subscribe("/excavator_1/localization/odometry/sensor_fusion", 1, &SmRd2::localizationCallback, this);
  driving_mode_sub =nh.subscribe("/excavator_1/driving/driving_mode",1, &SmRd2::drivingModeCallback, this);


  // Clients
  clt_wp_gen_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("/excavator_1/navigation/generate_goal");
  clt_wp_start_ = nh.serviceClient<waypoint_gen::StartWaypoint>("/excavator_1/navigation/start");
  clt_wp_nav_set_goal_ = nh.serviceClient<waypoint_nav::SetGoal>("/excavator_1/navigation/set_goal");
  clt_wp_nav_interrupt_ = nh.serviceClient<waypoint_nav::Interrupt>("/excavator_1/navigation/interrupt");
  clt_stop_ = nh.serviceClient<driving_tools::Stop>("/excavator_1/driving/stop");
  clt_rip_ = nh.serviceClient<driving_tools::RotateInPlace>("/excavator_1/driving/rotate_in_place");
  clt_drive_ = nh.serviceClient<driving_tools::MoveForward>("/excavator_1/driving/move_forward");
  clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>("/excavator_1/volatile/report");
  clt_vol_detect_ = nh.serviceClient<volatile_handler::ToggleDetector>("/excavator_1/volatile/toggle_detector");
  clt_lights_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("/excavator_1/toggle_light");
  clt_brake_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/excavator_1/brake_rover");
  clt_approach_base_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/excavator_1/approach_base_station");
  clt_rover_static_ = nh.serviceClient<sensor_fusion::RoverStatic>("/excavator_1/sensor_fusion/toggle_rover_static");
  clt_homing_ = nh.serviceClient<sensor_fusion::HomingUpdate>("/excavator_1/homing");
  clt_sf_true_pose_ = nh.serviceClient<sensor_fusion::GetTruePose>("/excavator_1/true_pose");
  clt_waypoint_checker_ = nh.serviceClient<waypoint_checker::CheckCollision>("/excavator_1/waypoint_checker");
  clt_srcp2_brake_rover_= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/excavator_1/brake_rover");
  // this is a comment

  driving_mode_=0;

  //HAULER
  //Publishers
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/hauler_1/driving/cmd_vel", 1);

  //Subscribers
  localized_base_sub_hauler_ = nh.subscribe("/state_machine/localized_base_hauler", 1, &SmRd2::localizedBaseCallbackHauler, this);
  mobility_sub_hauler_ = nh.subscribe("/state_machine/mobility_hauler", 10, &SmRd2::mobilityCallbackHauler, this);
  waypoint_unreachable_sub_hauler_ = nh.subscribe("/state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableCallbackHauler, this);
  arrived_at_waypoint_sub_hauler_ = nh.subscribe("/state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointCallbackHauler, this);
  localization_failure_sub_hauler_ = nh.subscribe("/state_machine/localization_failure", 1, &SmRd2::localizationFailureCallbackHauler, this);
  localization_sub_hauler_  = nh.subscribe("/hauler_1/localization/odometry/sensor_fusion", 1, &SmRd2::localizationCallbackHauler, this);
  driving_mode_sub_hauler_ =nh.subscribe("/hauler_1/driving/driving_mode",1, &SmRd2::drivingModeCallbackHauler, this);

  //Clients
  clt_wp_gen_hauler_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("/hauler_1/navigation/generate_goal");
  clt_wp_start_hauler_ = nh.serviceClient<waypoint_gen::StartWaypoint>("/hauler_1/navigation/start");
  clt_wp_nav_set_goal_hauler_ = nh.serviceClient<waypoint_nav::SetGoal>("/hauler_1/navigation/set_goal");
  clt_wp_nav_interrupt_hauler_ = nh.serviceClient<waypoint_nav::Interrupt>("/hauler_1/navigation/interrupt");
  clt_stop_hauler_ = nh.serviceClient<driving_tools::Stop>("/hauler_1/driving/stop");
  clt_rip_hauler_ = nh.serviceClient<driving_tools::RotateInPlace>("/hauler_1/driving/rotate_in_place");
  clt_drive_hauler_ = nh.serviceClient<driving_tools::MoveForward>("/hauler_1/driving/move_forward");
  clt_vol_report_hauler_ = nh.serviceClient<volatile_handler::VolatileReport>("/hauler_1/volatile/report");
  clt_vol_detect_hauler_ = nh.serviceClient<volatile_handler::ToggleDetector>("/hauler_1/volatile/toggle_detector");
  clt_lights_hauler_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("/hauler_1/toggle_light");
  clt_brake_hauler_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/hauler_1/brake_rover");
  clt_approach_base_hauler_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/hauler_1/approach_base_station");
  clt_rover_static_hauler_ = nh.serviceClient<sensor_fusion::RoverStatic>("/hauler_1/sensor_fusion/toggle_rover_static");
  clt_homing_hauler_ = nh.serviceClient<sensor_fusion::HomingUpdate>("/hauler_1/homing");
  clt_sf_true_pose_hauler_ = nh.serviceClient<sensor_fusion::GetTruePose>("/hauler_1/true_pose");
  clt_waypoint_checker_hauler_ = nh.serviceClient<waypoint_checker::CheckCollision>("/hauler_1/waypoint_checker");
  clt_srcp2_brake_rover_hauler_= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/hauler_1/brake_rover");

  driving_mode_hauler_=0;
  need_to_initialize_landmark=true;

  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();
}

void SmRd2::run()
{
  ros::Rate loop_rate(2); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ROS_INFO("flag_localized_base: %i",flag_localized_base);
    ROS_INFO("flag_mobility: %i",flag_mobility);
    ROS_INFO("flag_have_true_pose: %i",flag_have_true_pose);
    ROS_INFO("flag_waypoint_unreachable: %i",flag_waypoint_unreachable);
    ROS_INFO("flag_arrived_at_waypoint: %i",flag_arrived_at_waypoint);
    ROS_INFO("volatile_detected_distance: %f",volatile_detected_distance);
    ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
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
void SmRd2::stateInitialize()
{
  ROS_WARN("Initialization State!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  ToggleDetector(false);

  Lights("0.8");

  while (!clt_approach_base_.waitForExistence())
  {
    ROS_WARN("EXCAVATOR: Waiting for ApproachBaseStation service");
  }

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  bool approachSuccess = false;
  int homingRecoveryCount = 0;
  while(!approachSuccess && homingRecoveryCount<3){
      if (clt_approach_base_.call(srv_approach_base))
      {
        ROS_INFO("EXCAVATOR: Called service ApproachBaseStation");
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
      ROS_ERROR("EXCAVATOR: Failed  to call service ApproachBaseStation");
    }
    homingRecoveryCount=homingRecoveryCount+1;
  }

  Stop(2.0);

  Brake(100.0);

  while (!clt_sf_true_pose_.waitForExistence())
  {
    ROS_ERROR("EXCAVATOR: Waiting for TruePose service");
  }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose_.call(srv_sf_true_pose))
  {
    ROS_INFO("EXCAVATOR: Called service TruePose");
    ROS_INFO_STREAM("Status of SF True Pose: "<< srv_sf_true_pose.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Pose Update");
  }

  RoverStatic(true);

  if(approachSuccess){
  // Homing - Initialize Base Station Landmark
  sensor_fusion::HomingUpdate srv_homing;
  ros::spinOnce();

  srv_homing.request.angle = pitch_ + .4; // pitch up is negative number
  ROS_ERROR("Requesting Angle for LIDAR %f",srv_homing.request.angle);
  srv_homing.request.initializeLandmark = need_to_initialize_landmark;
  if (clt_homing_.call(srv_homing))
  {
    ROS_INFO("EXCAVATOR: Called service HomingUpdate");
    if(srv_homing.response.success){
    base_location_ = srv_homing.response.base_location;
    need_to_initialize_landmark = false;
  }else{
    ROS_ERROR(" Initial Homing Fail, Starting Without Base Location");
  }
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service HomingUpdate");
  }
}
else{
  ROS_ERROR(" Initial Homing Fail, Starting Without Base Location");
}

  RoverStatic(false);

  Lights("0.6");

  Brake(0.0);

  // Drive(-0.2, 4.0);
  DriveCmdVel(-0.3, 0.0, 0.0, 4.0);
  //
  Stop(5.0);

  Brake(100.0);

  // Brake(0.0);

  // DriveCmdVel(-0.3, 0.0, 0.0, 5.0);

  // Stop(5.0);

  // Brake(100.0);

  // Brake(0.0);
  //
  // DriveCmdVel(0.4, 0.0, 0.0, 5.0);
  //
  // Stop(10.0);
  //
  // Brake(100.0);
  //
  // Brake(0.0);

  // RotateInPlace(0.2, 3.0);

  // Stop(2.0);

  // Brake(100.0);

  ToggleDetector(true);

  ClearCostmaps();

  Brake(0.0);

  waypoint_gen::StartWaypoint srv_wp_start;
  srv_wp_start.request.start  = true;

  if (clt_wp_start_.call(srv_wp_start))
  {
    ROS_INFO("Starting Waypoint Gen");
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Waypoint Start");
  }



  // make sure we dont latch to a vol we skipped while homing
  volatile_detected_distance = -1.0;
  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_pub.publish(state_msg);
}

void SmRd2::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint = false;
  if(waypoint_type_==1)
  {
    flag_waypoint_unreachable=false;
  }

  ROS_INFO("EXCAVATOR: Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  Stop(1.0);

  Brake(100.0);

  // ROS_INFO_STREAM("goal pose: " << goal_pose);
  // Generate Goal
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
    ROS_INFO("EXCAVATOR: Called service Generate Waypoint");
    goal_pose_ = srv_wp_gen.response.goal;
    waypoint_type_ = srv_wp_gen.response.type;
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Generate Waypoint");
  }
  ROS_INFO_STREAM("EXCAVATOR: WP Generation: Goal pose: " << goal_pose_);

  goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

  Brake (0.0);

  RotateToHeading(goal_yaw_);

  Brake (100.0);

  // check waypoint

  waypoint_checker::CheckCollision srv_wp_check;
  bool is_colliding = true;
  while (is_colliding)
  {
    srv_wp_check.request.x  = goal_pose_.position.x;
    srv_wp_check.request.y = goal_pose_.position.y;
    if (clt_waypoint_checker_.call(srv_wp_check))
    {
      ROS_INFO("EXCAVATOR: Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if(is_colliding)
      {
        ROS_INFO("EXCAVATOR: Waypoint Unreachable. Getting new waypoint");
        srv_wp_gen.request.start  = true;
        srv_wp_gen.request.next  = true;
        if (clt_wp_gen_.call(srv_wp_gen))
        {
          ROS_INFO("EXCAVATOR: Called service Generate Waypoint");
          goal_pose_ = srv_wp_gen.response.goal;
	        waypoint_type_ = srv_wp_gen.response.type;

          goal_yaw_ = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

          Brake (0.0);

          RotateToHeading(goal_yaw_);

          Brake (100.0);
        }
        else
        {
          ROS_ERROR("EXCAVATOR: Failed to call service Generate Waypoint");
        }
      }
    }
    else
    {
      ROS_ERROR("EXCAVATOR: Failed to call service Waypoint Checker");
    }
  }

  ClearCostmaps();

  Stop (2.0);

  Brake (0.0);

  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("EXCAVATOR: Sending goal to MoveBase: " << move_base_goal);
  ac.sendGoal(move_base_goal, boost::bind(&SmRd2::doneCallback, this,_1,_2), boost::bind(&SmRd2::activeCallback, this), boost::bind(&SmRd2::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));

  std_msgs::Int64 state_msg;
  state_msg.data = _planning;
  sm_state_pub.publish(state_msg);
}
void SmRd2::stateTraverse()
{
  ROS_WARN("Traverse State\n");

  if(flag_localized_base && !flag_have_true_pose)
  {
    ROS_INFO("EXCAVATOR: Localized but don't have true pose.");
    flag_have_true_pose = true;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }
  if(flag_volatile_recorded)
  {
    ROS_INFO("EXCAVATOR: Volatile recorded.");
    volatile_detected_distance = -1.0;
    flag_localizing_volatile = false;
    flag_volatile_recorded = false;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }
  if(flag_recovering_localization && !flag_localization_failure)
  {
    ROS_INFO("EXCAVATOR: Recovering localization or failure in localization.");
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
    flag_recovering_localization = false;
  }
  bool is_colliding = false;
    waypoint_checker::CheckCollision srv_wp_check;
    if (clt_waypoint_checker_.call(srv_wp_check))
    {
      ROS_INFO("EXCAVATOR: Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if(is_colliding)
      {
        ROS_INFO("EXCAVATOR: Waypoint Unreachable. Sending to Planning");
        flag_waypoint_unreachable=true;
      }
  }
  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
  {
    ROS_INFO("EXCAVATOR: Close to goal, getting new waypoint.");
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

  // if (!flag_mobility)
  // {
  //   ROS_INFO("EXCAVATOR: Recovering maneuver initialized.");
  //   immobilityRecovery();
  //   //flag_have_true_pose = true;
  // }

  move_base_state_ = ac.getState();
  int mb_state =(int) move_base_state_.state_;
  ROS_WARN_STREAM("MoveBase status: "<< mb_state);

  if(mb_state==5 || mb_state==7)
  {
    ROS_ERROR("EXCAVATOR: MoveBase has failed to make itself useful.");
    flag_waypoint_unreachable= true;

    Stop (1.0);

    ClearCostmaps();
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_pub.publish(state_msg);

}

void SmRd2::stateVolatileHandler()
{

  ROS_WARN("Volatile Handling State!");
}

void SmRd2::stateLost()
{
  ROS_ERROR("LOST STATE!\n");
  flag_recovering_localization = false;

  flag_localizing_volatile = false;
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  ROS_INFO("EXCAVATOR: Canceling MoveBase goal.");
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
  // ROS_INFO_STREAM("EXCAVATOR: Sending goal to MoveBase: " << move_base_goal);
  // ac.sendGoal(move_base_goal, boost::bind(&SmRd2::doneCallback, this,_1,_2), boost::bind(&SmRd2::activeCallback, this), boost::bind(&SmRd2::feedbackCallback, this,_1));
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
        ROS_INFO("EXCAVATOR: Called service ApproachBaseStation");
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
      ROS_ERROR("EXCAVATOR: Failed  to call service ApproachBaseStation");
    }
    homingRecoveryCount=homingRecoveryCount+1;
  }


  Brake (100.0);
  if(approachSuccess){
  // Homing - Measurement Update
  sensor_fusion::HomingUpdate srv_homing;
  ros::spinOnce();

  srv_homing.request.angle = pitch_ + .4; // pitch up is negative number
  ROS_INFO("Requesting Angle for LIDAR %f",srv_homing.request.angle);
  srv_homing.request.initializeLandmark = need_to_initialize_landmark;
  if (clt_homing_.call(srv_homing))
  {
    ROS_INFO("EXCAVATOR: Called service Homing [Update]");
    if(srv_homing.request.initializeLandmark && srv_homing.response.success){
        base_location_ = srv_homing.response.base_location;
        ROS_WARN("EXCAVATOR: Saving Base Location %f %f",base_location_.x, base_location_.y);
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
    ROS_ERROR("EXCAVATOR: Failed to call service Homing [Update]");
  }

}
else{
  ROS_ERROR(" Homing Attempt Failed, Just Moving On For Now");

}

  Lights ("0.6");

  Brake (0.0);

  Drive (-0.3, 2.0);

  Stop (2.0);

  RotateInPlace (0.2, 3.0);

  Stop (2.0);

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
void SmRd2::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = msg->data;
  if (flag_localized_base) {
    ROS_WARN_ONCE("Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

void SmRd2::mobilityCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_mobility = msg->data;
  if (flag_mobility) {
    ROS_WARN_ONCE("Rover is traversing = %i",flag_mobility);
  }
  else {
    ROS_ERROR("ROVER IMMOBILIZATION!  = %i",flag_mobility);
    immobilityRecovery();
  }
}

void SmRd2::waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable = msg->data;
}

void SmRd2::arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint = msg->data;
}

void SmRd2::volatileDetectedCallback(const std_msgs::Float32::ConstPtr& msg)
{

  prev_volatile_detected_distance = volatile_detected_distance;
  volatile_detected_distance = msg->data;

}

void SmRd2::volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg)
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

void SmRd2::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
}
void SmRd2::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_=msg->data;
}
void SmRd2::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev_ = yaw_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

}


void SmRd2::setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    poseGoal.target_pose.header.frame_id = "excavator_1_tf/odom";
    poseGoal.target_pose.pose.position.x = x;
    poseGoal.target_pose.pose.position.y = y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmRd2::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_ = true;
    // ROS_INFO("Goal done");
}
void SmRd2::activeCallback()
{
    // ROS_INFO("Goal went active");
}
void SmRd2::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    ROS_INFO("Got feedback");
}

void SmRd2::RotateToHeading(double desired_yaw)
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

     Drive (-0.3, 4.0);

  //  immobilityRecovery(); //TODO: Use this instead of Stop and Drive at line 714 and 716

    flag_heading_fail=false;
  }
  else
  {
    Stop(0.0);
  }
}

void SmRd2::homingRecovery()
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Homing Recovery.");

  Stop(2.0);

  Brake(100.0);

  Brake(0.0);

  Drive(-0.3, 4.0);

  Stop(0.0);

  RotateInPlace(.5,2.0);

  Stop(0.0);

  Drive(0.3, 4.0);

  Brake(100.0);

  Brake(0.0);



}

void SmRd2::immobilityRecovery()
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Recovery.");

  Stop(2.0);

  Brake(100.0);

  Brake(0.0);

  Drive(-0.3, 4.0);

  Stop(0.0);

  Brake(100.0);

  Brake(0.0);

  flag_waypoint_unreachable=true;



}

void SmRd2::ClearCostmaps()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/excavator_1/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/excavator_1/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("EXCAVATOR: Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR("EXCAVATOR: Failed calling clear_costmaps service.");
  }
}

void SmRd2::Lights(std::string intensity)
{
  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = intensity;
  if (clt_lights_.call(srv_lights))
  {
    ROS_INFO("EXCAVATOR: Called service ToggleLight");
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service ToggleLight");
  }
}

void SmRd2::RotateInPlace(double throttle, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = throttle;
  if (clt_rip_.call(srv_turn))
  {
    ROS_INFO("EXCAVATOR: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("EXCAVATOR: Failed to call service RotateInPlace");
  }
}

void SmRd2::Stop(double time)
{
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    ROS_INFO("EXCAVATOR: Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Stop");
  }
}

void SmRd2::Brake(double intensity)
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
    ROS_INFO_STREAM("EXCAVATOR: Called service SRCP2 Brake. Engaged?" << flag_brake_engaged);
  }
  else
  {
      ROS_ERROR("EXCAVATOR: Failed  to call service Brake");
  }
}

void SmRd2::Drive(double throttle, double time)
{
  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle = throttle;
  if (clt_drive_.call(srv_drive))
  {
    ROS_INFO("EXCAVATOR: Called service Drive");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Drive");
  }
}


void SmRd2::DriveCmdVel(double vx, double vy, double wz, double time)
{
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

void SmRd2::ToggleDetector(bool flag)
{
  volatile_handler::ToggleDetector srv_vol_detect;
  srv_vol_detect.request.on  = flag;
  if (clt_vol_detect_.call(srv_vol_detect))
  {
    ROS_INFO_STREAM("EXCAVATOR: Called service ToggleDetector. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service ToggleDetector");
  }
}

void SmRd2::RoverStatic(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static_.call(srv_rover_static))
  {
    ROS_INFO_STREAM("EXCAVATOR: Called service RoverStatic. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service RoverStatic");
  }

}

//------------------------------------------------------------------------------------------------------------------------
// HAULER
//------------------------------------------------------------------------------------------------------------------------

// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmRd2::localizedBaseCallbackHauler(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base_hauler_ = msg->data;
  if (flag_localized_base_hauler_) {
    ROS_WARN_ONCE("Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

void SmRd2::mobilityCallbackHauler(const std_msgs::Int64::ConstPtr& msg)
{
  flag_mobility_hauler_ = msg->data;
  if (flag_mobility_hauler_) {
    ROS_WARN_ONCE("Rover is traversing = %i",flag_mobility_hauler_);
  }
  else {
    ROS_ERROR("ROVER IMMOBILIZATION!  = %i",flag_mobilityv);
    immobilityRecoveryHauler();
  }
}

void SmRd2::waypointUnreachableCallbackHauler(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable_hauler_ = msg->data;
}

void SmRd2::arrivedAtWaypointCallbackHauler(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint_hauler_ = msg->data;
}

void SmRd2::localizationFailureCallbackHauler(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure_hauler_ = msg->data;
}
void SmRd2::drivingModeCallbackHauler(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_hauler_=msg->data;
}
void SmRd2::localizationCallbackHauler(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_hauler_ = msg->pose.pose;

  yaw_prev_hauler_ = yaw_hauler_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_hauler_, pitch_hauler_, yaw_hauler_);

}


void SmRd2::setPoseGoalHauler(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    poseGoal.target_pose.header.frame_id = "hauler_1_tf/odom";
    poseGoal.target_pose.pose.position.x = x;
    poseGoal.target_pose.pose.position.y = y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmRd2::doneCallbackHauler(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_hauler_ = true;
    // ROS_INFO("Goal done");
}
void SmRd2::activeCallbackHauler()
{
    // ROS_INFO("Goal went active");
}
void SmRd2::feedbackCallbackHauler(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    ROS_INFO("Got feedback");
}

void SmRd2::RotateToHeadingHauler(double desired_yaw)
{
  ros::Rate raterotateToHeading(20);

  ROS_INFO("Starting yaw control.");

  double yaw_thres = 0.1;

  ros::spinOnce();

  double yaw_error = desired_yaw - yaw_hauler_;
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
    RotateInPlaceHauler(copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error),0.0);

    raterotateToHeading.sleep();
    ros::spinOnce();

    yaw_error = desired_yaw - yaw_hauler_;
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
      flag_heading_fail_hauler_ = true;
      break;
    }
  }

  if (flag_heading_fail_hauler_)
  {
    ROS_WARN("Recovery action initiated in yaw control.");

     StopHauler(2.0);

     DriveHauler (-0.3, 4.0);

  //  immobilityRecovery(); //TODO: Use this instead of Stop and Drive at line 714 and 716

    flag_heading_fail_hauler_=false;
  }
  else
  {
    StopHauler(0.0);
  }
}

void SmRd2::homingRecoveryHauler()
{

  ac_hauler_.waitForServer();
  ac_hauler_.cancelGoal();
  ac_hauler_.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Homing Recovery.");

  StopHauler(2.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);

  DriveHauler(-0.3, 4.0);

  StopHauler(0.0);

  RotateInPlaceHauler(.5,2.0);

  StopHauler(0.0);

  DriveHauler(0.3, 4.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);



}

void SmRd2::immobilityRecoveryHauler()
{

  ac_hauler_.waitForServer();
  ac_hauler_.cancelGoal();
  ac_hauler_.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Recovery.");

  StopHauler(2.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);

  DriveHauler(-0.3, 4.0);

  StopHauler(0.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);

  flag_waypoint_unreachable_hauler_=true;



}

void SmRd2::ClearCostmapsHauler()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/hauler_1/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/hauler_1/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("HAULER: Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR("HAULER: Failed calling clear_costmaps service.");
  }
}

void SmRd2::LightsHauler(std::string intensity)
{
  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = intensity;
  if (clt_lights_hauler_.call(srv_lights))
  {
    ROS_INFO("HAULER: Called service ToggleLight");
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service ToggleLight");
  }
}

void SmRd2::RotateInPlaceHauler(double throttle, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = throttle;
  if (clt_rip_hauler_.call(srv_turn))
  {
    ROS_INFO("HAULER: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("HAULER: Failed to call service RotateInPlace");
  }
}

void SmRd2::StopHauler(double time)
{
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_hauler_.call(srv_stop))
  {
    ROS_INFO("HAULER: Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Stop");
  }
}

void SmRd2::BrakeHauler(double intensity)
{
  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = intensity;
  if (clt_srcp2_brake_rover_hauler_.call(srv_brake))
  {
    if (intensity < 0.01)
    {
      flag_brake_engaged_hauler_ =false;
    }
    else
    {
      flag_brake_engaged_hauler_ =true;
    }
    ROS_INFO_STREAM("HAULER: Called service SRCP2 Brake. Engaged?" << flag_brake_engaged_hauler_);
  }
  else
  {
      ROS_ERROR("HAULER: Failed  to call service Brake");
  }
}

void SmRd2::DriveHauler(double throttle, double time)
{
  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle = throttle;
  if (clt_drive_hauler_.call(srv_drive))
  {
    ROS_INFO("HAULER: Called service Drive");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Drive");
  }
}


void SmRd2::DriveCmdVelHauler(double vx, double vy, double wz, double time)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = wz;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 20 seconds
  ROS_ERROR("Drive Cmd Vel publisher.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub_hauler_.publish(cmd_vel);
  }
}

void SmRd2::RoverStaticHauler(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static_hauler_.call(srv_rover_static))
  {
    ROS_INFO_STREAM("HAULER: Called service RoverStatic. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("HAULER: Failed to call service RoverStatic");
  }

}void SmRd2::localizedBaseCallbackHauler(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base_hauler_ = msg->data;
  if (flag_localized_base_hauler_) {
    ROS_WARN_ONCE("HAULER: Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("HAULER: Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

void SmRd2::mobilityCallbackHauler(const std_msgs::Int64::ConstPtr& msg)
{
  flag_mobility_hauler_ = msg->data;
  if (flag_mobility_hauler_) {
    ROS_WARN_ONCE("HAULER: Rover is traversing = %i",flag_mobility_hauler_);
  }
  else {
    ROS_ERROR("HAULER: ROVER IMMOBILIZATION!  = %i",flag_mobilityv);
    immobilityRecoveryHauler();
  }
}

void SmRd2::waypointUnreachableCallbackHauler(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable_hauler_ = msg->data;
}

void SmRd2::arrivedAtWaypointCallbackHauler(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint_hauler_ = msg->data;
}

void SmRd2::localizationFailureCallbackHauler(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure_hauler_ = msg->data;
}
void SmRd2::drivingModeCallbackHauler(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_hauler_=msg->data;
}
void SmRd2::localizationCallbackHauler(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_hauler_ = msg->pose.pose;

  yaw_prev_hauler_ = yaw_hauler_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_hauler_, pitch_hauler_, yaw_hauler_);

}


void SmRd2::setPoseGoalHauler(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
{
    const double pitch = 0.0;
    const double roll = 0.0;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    poseGoal.target_pose.header.frame_id = "hauler_1_tf/odom";
    poseGoal.target_pose.pose.position.x = x;
    poseGoal.target_pose.pose.position.y = y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmRd2::doneCallbackHauler(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_hauler_ = true;
    // ROS_INFO("Goal done");
}
void SmRd2::activeCallbackHauler()
{
    // ROS_INFO("Goal went active");
}
void SmRd2::feedbackCallbackHauler(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    ROS_INFO("HAULER: Got feedback");
}

void SmRd2::RotateToHeadingHauler(double desired_yaw)
{
  ros::Rate raterotateToHeading(20);

  ROS_INFO("HAULER: Starting yaw control.");

  double yaw_thres = 0.1;

  ros::spinOnce();

  double yaw_error = desired_yaw - yaw_hauler_;
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
    RotateInPlaceHauler(copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error),0.0);

    raterotateToHeading.sleep();
    ros::spinOnce();

    yaw_error = desired_yaw - yaw_hauler_;
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
      ROS_ERROR("HAULER: Yaw Control Failed. Possibly stuck. Break control.");
      flag_heading_fail_hauler_ = true;
      break;
    }
  }

  if (flag_heading_fail_hauler_)
  {
    ROS_WARN("HAULER: Recovery action initiated in yaw control.");

     StopHauler(2.0);

     DriveHauler (-0.3, 4.0);

  //  immobilityRecovery(); //TODO: Use this instead of Stop and Drive at line 714 and 716

    flag_heading_fail_hauler_=false;
  }
  else
  {
    StopHauler(0.0);
  }
}

void SmRd2::homingRecoveryHauler()
{

  ac_hauler_.waitForServer();
  ac_hauler_.cancelGoal();
  ac_hauler_.waitForResult(ros::Duration(0.25));

  ROS_WARN("HAULER: Starting Homing Recovery.");

  StopHauler(2.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);

  DriveHauler(-0.3, 4.0);

  StopHauler(0.0);

  RotateInPlaceHauler(.5,2.0);

  StopHauler(0.0);

  DriveHauler(0.3, 4.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);



}

void SmRd2::immobilityRecoveryHauler()
{

  ac_hauler_.waitForServer();
  ac_hauler_.cancelGoal();
  ac_hauler_.waitForResult(ros::Duration(0.25));

  ROS_WARN("HAULER: Starting Recovery.");

  StopHauler(2.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);

  DriveHauler(-0.3, 4.0);

  StopHauler(0.0);

  BrakeHauler(100.0);

  BrakeHauler(0.0);

  flag_waypoint_unreachable_hauler_=true;



}

void SmRd2::ClearCostmapsHauler()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/hauler_1/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/hauler_1/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("HAULER: Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR("HAULER: Failed calling clear_costmaps service.");
  }
}

void SmRd2::LightsHauler(std::string intensity)
{
  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = intensity;
  if (clt_lights_hauler_.call(srv_lights))
  {
    ROS_INFO("HAULER: Called service ToggleLight");
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service ToggleLight");
  }
}

void SmRd2::RotateInPlaceHauler(double throttle, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = throttle;
  if (clt_rip_hauler_.call(srv_turn))
  {
    ROS_INFO("HAULER: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("HAULER: Failed to call service RotateInPlace");
  }
}

void SmRd2::StopHauler(double time)
{
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_hauler_.call(srv_stop))
  {
    ROS_INFO("HAULER: Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Stop");
  }
}

void SmRd2::BrakeHauler(double intensity)
{
  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = intensity;
  if (clt_srcp2_brake_rover_hauler_.call(srv_brake))
  {
    if (intensity < 0.01)
    {
      flag_brake_engaged_hauler_ =false;
    }
    else
    {
      flag_brake_engaged_hauler_ =true;
    }
    ROS_INFO_STREAM("HAULER: Called service SRCP2 Brake. Engaged?" << flag_brake_engaged_hauler_);
  }
  else
  {
      ROS_ERROR("HAULER: Failed  to call service Brake");
  }
}

void SmRd2::DriveHauler(double throttle, double time)
{
  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle = throttle;
  if (clt_drive_hauler_.call(srv_drive))
  {
    ROS_INFO("HAULER: Called service Drive");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Drive");
  }
}


void SmRd2::DriveCmdVelHauler(double vx, double vy, double wz, double time)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = vx;
  cmd_vel.linear.y = vy;
  cmd_vel.angular.z = wz;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 20 seconds
  ROS_ERROR("HAULER: Drive Cmd Vel publisher.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub_hauler_.publish(cmd_vel);
  }
}

void SmRd2::RoverStaticHauler(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static_hauler_.call(srv_rover_static))
  {
    ROS_INFO_STREAM("HAULER: Called service RoverStatic. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("HAULER: Failed to call service RoverStatic");
  }

}

//------------------------------------------------------------------------------------------------------------------------


