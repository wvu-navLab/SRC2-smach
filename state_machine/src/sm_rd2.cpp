#include <state_machine/sm_rd2.hpp>

SmRd2::SmRd2() :
ac_excavator_("/excavator_1/move_base", true),
move_base_state_excavator_(actionlib::SimpleClientGoalState::PREEMPTED),
ac_hauler_("/hauler_1/move_base", true),
move_base_state_hauler_(actionlib::SimpleClientGoalState::PREEMPTED)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers Excavator
  sm_state_pub_ = nh.advertise<std_msgs::Int64>("/state_machine/state_excavator", 1);
  cmd_vel_pub_excavator_ = nh.advertise<geometry_msgs::Twist>("/excavator_1/driving/cmd_vel", 1);
  manip_state_pub_excavator_ = nh.advertise<std_msgs::Int64>("/excavator_1/manipulation/state", 10);
  manip_volatile_pose_pub_excavator_ = nh.advertise<geometry_msgs::Pose>("/excavator_1/manipulation/volatile_pose", 10);

  // Subscribers Excavator
  localized_base_sub_excavator_ = nh.subscribe("/state_machine/localized_base_excavator", 1, &SmRd2::localizedBaseCallbackExcavator, this);
  mobility_sub_excavator_ = nh.subscribe("/state_machine/mobility_excavator", 10, &SmRd2::mobilityCallbackExcavator, this);
  waypoint_unreachable_sub_excavator_ = nh.subscribe("/state_machine/waypoint_unreachable_excavator", 1, &SmRd2::waypointUnreachableCallbackExcavator, this);
  arrived_at_waypoint_sub_excavator_ = nh.subscribe("/state_machine/arrived_at_waypoint_excavator", 1, &SmRd2::arrivedAtWaypointCallbackExcavator, this);
  localization_failure_sub_excavator_ = nh.subscribe("/state_machine/localization_failure_excavator", 1, &SmRd2::localizationFailureCallbackExcavator, this);
  localization_sub_excavator_  = nh.subscribe("/excavator_1/localization/odometry/sensor_fusion", 1, &SmRd2::localizationCallbackExcavator, this);
  driving_mode_sub_excavator_ =nh.subscribe("/excavator_1/driving/driving_mode",1, &SmRd2::drivingModeCallbackExcavator, this);
  manip_feedback_sub_excavator_ = nh.subscribe("/excavator_1/manipulation/feedback", 1, &SmRd2::manipulationFeedbackCallbackExcavator, this);

  // Clients Excavator
  clt_wp_gen_excavator_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("/excavator_1/navigation/generate_goal");
  clt_wp_start_excavator_ = nh.serviceClient<waypoint_gen::StartWaypoint>("/excavator_1/navigation/start");
  clt_wp_nav_set_goal_excavator_ = nh.serviceClient<waypoint_nav::SetGoal>("/excavator_1/navigation/set_goal");
  clt_wp_nav_interrupt_excavator_ = nh.serviceClient<waypoint_nav::Interrupt>("/excavator_1/navigation/interrupt");
  clt_stop_excavator_ = nh.serviceClient<driving_tools::Stop>("/excavator_1/driving/stop");
  clt_rip_excavator_ = nh.serviceClient<driving_tools::RotateInPlace>("/excavator_1/driving/rotate_in_place");
  clt_drive_excavator_ = nh.serviceClient<driving_tools::MoveForward>("/excavator_1/driving/move_forward");
  clt_lights_excavator_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("/excavator_1/toggle_light");
  clt_brake_excavator_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/excavator_1/brake_rover");
  clt_approach_base_excavator_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/excavator_1/approach_base_station");
  clt_rover_static_excavator_ = nh.serviceClient<sensor_fusion::RoverStatic>("/excavator_1/sensor_fusion/toggle_rover_static");
  clt_homing_excavator_ = nh.serviceClient<sensor_fusion::HomingUpdate>("/excavator_1/homing");
  clt_sf_true_pose_excavator_ = nh.serviceClient<sensor_fusion::GetTruePose>("/excavator_1/true_pose");
  clt_waypoint_checker_excavator_ = nh.serviceClient<waypoint_checker::CheckCollision>("/excavator_1/waypoint_checker");
  clt_srcp2_brake_rover_excavator_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/excavator_1/brake_rover");
  clt_next_vol_excavator_ = nh.serviceClient<round2_volatile_handler::NextVolatileLocation>("/excavator_1/volatile/next");
  clt_set_base_excavator_ = nh.serviceClient<sensor_fusion::SetBaseLocation>("/excavator_1/set_base_location");
  clt_location_of_base_excavator_ = nh.serviceClient<sensor_fusion::SetBaseLocation>("/excavator_1/location_of_base_service");

  clt_home_arm_excavator_ = nh.serviceClient<move_excavator::HomeArm>("/excavator_1/manipulation/home_arm");
  clt_dig_volatile_excavator_ = nh.serviceClient<move_excavator::DigVolatile>("/excavator_1/manipulation/dig_volatile");
  clt_scoop_excavator_ = nh.serviceClient<move_excavator::Scoop>("/excavator_1/manipulation/scoop");
  clt_extend_arm_excavator_ = nh.serviceClient<move_excavator::ExtendArm>("/excavator_1/manipulation/extend_arm");
  clt_drop_volatile_excavator_ = nh.serviceClient<move_excavator::DropVolatile>("/excavator_1/manipulation/drop_volatile");

  driving_mode_excavator_=0;

  //HAULER
  //Publishers
  cmd_vel_pub_hauler_ = nh.advertise<geometry_msgs::Twist>("/hauler_1/driving/cmd_vel", 1);

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
  clt_lights_hauler_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("/hauler_1/toggle_light");
  clt_brake_hauler_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/hauler_1/brake_rover");
  clt_approach_base_hauler_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/hauler_1/approach_base_station");
  clt_approach_excavator_hauler_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/hauler_1/approach_excavator");
  clt_rover_static_hauler_ = nh.serviceClient<sensor_fusion::RoverStatic>("/hauler_1/sensor_fusion/toggle_rover_static");
  clt_homing_hauler_ = nh.serviceClient<sensor_fusion::HomingUpdate>("/hauler_1/homing");
  clt_sf_true_pose_hauler_ = nh.serviceClient<sensor_fusion::GetTruePose>("/hauler_1/true_pose");
  clt_waypoint_checker_hauler_ = nh.serviceClient<waypoint_checker::CheckCollision>("/hauler_1/waypoint_checker");
  clt_srcp2_brake_rover_hauler_= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/hauler_1/brake_rover");
  clt_set_base_hauler_ = nh.serviceClient<sensor_fusion::SetBaseLocation>("/hauler_1/set_base_location");
  clt_location_of_base_hauler_ = nh.serviceClient<sensor_fusion::SetBaseLocation>("/hauler_1/location_of_base_service");

  need_to_initialize_landmark_excavator_=true;
  need_to_initialize_landmark_hauler_=true;

  // detection_timer = ros::Time::now();
  // not_detected_timer = ros::Time::now();
}

void SmRd2::run()
{
  ros::Rate loop_rate(5); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ROS_INFO("flag_localized_base_excavator_: %i",flag_localized_base_excavator_);
    ROS_INFO("flag_mobility_excavator_: %i",flag_mobility_excavator_);
    ROS_INFO("flag_have_true_pose_excavator_: %i",flag_have_true_pose_excavator_);
    ROS_INFO("flag_waypoint_unreachable_excavator_: %i",flag_waypoint_unreachable_excavator_);
    ROS_INFO("flag_arrived_at_waypoint_excavator_: %i",flag_arrived_at_waypoint_excavator_);
    ROS_INFO("flag_localizing_volatile_excavator_: %i",flag_localizing_volatile_excavator_);
    ROS_INFO("flag_volatile_dug_excavator_: %i",flag_volatile_dug_excavator_);
    ROS_INFO("flag_volatile_recorded_excavator_: %i",flag_volatile_recorded_excavator_);
    ROS_INFO("flag_volatile_unreachable_excavator_: %i",flag_volatile_unreachable_excavator_);
    ROS_INFO("flag_localization_failure_excavator_: %i",flag_localization_failure_excavator_);
    ROS_INFO("flag_brake_engaged_excavator_: %i",flag_brake_engaged_excavator_);
    ROS_INFO("flag_fallthrough_condition_: %i",flag_fallthrough_condition_);
    //---------------------------------------------------------------------------------------------------------------------


    // Conditional flag logic (Preemptive conditions) +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(!flag_volatile_dug_excavator_ && !flag_localizing_volatile_excavator_ && !flag_localization_failure_excavator_ && flag_have_true_pose_excavator_)
    {
      ROS_INFO("Preemptive condition1");
      flag_arrived_at_waypoint_excavator_ = true;
      flag_waypoint_unreachable_excavator_ = false;
    }
    if(flag_localization_failure_excavator_ && !flag_recovering_localization_excavator_)
    {
      ROS_INFO("Preemptive condition2");
      flag_arrived_at_waypoint_excavator_ = true;
      flag_waypoint_unreachable_excavator_ = false;
    }
    //---------------------------------------------------------------------------------------------------------------------


    // State machine truth table ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    state_to_exec.clear();
    state_to_exec.resize(num_states,0);
    if(!flag_have_true_pose_excavator_ && (flag_arrived_at_waypoint_excavator_ || flag_waypoint_unreachable_excavator_))
    {
      state_to_exec.at(_initialize) = 1;
    }
    else if((flag_arrived_at_waypoint_excavator_ || flag_waypoint_unreachable_excavator_) && (flag_localization_failure_excavator_ || flag_recovering_localization_excavator_))
    {
      state_to_exec.at(_lost) = 1;
    }
    else if((flag_arrived_at_waypoint_excavator_ || flag_waypoint_unreachable_excavator_) && !flag_localizing_volatile_excavator_ && !flag_brake_engaged_excavator_)
    {
      state_to_exec.at(_planning) = 1;
    }
    else if((!flag_arrived_at_waypoint_excavator_ && !flag_waypoint_unreachable_excavator_) && !flag_brake_engaged_excavator_)
    {
      state_to_exec.at(_traverse) = 1;
    }
    else if(!flag_volatile_dug_excavator_ && flag_brake_engaged_excavator_)
    {
      state_to_exec.at(_volatile_handler) = 1;
    }
    else
    {
      flag_arrived_at_waypoint_excavator_ = true;
      flag_waypoint_unreachable_excavator_ = false;
      flag_localizing_volatile_excavator_ = false;
      flag_volatile_dug_excavator_ = true;
      flag_fallthrough_condition_ = true;
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

    if(!flag_fallthrough_condition_ || state_to_exec_count == 1)
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
      ROS_WARN("State fallthough, flag_fallthrough_condition = %i, state_to_exec_count = %i",flag_fallthrough_condition_, state_to_exec_count);
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
  flag_arrived_at_waypoint_excavator_ = false;
  flag_waypoint_unreachable_excavator_ = false;

  // ToggleDetectorExcavator(false);

  // ToggleDetectorHauler(false);

  while(!clt_lights_hauler_.waitForExistence())
  {
    ROS_WARN("HAULER: Waiting for Light service");
  }

  LightsExcavator("0.8");
  ExecuteHomeArmExcavator(0.0, 1.0);
  BrakeExcavator(100.0);

  LightsHauler("0.6");
  StopHauler(1.0);
  BrakeHauler(100.0);
  BrakeHauler(0.0);

  // TODO: Hauler needs to disappear from sight
  // 1. Set higher confidence
  // 2. Filter by color?

  while (!clt_approach_base_hauler_.waitForExistence())
  {
    ROS_WARN("HAULER: Waiting for ApproachBaseStation service");
  }

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  bool approachSuccessHauler = false;
  int homingRecoveryCountHauler = 0;
  while(!approachSuccessHauler && homingRecoveryCountHauler<3)
  {
    if (clt_approach_base_hauler_.call(srv_approach_base))
    {
      ROS_INFO("HAULER: Called service ApproachBaseStation");
      ROS_INFO_STREAM("HAULER: Success finding the Base? "<< srv_approach_base.response.success.data);
      if(!srv_approach_base.response.success.data){
        homingRecoveryHauler();
      }
      else
      {
        approachSuccessHauler=true;
      }
    }
    else
    {
      ROS_ERROR("HAULER: Failed to call service ApproachBaseStation");
    }
    homingRecoveryCountHauler=homingRecoveryCountHauler+1;
  }

  BrakeHauler(100.0);
  RoverStaticHauler(true);
  while (!clt_sf_true_pose_hauler_ .waitForExistence())
  {
    ROS_ERROR("HAULER: Waiting for TruePose service");
  }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose_hauler_ .call(srv_sf_true_pose))
  {
    ROS_INFO("HAULER: Called service TruePose");
    ROS_INFO_STREAM("HAULER: Status of SF True Pose: "<< srv_sf_true_pose.response.success);
    flag_have_true_pose_hauler_ = true;
  }
  else
  {
    ROS_ERROR("HAULER: Failed to call service Pose Update");
  }

  RoverStaticHauler(false);

  BrakeExcavator(100.0);
  RoverStaticExcavator(true);

  while (!clt_sf_true_pose_excavator_ .waitForExistence())
  {
    ROS_ERROR("EXCAVATOR: Waiting for TruePose service");
  }

  // Update SF with True Pose
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose_excavator_ .call(srv_sf_true_pose))
  {
    ROS_INFO("EXCAVATOR: Called service TruePose");
    ROS_INFO_STREAM("EXCAVATOR: Status of SF True Pose: "<< srv_sf_true_pose.response.success);
    flag_have_true_pose_excavator_ = true;
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Pose Update");
  }

  RoverStaticExcavator(false);

  while (!clt_homing_hauler_.waitForExistence())
  {
    ROS_WARN("HAULER: Waiting for Homing service");
  }


  sensor_fusion::HomingUpdate srv_homing;
  if(approachSuccessHauler)
  {
    // Homing - Initialize Base Station Landmark
    ros::spinOnce();

    srv_homing.request.angle = pitch_hauler_ + .4; // pitch up is negative number
    ROS_ERROR("HAULER: Requesting Angle for LIDAR %f",srv_homing.request.angle);
    srv_homing.request.initializeLandmark = need_to_initialize_landmark_hauler_;
    if (clt_homing_hauler_.call(srv_homing))
    {
      ROS_INFO("HAULER: Called service HomingUpdate");
      if(srv_homing.response.success)
      {
      base_location_ = srv_homing.response.base_location;
      need_to_initialize_landmark_hauler_ = false;
      need_to_initialize_landmark_excavator_ = false;
      }
      else
      {
        ROS_ERROR("HAULER: Initial Homing Fail, Starting Without Base Location");
      }

      // pass the base location to Excavator
      sensor_fusion::SetBaseLocation srv_set_base_excavator;
      srv_set_base_excavator.request.base_location = base_location_;
      if(clt_set_base_excavator_.call(srv_set_base_excavator))
      {
        ROS_INFO("EXCAVATOR: Sent the Base Location to Excavator's Homing Node");
      }
      else{
        ROS_INFO("EXCAVATOR: Failed to Send the Base Location to Excavator's Homing Node");
      }


    }
    else
    {
      ROS_ERROR("HAULER: Failed to call service HomingUpdate");
    }
  }
  else
  {
    ROS_ERROR("HAULER: Initial Homing Fail, Starting Without Base Location");
  }



  LightsHauler("0.6");
  // BrakeHauler(0.0);
  // DriveHauler(-0.3, 3.0);
  // StopHauler(3.0);
  // RotateInPlaceHauler(0.3, 3.0);
  // StopHauler(3.0);
  // BrakeHauler(100.0);

  DriveCmdVelHauler(-0.5,0.0,0.0,4);
  BrakeRampHauler(100, 3, 0);
  BrakeHauler(0.0);
  RotateInPlaceHauler(0.2, 3);
  BrakeRampHauler(100, 3, 0);



  ClearCostmapsExcavator();
  BrakeExcavator(0.0);

  ClearCostmapsHauler();
  BrakeHauler(0.0);

  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_pub_.publish(state_msg);
}

void SmRd2::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint_excavator_ = false;
  flag_waypoint_unreachable_excavator_= false;

  ROS_INFO("EXCAVATOR: Canceling MoveBase goal.");
  ac_excavator_.waitForServer();
  ac_excavator_.cancelGoal();
  ac_excavator_.waitForResult(ros::Duration(0.25));

  StopExcavator(3.0);
  BrakeExcavator(100.0);

  ROS_INFO("HAULER: Canceling MoveBase goal.");
  ac_hauler_.waitForServer();
  ac_hauler_.cancelGoal();
  ac_hauler_.waitForResult(ros::Duration(0.25));

  StopHauler(3.0);
  BrakeHauler(100.0);

  // Generate Goal
  round2_volatile_handler::NextVolatileLocation srv_next_vol;
  srv_next_vol.request.rover_position  = current_pose_excavator_;

  ROS_INFO_STREAM("EXCAVATOR: Rover Pose: "<< current_pose_excavator_);
  ROS_INFO_STREAM("HAULER: Rover Pose: "<< current_pose_hauler_);

  if (clt_next_vol_excavator_.call(srv_next_vol))
  {
    ROS_WARN("EXCAVATOR: Called NextVolatileLocation service.");
    ROS_INFO_STREAM("Next Volatile Pose: "<< srv_next_vol.response.vol_position);
    goal_vol_pose_ = srv_next_vol.response.vol_position;
    goal_vol_type_ = srv_next_vol.response.type;
  }
  else
  {
    ROS_ERROR("EXCAVATOR Failed to call service NextVolatileLocation.");
  }

  UpdateGoalPoseExcavator();
  BrakeExcavator (0.0);
  RotateToHeadingExcavator(goal_yaw_excavator_);
  BrakeRampExcavator (100, 3, 0);

  UpdateGoalPoseHauler();
  BrakeHauler (0.0);
  RotateToHeadingHauler(goal_yaw_hauler_);
  BrakeRampHauler (100, 3, 0);

  ROS_INFO_STREAM("EXCAVATOR: Goal Pose: "<< goal_pose_excavator_);
  ROS_INFO_STREAM("HAULER: Goal Pose: "<< goal_pose_hauler_);

  // check waypoint

  // waypoint_checker::CheckCollision srv_wp_check;
  // bool is_colliding = true;
  // while (is_colliding)
  // {
  //   srv_wp_check.request.x  = goal_pose_excavator_.position.x;
  //   srv_wp_check.request.y = goal_pose_excavator_.position.y;
  //   if (clt_waypoint_checker_excavator_.call(srv_wp_check))
  //   {
  //     ROS_INFO("EXCAVATOR: Called service Waypoint Checker");
  //     is_colliding = srv_wp_check.response.collision;
  //     if(is_colliding)
  //     {
  //       ROS_INFO("EXCAVATOR: Waypoint Unreachable. Getting new waypoint");
  //       if (clt_next_vol_excavator_.call(srv_next_vol))
  //       {
  //         std::cout << "Calling Next Vol pose" << '\n';
  //         ROS_INFO_STREAM("Next Volatile Pose: "<< srv_next_vol.response.vol_position);
  //         goal_vol_pose_ = srv_next_vol.response.vol_position;
  //         goal_vol_type_ = srv_next_vol.response.type;
  //
  //         UpdateGoalPoseExcavator();
  //
  //         BrakeExcavator (0.0);
  //
  //         RotateToHeadingExcavator(goal_yaw_excavator_);
  //
  //         BrakeExcavator (100.0);
  //
  //       }
  //       else
  //       {
  //         ROS_ERROR("EXCAVATOR Failed to call service Next Volatile Pose");
  //       }
  //     }
  //   }
  //   else
  //   {
  //     ROS_ERROR("EXCAVATOR: Failed to call service Waypoint Checker");
  //   }
  // }

  ClearCostmapsExcavator();
  BrakeExcavator (0.0);

  move_base_msgs::MoveBaseGoal move_base_goal;
  ac_excavator_.waitForServer();
  setPoseGoalExcavator(move_base_goal, goal_pose_excavator_.position.x, goal_pose_excavator_.position.y, goal_yaw_excavator_);
  ROS_INFO_STREAM("EXCAVATOR: Sending goal to MoveBase: " << move_base_goal);
  ac_excavator_.sendGoal(move_base_goal, boost::bind(&SmRd2::doneCallbackExcavator, this,_1,_2), boost::bind(&SmRd2::activeCallbackExcavator, this), boost::bind(&SmRd2::feedbackCallbackExcavator, this,_1));
  ac_excavator_.waitForResult(ros::Duration(0.25));

  ClearCostmapsHauler();
  BrakeHauler (0.0);

  ac_hauler_.waitForServer();
  setPoseGoalHauler(move_base_goal, goal_pose_hauler_.position.x, goal_pose_hauler_.position.y, goal_yaw_hauler_);
  ROS_INFO_STREAM("HAULER: Sending goal to MoveBase: " << move_base_goal);
  ac_hauler_.sendGoal(move_base_goal, boost::bind(&SmRd2::doneCallbackHauler, this,_1,_2), boost::bind(&SmRd2::activeCallbackHauler, this), boost::bind(&SmRd2::feedbackCallbackHauler, this,_1));
  ac_hauler_.waitForResult(ros::Duration(0.25));

  std_msgs::Int64 state_msg;
  state_msg.data = _planning;
  sm_state_pub_.publish(state_msg);
}

void SmRd2::stateTraverse()
{
  ROS_WARN("Traverse State\n");

  if(flag_localized_base_excavator_ && !flag_have_true_pose_excavator_)
  {
    ROS_INFO("EXCAVATOR: Localized but don't have true pose.");
    flag_have_true_pose_excavator_ = true;
    flag_arrived_at_waypoint_excavator_ = true;
    flag_waypoint_unreachable_excavator_ = false;
  }

  if(flag_recovering_localization_excavator_ && !flag_localization_failure_excavator_)
  {
    ROS_INFO("EXCAVATOR: Recovering localization or failure in localization.");
    flag_arrived_at_waypoint_excavator_ = true;
    flag_waypoint_unreachable_excavator_ = false;
    flag_recovering_localization_excavator_ = false;
  }

  double distance_to_goal = std::hypot(goal_pose_excavator_.position.y - current_pose_excavator_.position.y, goal_pose_excavator_.position.x - current_pose_excavator_.position.x);
  if (distance_to_goal < 2.0)
  {
    ROS_INFO("EXCAVATOR: Close to goal, getting new waypoint.");
    flag_arrived_at_waypoint_excavator_ = true;
    flag_waypoint_unreachable_excavator_ = false;
    ROS_INFO_STREAM("Let's start manipulation.");
    flag_volatile_dug_excavator_ = false;
    flag_localizing_volatile_excavator_ = true;

    ROS_INFO("EXCAVATOR: Canceling MoveBase goal.");
    ac_excavator_.waitForServer();
    ac_excavator_.cancelGoal();
    ac_excavator_.waitForResult(ros::Duration(0.25));

    BrakeExcavator(100.0);
  }

  bool is_colliding = false;
  waypoint_checker::CheckCollision srv_wp_check;
  if (clt_waypoint_checker_excavator_.call(srv_wp_check))
  {
    ROS_INFO("EXCAVATOR: Called service Waypoint Checker");
    is_colliding = srv_wp_check.response.collision;
    if(is_colliding)
    {
      ROS_INFO("EXCAVATOR: Waypoint Unreachable. Sending to Planning");
      flag_waypoint_unreachable_excavator_=true;
    }
  }

  if (!flag_mobility_excavator_)
  {
    ROS_INFO("EXCAVATOR: Recovering maneuver initialized.");
    immobilityRecoveryExcavator();
  }

  if (!flag_mobility_hauler_)
  {
    ROS_INFO("HAULER: Recovering maneuver initialized.");
    immobilityRecoveryHauler();
  }

  move_base_state_excavator_ = ac_excavator_.getState();
  int mb_state_excavator =(int) move_base_state_excavator_.state_;
  ROS_WARN_STREAM("EXCAVATOR: MoveBase status: "<< mb_state_excavator);

  if(mb_state_excavator==5 || mb_state_excavator==7)
  {
    ROS_ERROR("EXCAVATOR: MoveBase has failed to make itself useful.");
    flag_waypoint_unreachable_excavator_= true;

    StopExcavator (2.0);
    ClearCostmapsExcavator();
  }

  move_base_state_hauler_ = ac_hauler_.getState();
  int mb_state_hauler =(int) move_base_state_hauler_.state_;
  ROS_WARN_STREAM("HAULER: MoveBase status: "<< mb_state_hauler);

  if(mb_state_hauler==5 || mb_state_hauler==7)
  {
    ROS_ERROR("EXCAVATOR: MoveBase has failed to make itself useful.");
    flag_waypoint_unreachable_excavator_= true;

    StopExcavator (2.0);
    ClearCostmapsHauler();
  }
//////////////////////////////////////////////////////////////////////////
// IF IT'S NEEDED THE TIMEOUT FOR CLEARING COSTMAP AND WAYPOINT IS here //
//////////////////////////////////////////////////////////////////////////

// ros::Duration timeoutMapHauler(30.0);
//
// if (ros::Time::now() - map_timerHauler > timeoutMapHauler)
// {
//   std_srvs::Empty emptymsg;
//   ros::service::call("/hauler_1/move_base/clear_costmaps",emptymsg);
//   map_timerHauler =ros::Time::now();
//   ROS_WARN("Map Cleared for Hauler");
// }
//
// if (ros::Time::now() - map_timerExcavator > timeoutMapExcavator)
// {
//   std_srvs::Empty emptymsg;
//   ros::service::call("/excavator_1/move_base/clear_costmaps",emptymsg);
//   map_timerExcavator =ros::Time::now();
//   ROS_WARN("Map Cleared for Excavator");
// }
//
// ros::Duration timeoutWaypointHauler(120);
// if (ros::Time::now() - waypoint_timer_Hauler > timeoutWaypointHauler )
// {
//   ROS_ERROR("Waypoint Unreachable for Hauler");
//   flag_waypoint_unreachable_hauler_= true;
//   StopHauler (1.0);
//   ClearCostmapsHauler();
// }
// ros::Duration timeoutWaypointExcavator(120);
// if (ros::Time::now() - waypoint_timer_Excavator > timeoutWaypointExcavator )
// {
//   ROS_ERROR("Waypoint Unreachable for Excavator");
//   flag_waypoint_unreachable_excavator_= true;
//   StopExcavator (1.0);
//   ClearCostmapsExcavator();
// }


  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_pub_.publish(state_msg);
}

void SmRd2::stateVolatileHandler()
{
  ROS_WARN("Excavation State\n");

  BrakeExcavator(100.0);
  StartManipulation();

  ros::Rate manipulation_rate(10);

  // TODO: Location of Base Station:
  // This service will give (x,y) in global frame of hauler
  // Subtract (x,y) estimate of the hauler
  // Put some sanity check
  bool approachSuccessHauler = false;

  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  int homingRecoveryCountHauler = 0;
  while(!approachSuccessHauler && homingRecoveryCountHauler<3)
  {
    if (clt_approach_excavator_hauler_.call(srv_approach_base))
    {
      ROS_INFO("HAULER: Called service ApproachExcavator");
      ROS_INFO_STREAM("Success finding the Excavator? "<< srv_approach_base.response.success.data);
      if(!srv_approach_base.response.success.data)
      {
        homingRecoveryHauler();
      }
      else
      {
        approachSuccessHauler = true;
      }
    }
    else
    {
      ROS_ERROR("HAULER: Failed  to call service ApproachExcavator");
    }
    homingRecoveryCountHauler=homingRecoveryCountHauler+1;
  }

  std::vector<double> vx {-1.0, 2.0, -1.0, 0.01};
  std::vector<double> vy {-0.0, 0.0, -1.0, 2.0};

  int position_counter = 0;
  while(position_counter < 3)
  {
    while(!flag_volatile_found_excavator_)
    {
      if(approachSuccessHauler)
      {
        // TODO: Service come closer and send relative yaw
      }
      else
      { 
        // TODO: What?
      }

      ROS_WARN_THROTTLE(10, "In Manipulation State Machine");
      ros::spinOnce();
      manipulation_rate.sleep();
    }
    position_counter++;
    BrakeExcavator(0.0);
    DriveCmdVelExcavator(vx[position_counter], vy[position_counter], 0.0, 2.0);
    BrakeRampExcavator(100, 3.0, 0);
  }

  DriveCmdVelHauler(-0.5, 0.0, 0.0, 4.0);
  BrakeRampHauler(100,3.0,0);

  BrakeExcavator(0.0);
  BrakeHauler(0.0);

  // TODO: SETUP FLAGS TO GO TO PLANNING
  flag_arrived_at_waypoint_excavator_ = true;

  std_msgs::Int64 state_msg;
  state_msg.data = _volatile_handler;
  sm_state_pub_.publish(state_msg);
}

void SmRd2::stateLost()
{
  ROS_ERROR("LOST STATE!\n");
  flag_recovering_localization_excavator_ = false;

  flag_localizing_volatile_excavator_ = false;
  flag_arrived_at_waypoint_excavator_ = false;
  flag_waypoint_unreachable_excavator_ = false;

  ROS_INFO("EXCAVATOR: Canceling MoveBase goal.");
  ac_excavator_.waitForServer();
  ac_excavator_.cancelGoal();
  ac_excavator_.waitForResult(ros::Duration(0.25));


  StopExcavator (2.0);

  LightsExcavator ("0.8");


  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  bool approachSuccess = false;
  int homingRecoveryCount=0;
  while(!approachSuccess && homingRecoveryCount<3){
      if (clt_approach_base_excavator_.call(srv_approach_base))
      {
        ROS_INFO("EXCAVATOR: Called service ApproachBaseStation");
        ROS_INFO_STREAM("Success finding the Base? "<< srv_approach_base.response.success.data);
        if(!srv_approach_base.response.success.data){
        homingRecoveryExcavator();
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


  BrakeExcavator (100.0);
  if(approachSuccess){
  // Homing - Measurement Update
  sensor_fusion::HomingUpdate srv_homing;

  ros::spinOnce();

  srv_homing.request.angle = pitch_excavator_ + .4; // pitch up is negative number
  ROS_INFO("Requesting Angle for LIDAR %f",srv_homing.request.angle);
  srv_homing.request.initializeLandmark = need_to_initialize_landmark_excavator_;
  if (clt_homing_excavator_.call(srv_homing))
  {
    ROS_INFO("EXCAVATOR: Called service Homing [Update]");
    if(srv_homing.request.initializeLandmark && srv_homing.response.success){
      base_location_ = srv_homing.response.base_location;
      ROS_WARN("EXCAVATOR: Saving Base Location %f %f",base_location_.x, base_location_.y);
    }
    flag_localization_failure_excavator_=false;
    flag_arrived_at_waypoint_excavator_ = true;
    flag_completed_homing_excavator_ = true;
    if(srv_homing.response.success){
      need_to_initialize_landmark_excavator_=false;
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

  LightsExcavator ("0.6");

  // BrakeExcavator (0.0);
  //
  // DriveExcavator (-0.3, 2.0);
  //
  // StopExcavator (3.0);
  //
  // RotateInPlaceExcavator (0.2, 3.0);
  //
  // StopExcavator (3.0);

  BrakeExcavator(0.0);

  DriveCmdVelExcavator(-0.5,0.0,0.0,4);

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);

  RotateInPlaceExcavator(0.2, 3);

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);

  // ToggleDetectorExcavator(true);

  ClearCostmapsExcavator();

  std_msgs::Int64 state_msg;
  state_msg.data = _lost;
  sm_state_pub_.publish(state_msg);
}
//------------------------------------------------------------------------------------------------------------------------


// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmRd2::localizedBaseCallbackExcavator(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base_excavator_ = msg->data;
  if (flag_localized_base_excavator_) {
    ROS_WARN_ONCE("EXCAVATOR Initial Localization Successful = %i",flag_localized_base_excavator_);

  }
  else {
    ROS_INFO("EXCAVATOR Waiting for Initial Localization  = %i",flag_localized_base_excavator_);
  }
}

void SmRd2::mobilityCallbackExcavator(const std_msgs::Int64::ConstPtr& msg)
{
  flag_mobility_excavator_ = msg->data;
  if (flag_mobility_excavator_) {
    ROS_WARN_ONCE("EXCAVATOR Rover is traversing = %i",flag_mobility_excavator_);
  }
  else {
    ROS_ERROR("EXCAVATOR OVER IMMOBILIZATION!  = %i",flag_mobility_excavator_);
    immobilityRecoveryExcavator();
  }
}

void SmRd2::waypointUnreachableCallbackExcavator(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable_excavator_ = msg->data;
}

void SmRd2::arrivedAtWaypointCallbackExcavator(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint_excavator_ = msg->data;
}


void SmRd2::localizationFailureCallbackExcavator(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure_excavator_ = msg->data;
}
void SmRd2::drivingModeCallbackExcavator(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_excavator_=msg->data;
}
void SmRd2::localizationCallbackExcavator(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_excavator_ = msg->pose.pose;

  yaw_prev_excavator_ = yaw_excavator_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_excavator_, pitch_excavator_, yaw_excavator_);

}


void SmRd2::setPoseGoalExcavator(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
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

void SmRd2::doneCallbackExcavator(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_excavator_= true;
    // ROS_INFO("Goal done");
}
void SmRd2::activeCallbackExcavator()
{
    // ROS_INFO("Goal went active");
}
void SmRd2::feedbackCallbackExcavator(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    // ROS_INFO("Got feedback");
}

void SmRd2::manipulationFeedbackCallbackExcavator(const move_excavator::ExcavationStatus::ConstPtr& msg)
{
  collected_mass_excavator_ = collected_mass_excavator_ + msg->collectedMass;
  flag_volatile_dug_excavator_ = msg->isFinished;
  flag_volatile_found_excavator_ = msg->foundVolatile;
}

void SmRd2::UpdateGoalPoseExcavator(){
  double dx = goal_vol_pose_.position.x - current_pose_excavator_.position.x;
  double dy = goal_vol_pose_.position.y - current_pose_excavator_.position.y;
  goal_yaw_excavator_ = atan2(dy, dx);

  goal_pose_excavator_.position.x = goal_vol_pose_.position.x - dx / hypot(dx,dy) * 0.5;
  goal_pose_excavator_.position.y = goal_vol_pose_.position.y - dy / hypot(dx,dy) * 0.5;
}


void SmRd2::RotateToHeadingExcavator(double desired_yaw)
{
  ros::Rate raterotateToHeading(20);

  ROS_INFO("EXCAVATOR: Starting yaw control.");

  double yaw_thres = 0.1;

  ros::spinOnce();

  double yaw_error = desired_yaw - yaw_excavator_;
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
  flag_heading_fail_excavator_=false;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeoutHeading(10.0); // Timeout of 20 seconds

  while(fabs(yaw_error) > yaw_thres)
  {
    RotateInPlaceExcavator(copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error),0.0);

    raterotateToHeading.sleep();
    ros::spinOnce();

    yaw_error = desired_yaw - yaw_excavator_;
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
      ROS_ERROR("EXCAVATOR: Yaw Control Failed. Possibly stuck. Break control.");
      flag_heading_fail_excavator_ = true;
      break;
    }
  }

  if (flag_heading_fail_excavator_)
  {
    ROS_WARN("EXCAVATOR: Recovery action initiated in yaw control.");

    StopExcavator(2.0);

    DriveCmdVelExcavator(-0.5, 0.0, 0.0, 4.0);

    BrakeRampExcavator(100, 3, 0);

    BrakeExcavator(0.0);

  //  immobilityRecovery(); //TODO: Use this instead of Stop and Drive at line 714 and 716

    flag_heading_fail_excavator_=false;
  }
  else
  {
    StopExcavator(2.0);
  }
}

void SmRd2::homingRecoveryExcavator()
{

  ac_excavator_.waitForServer();
  ac_excavator_.cancelGoal();
  ac_excavator_.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Homing Recovery.");

  StopExcavator(2.0);

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);

  DriveCmdVelExcavator(-0.3,-0.6, 0.0, 4.0);

  StopExcavator(0.0);

  RotateToHeadingExcavator(yaw_excavator_ - M_PI/4);

  StopExcavator(0.0);

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);

  DriveCmdVelExcavator(0.7,0.0,0.0,4.0);

  StopExcavator(0.0);

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);



}

void SmRd2::immobilityRecoveryExcavator()
{

  ac_excavator_.waitForServer();
  ac_excavator_.cancelGoal();
  ac_excavator_.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Recovery.");

  StopExcavator(2.0);

  BrakeExcavator(100.0);

  BrakeExcavator(0.0);

  DriveCmdVelExcavator(-0.5,0.0,0.0,3.0);

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);

  DriveCmdVelExcavator(-0.3,-0.6, 0.0, 4.0);

  // Stop(3.0); //TODO: CMDvelZero try

  BrakeRampExcavator(100, 3, 0);

  BrakeExcavator(0.0);

  flag_waypoint_unreachable_excavator_=true;



}

void SmRd2::ClearCostmapsExcavator()
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

void SmRd2::LightsExcavator(std::string intensity)
{
  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = intensity;
  if (clt_lights_excavator_.call(srv_lights))
  {
    ROS_INFO("EXCAVATOR: Called service ToggleLight");
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service ToggleLight");
  }
}

void SmRd2::RotateInPlaceExcavator(double throttle, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = throttle;
  if (clt_rip_excavator_.call(srv_turn))
  {
    ROS_INFO("EXCAVATOR: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("EXCAVATOR: Failed to call service RotateInPlace");
  }
}

void SmRd2::StopExcavator(double time)
{
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    ROS_INFO("EXCAVATOR: Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Stop");
  }
}

void SmRd2::BrakeExcavator(double intensity)
{
  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = intensity;
  if (clt_srcp2_brake_rover_excavator_.call(srv_brake))
  {
    if (intensity < 0.01)
    {
      flag_brake_engaged_excavator_ =false;
    }
    else
    {
      flag_brake_engaged_excavator_ =true;
    }
    ROS_INFO_STREAM("EXCAVATOR: Called service SRCP2 Brake. Engaged?" << flag_brake_engaged_excavator_);
  }
  else
  {
      ROS_ERROR("EXCAVATOR: Failed  to call service Brake");
  }
}

void SmRd2::DriveExcavator(double throttle, double time)
{
  driving_tools::MoveForward srv_drive;

  if (pitch_excavator_ > 0.0 && throttle < 0.0) {
   throttle = throttle - (pitch_excavator_ / 0.52) * 0.3;
  }
  srv_drive.request.throttle = throttle;

  if (clt_drive_excavator_.call(srv_drive))
  {
    ROS_INFO("EXCAVATOR: Called service Drive");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Drive");
  }
}


void SmRd2::DriveCmdVelExcavator(double vx, double vy, double wz, double time)
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
    cmd_vel_pub_excavator_.publish(cmd_vel);
  }
}

void SmRd2::BrakeRampExcavator(double max_intensity, double time, int aggressivity)
{
  double freq = 10;
  ros::Rate brake_rate(freq);
  int num_steps = (int) freq * time;
  if(aggressivity == 0)
  {
    // ROS_INFO("EXCAVATOR: Brake Ramp.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double intensity = (static_cast<double>(counter + 1)/(freq * time))*max_intensity;
      // ROS_INFO_STREAM("Brake intensity: " << intensity);
      BrakeExcavator(intensity);
      brake_rate.sleep();
    }
  }
  else if (aggressivity == 1)
  {
    // ROS_INFO("EXCAVATOR: Brake Logistics Curve.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double multiplier = 2;
      double x = (static_cast<double>(counter + 1)/(freq * time)) * time * multiplier;
      double intensity =  max_intensity / (1 + exp(-x)) - max_intensity/2;
      // ROS_INFO_STREAM("Brake intensity: " << intensity);
      BrakeExcavator(intensity);
      brake_rate.sleep();
    }
  }
  else
  {
    ROS_INFO("EXCAVATOR: Brake FULL.");
    BrakeExcavator(max_intensity);
    ros::Duration(time).sleep();
  }
}

void SmRd2::RoverStaticExcavator(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static_excavator_.call(srv_rover_static))
  {
    ROS_INFO_STREAM("EXCAVATOR: Called service RoverStatic. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service RoverStatic");
  }

}

void SmRd2::StartManipulation()
{

  std_msgs::Int64 msg;
  msg.data = 9;
  ros::service::waitForService("/excavator_1/manipulation/home_arm",ros::Duration(3.0));
  manip_state_pub_excavator_.publish(msg);
  ROS_INFO_STREAM("Start 9="<<  msg.data);

  manip_volatile_pose_pub_excavator_.publish(goal_vol_pose_);
  ROS_INFO_STREAM("Goal volatile pose: "<<goal_vol_pose_);

}

void SmRd2::ExecuteHomeArmExcavator(double heading, double time)
{
  move_excavator::HomeArm srv;
  srv.request.heading = heading;
  srv.request.timeLimit = time;
  // bool success = clt_home_arm_excavator_.call(srv);
  if (clt_home_arm_excavator_.call(srv))
  {
    ROS_INFO("EXCAVATOR: Called service Home Arm");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Home Arm");
  }
}

void SmRd2::ExecuteDigExcavator(double heading, double time)
{
  move_excavator::DigVolatile srv;
  srv.request.heading = heading;
  srv.request.timeLimit = time;
  // bool success = clt_dig_volatile_excavator_.call(srv);
  if (clt_dig_volatile_excavator_.call(srv))
  {
    ROS_INFO("EXCAVATOR: Called service Dig Volatile");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Dig Volatile");
  }
}

void SmRd2::ExecuteScoopExcavator(double heading, double time)
{
  move_excavator::Scoop srv;
  srv.request.heading = heading;
  srv.request.timeLimit = time;
  // bool success = clt_scoop_excavator_.call(srv);
  if (clt_scoop_excavator_.call(srv))
  {
    ROS_INFO("EXCAVATOR: Called service Scoop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Scoop");
  }
}

void SmRd2::ExecuteExtendArmExcavator(double heading, double time)
{
  move_excavator::ExtendArm srv;
  srv.request.heading = heading;
  srv.request.timeLimit = time;
  // bool success = clt_extend_arm_excavator_.call(srv);
  if (clt_extend_arm_excavator_.call(srv))
  {
    ROS_INFO("EXCAVATOR: Called service Extend Arm");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Extend Arm");
  }
}

void SmRd2::ExecuteDropExcavator(double heading, double time)
{
  move_excavator::DropVolatile srv;
  srv.request.heading = heading;
  srv.request.timeLimit = time;
  // bool success = clt_drop_volatile_excavator_.call(srv);
  if (clt_drop_volatile_excavator_.call(srv))
  {
    ROS_INFO("EXCAVATOR: Called service Drop Volatile");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed  to call service Drop Volatile");
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
    ROS_WARN_ONCE("HAULER: Initial Localization Successful = %i",flag_localized_base_hauler_);

  }
  else {
    ROS_INFO("HAULER: Waiting for Initial Localization  = %i",flag_localized_base_hauler_);
  }
}

void SmRd2::mobilityCallbackHauler(const std_msgs::Int64::ConstPtr& msg)
{
  flag_mobility_hauler_ = msg->data;
  if (flag_mobility_hauler_) {
    ROS_WARN_ONCE("HAULER: Rover is traversing = %i",flag_mobility_hauler_);
  }
  else {
    ROS_ERROR("HAULER: ROVER IMMOBILIZATION!  = %i",flag_mobility_hauler_);
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

void SmRd2::UpdateGoalPoseHauler(){
  double dx = goal_vol_pose_.position.x - current_pose_hauler_.position.x;
  double dy = goal_vol_pose_.position.y - current_pose_hauler_.position.y;
  goal_yaw_hauler_ = atan2(dy, dx);

  goal_pose_hauler_.position.x = goal_vol_pose_.position.x - dx / hypot(dx,dy) * 4;
  goal_pose_hauler_.position.y = goal_vol_pose_.position.y - dy / hypot(dx,dy) * 4;
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
    // ROS_INFO("Got feedback");
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
  flag_heading_fail_hauler_=false;
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

    DriveCmdVelHauler (-0.5, 0.0, 0.0, 4.0);

    BrakeRampHauler(100, 3, 0);

    BrakeHauler(0.0);

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

  BrakeRampHauler(100, 3, 0);

  BrakeHauler(0.0);

  DriveCmdVelHauler(-0.3,-0.6, 0.0, 4.0);

  StopHauler(0.0);

  RotateToHeadingHauler(yaw_hauler_ - M_PI/4);

  StopHauler(0.0);

  BrakeRampHauler(100, 3, 0);

  DriveCmdVelHauler(0.7,0.0,0.0,4.0);

  BrakeRampHauler(100, 3, 0);

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

  DriveCmdVelHauler(-0.5,0.0, 0.0, 3.0);

  BrakeRampHauler(100, 3, 0);

  // BrakeHauler(100.0);
  BrakeHauler(0.0);

  DriveCmdVelHauler(-0.3,-0.6, 0.0, 4.0);

  BrakeRampHauler(100, 3, 0);

  BrakeHauler(0.0);

  flag_mobility_hauler_=true;

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

  if (pitch_hauler_ > 0.0 && throttle < 0.0) {
   throttle = throttle - (pitch_hauler_ / 0.52) * 0.3;
  }
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

void SmRd2::BrakeRampHauler(double max_intensity, double time, int aggressivity)
{
  double freq = 10;
  ros::Rate brake_rate(freq);
  int num_steps = (int) freq * time;
  if(aggressivity == 0)
  {
    // ROS_INFO("HAULER: Brake Ramp.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double intensity = (static_cast<double>(counter + 1)/(freq * time))*max_intensity;
      // ROS_INFO_STREAM("Brake intensity: " << intensity);
      BrakeHauler(intensity);
      brake_rate.sleep();
    }
  }
  else if (aggressivity == 1)
  {
    // ROS_INFO("HAULER: Brake Logistics Curve.");
    for (int counter = 0; counter < num_steps; ++counter)
    {
      double multiplier = 2;
      double x = (static_cast<double>(counter + 1)/(freq * time)) * time * multiplier;
      double intensity =  max_intensity / (1 + exp(-x)) - max_intensity/2;
      // ROS_INFO_STREAM("Brake intensity: " << intensity);
      BrakeHauler(intensity);
      brake_rate.sleep();
    }
  }
  else
  {
    ROS_INFO("HAULER: Brake FULL.");
    BrakeHauler(max_intensity);
    ros::Duration(time).sleep();
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
