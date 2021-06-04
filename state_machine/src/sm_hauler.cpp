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
  // mobility_sub = nh.subscribe("/state_machine/mobility_scout", 1, &SmHauler::mobilityCallback, this);
  waypoint_unreachable_sub = nh.subscribe("state_machine/waypoint_unreachable", 1, &SmHauler::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("state_machine/arrived_at_waypoint", 1, &SmHauler::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("state_machine/volatile_detected", 1, &SmHauler::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("state_machine/volatile_recorded", 1, &SmHauler::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("state_machine/localization_failure", 1, &SmHauler::localizationFailureCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmHauler::localizationCallback, this);
  driving_mode_sub =nh.subscribe("driving/driving_mode",1, &SmHauler::drivingModeCallback, this);
  laser_scan_sub =nh.subscribe("laser/scan",1, &SmHauler::laserCallback, this);
  planner_interrupt_sub = nh.subscribe("/planner_interrupt", 1, &SmHauler::plannerInterruptCallback, this);

  // Clients
  clt_wp_gen = nh.serviceClient<waypoint_gen::GenerateWaypoint>("navigation/generate_goal");
  clt_wp_start = nh.serviceClient<waypoint_gen::StartWaypoint>("navigation/start");
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

  srv_mobility = nh.advertiseService("state_machine/mobility_service",&SmHauler::setMobility, this);

  driving_mode =0;
  waypoint_type =0;
  flag_need_init_landmark=true;

  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();
  last_time_laser_collision = ros::Time::now();
  map_timer = ros::Time::now();
  wp_checker_timer=  ros::Time::now();

  node_name_ = "state_machine";
  if (ros::param::get(node_name_ + "/robot_name", robot_name_) == false)
  {
    ROS_FATAL("No parameter 'robot_name' specified");
    ros::shutdown();
    exit(1);
  }
  if (ros::param::get(node_name_ + "/robot_id", robot_id_) == false)
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
    ROS_INFO("flag_have_true_pose: %i",flag_have_true_pose);
    ROS_INFO("flag_interrupt_plan: %i",flag_interrupt_plan);
    ROS_INFO("flag_arrived_at_waypoint: %i",flag_arrived_at_waypoint);
    ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    ROS_INFO("flag_dumping: %i",flag_dumping);
    ROS_INFO("flag_recovering_localization: %i",flag_recovering_localization);
    ROS_INFO("flag_brake_engaged: %i",flag_brake_engaged);
    //---------------------------------------------------------------------------------------------------------------------


    // // Conditional flag logic (Preemptive conditions) +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // if((volatile_detected_distance>0.0) && !flag_localizing_volatile && !flag_localization_failure && flag_have_true_pose)
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
    else if(flag_interrupt_plan || (flag_arrived_at_waypoint && !flag_recovering_localization && !flag_localizing_volatile && !flag_dumping && !flag_brake_engaged))
    {
      state_to_exec.at(_planning) = 1;
    }
    else if(flag_arrived_at_waypoint && flag_recovering_localization && !flag_brake_engaged)
    {
      state_to_exec.at(_lost) = 1;
    }
    else if(flag_arrived_at_waypoint && flag_localizing_volatile && !flag_brake_engaged && flag_volatile_handler)
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


    // State execution ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // int state_to_exec_count = 0;
    // for(int i=0; i<state_to_exec.size(); i++)
    // {
    //   if(state_to_exec.at(i))
    //   {
    //     state_to_exec_count++;
    //   }
    // }

    // ---------------DUMPING TESTS-------------------------------------------------

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
      else if(state_to_exec.at(_hauler_dumping))
      {
        stateDump();
      }
      else
      {
        ROS_FATAL("No state to execute");
        flag_fallthrough_condition = false;
      }
    // }
    // else
    // {
    //   ROS_WARN("State fallthough, flag_fallthrough_condition = %i, state_to_exec_count = %i",flag_fallthrough_condition, state_to_exec_count);
    // }
    // -------------------------------------------------------------------------------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// State function definitions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmHauler::stateInitialize()
{
  ROS_WARN("Initialization State!\n");

  while (!clt_lights.waitForExistence())
  {
    ROS_WARN("HAULER: Waiting for Lights");
  }

  Lights(20);

  // while (!clt_approach_base.waitForExistence())
  // {
  //   ROS_WARN("HAULER: Waiting for ApproachChargingStation service");
  // }
  //
  // src2_approach_services::ApproachChargingStation srv_approach_charging_station;
  // srv_approach_charging_station.request.approach_charging_station.data = true;
  // bool approachSuccess = false;
  // int baseStationApproachRecoveryCount = 0;
  // while(!approachSuccess && baseStationApproachRecoveryCount<3){
  //   if (clt_approach_base.call(srv_approach_charging_station))
  //   {
  //     ROS_INFO("HAULER: Called service ApproachChargingStation");
  //     ROS_INFO_STREAM("Success finding the baseStation? "<< srv_approach_charging_station.response.success.data);
  //     if(srv_approach_charging_station.response.success.data){
  //       // homingRecovery(); //TODO: bin recovery behavior/fine align
  //       // }
  //       // else
  //
  //       approachSuccess=true;
  //       ROS_INFO("HAULER: approach base Station with classifier successful");
  //     }
  //   }
  //   else
  //   {
  //     ROS_ERROR("HAULER: Failed  to call service ApproachChargingStation");
  //   }
  //   baseStationApproachRecoveryCount=baseStationApproachRecoveryCount+1;
  // }




  Stop(2.0);

  Brake(100.0);

  while (!clt_sf_true_pose.waitForExistence())
  {
    ROS_ERROR("HAULER: Waiting for TruePose service");
  }

  GetTruePose();

  ClearCostmaps();

  Brake(0.0);

  double progress = 0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _initialize;
  sm_status_pub.publish(status_msg);
}

void SmHauler::statePlanning()
{
  ROS_INFO("Planning!\n");

  ROS_INFO("HAULER: Canceling MoveBase goal.");
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
      ROS_INFO("HAULER: Called service Waypoint Checker");
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
      ROS_ERROR("HAULER: Failed to call service Waypoint Checker");
    }
    ros::spinOnce();
    counter=counter+1;
  }

  ClearCostmaps();
  BrakeRamp(100, 2, 0);
  Brake(0.0);
if (!no_objective) {
  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("HAULER: Sending goal to MoveBase: " << move_base_goal);
  waypoint_timer = ros::Time::now();
  ac.sendGoal(move_base_goal, boost::bind(&SmHauler::doneCallback, this,_1,_2), boost::bind(&SmHauler::activeCallback, this), boost::bind(&SmHauler::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));
  flag_arrived_at_waypoint = false;
  flag_localizing_volatile = true;
}
else
{
  ROS_WARN ("HAULER: no_objective\n");
}
  double progress = 0;
  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _planning;
  sm_status_pub.publish(status_msg);
}

void SmHauler::stateTraverse()
{
  ROS_WARN("Traverse State\n");

  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
  {
    ROS_INFO("HAULER: Close to goal, getting new waypoint.");
    flag_arrived_at_waypoint = true;
  }

  ros::Duration timeOutWPCheck(3.0);
  if (ros::Time::now() - wp_checker_timer > timeOutWPCheck) {
    bool is_colliding = false;
    waypoint_checker::CheckCollision srv_wp_check;
    if (clt_waypoint_checker.call(srv_wp_check)) {
      ROS_INFO("HAULER: Called service Waypoint Checker");
      is_colliding = srv_wp_check.response.collision;
      if (is_colliding) {
        ROS_INFO("HAULER: Waypoint Unreachable. Sending to Planning");
        flag_waypoint_unreachable = true;
      }
    }
    wp_checker_timer = ros::Time::now();
  }

  move_base_state = ac.getState();
  int mb_state =(int) move_base_state.state_;
  ROS_WARN_STREAM("MoveBase status: "<< mb_state);

  if(mb_state==5 || mb_state==7)
  {
    ROS_ERROR("HAULER: MoveBase has failed to make itself useful.");
    flag_waypoint_unreachable= true;

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

    ROS_ERROR("Rover is stopped to clear the Map");
    ROS_WARN_STREAM("Move Base State: "<< mb_state);
    ROS_WARN("Map Cleared");
  }

  ros::Duration timeoutWaypoint(120);
  if (ros::Time::now() - waypoint_timer > timeoutWaypoint )
  {
    ROS_ERROR("Waypoint Unreachable");
    flag_waypoint_unreachable= true;
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
  ROS_WARN("Volatile Handling State!");

  while (!clt_lights.waitForExistence())
  {
      ROS_WARN("HAULER: Waiting for Lights");
  }
  Lights(20);

  // *******get true pose for dump testing
  while (!clt_sf_true_pose.waitForExistence())
  {
    ROS_ERROR("HAULER: Waiting for TruePose service");
  }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose.call(srv_sf_true_pose))
  {
    ROS_INFO("HAULER: Called service TruePose");
    ROS_INFO_STREAM("Status of SF True Pose: "<< srv_sf_true_pose.response.success);
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Pose Update");
  }

  RoverStatic(true);

  while (!clt_approach_excavator.waitForExistence())
  {
    ROS_WARN("HAULER: Waiting for ApproachExacavator service");
  }

  // approach bin with cv detector
  src2_approach_services::ApproachExcavator srv_approach_excavator;
  srv_approach_excavator.request.approach_excavator.data= true;
  bool approachSuccess = false;
  int ExcavatorApproachRecoveryCount = 0;
  while(!approachSuccess && ExcavatorApproachRecoveryCount<3){
    if (clt_approach_excavator.call(srv_approach_excavator))
    {
      ROS_INFO("HAULER: Called service ApproachExcavator");
      ROS_INFO_STREAM("Success finding the Excavator? "<< srv_approach_excavator.response.success.data);
      if(srv_approach_excavator.response.success.data){
        // homingRecovery(); //TODO: bin recovery behavior/fine align
        // }
        // else

        approachSuccess=true;
        ROS_INFO("HAULER: approach excavator with classifier successful");
        flag_dumping = true;
        flag_volatile_handler = false;
        // flag_arrived_at_waypoint = true;
      }
    }
    else
    {
      ROS_ERROR("HAULER: Failed  to call service ApproachBin");
    }
    ExcavatorApproachRecoveryCount= ExcavatorApproachRecoveryCount+1;
  }

  if (!approachSuccess){
    ROS_INFO("HAULER : failed after 3 attempts");
  }
}

void SmHauler::stateLost()
{
  ROS_ERROR("LOST STATE!\n");

  double progress = 1.0;

  ROS_INFO("SCOUT: Canceling MoveBase goal.");
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
        ROS_INFO("SCOUT: Called service ApproachChargingStation");
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
      ROS_ERROR("SCOUT: Failed  to call service ApproachChargingStation");
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
    ROS_INFO("Requesting Angle for LIDAR %f",srv_homing.request.angle);
    srv_homing.request.initializeLandmark = flag_need_init_landmark;
    if (clt_homing.call(srv_homing))
    {
      ROS_INFO("SCOUT: Called service Homing [Update]");
      if(srv_homing.request.initializeLandmark && srv_homing.response.success)
      {
          base_location_ = srv_homing.response.base_location;
          ROS_WARN("SCOUT: Saving Base Location %f %f",base_location_.x, base_location_.y);
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
      ROS_ERROR("SCOUT: Failed to call service Homing [Update]");
      progress = -1.0;
    }
  }
  else
  {
    progress = -1.0;
    ROS_ERROR(" Homing Attempt Failed, Just Moving On For Now");
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

  ClearCostmaps();
  BrakeRamp(100, 2, 0);
  Brake(0.0);

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _lost;
  sm_status_pub.publish(status_msg);
}

//------------------------------------------------------------------------------------------------------------------------

void SmHauler::stateDump()
{
  ROS_WARN("DUMPING STATE!\n");
  double progress = 1.0;

  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  // ToggleDetector(false);

  // while (!clt_lights.waitForExistence())
  // {
  //     ROS_WARN("HAULER: Waiting for Lights");
  // }
  // Lights(20);
  //
  // // *******get true pose for dump testing
  // while (!clt_sf_true_pose.waitForExistence())
  // {
  //   ROS_ERROR("HAULER: Waiting for TruePose service");
  // }
  //
  // // Update SF with True Pose
  // sensor_fusion::GetTruePose srv_sf_true_pose;
  // srv_sf_true_pose.request.start = true;
  // if (clt_sf_true_pose.call(srv_sf_true_pose))
  // {
  //   ROS_INFO("HAULER: Called service TruePose");
  //   ROS_INFO_STREAM("Status of SF True Pose: "<< srv_sf_true_pose.response.success);
  // }
  // else
  // {
  //   ROS_ERROR("HAULER: Failed  to call service Pose Update");
  // }
  //
  // RoverStatic(true);

  while (!clt_approach_bin.waitForExistence())
  {
    ROS_WARN("HAULER: Waiting for ApproachBin service");
  }

  // approach bin with cv detector
  src2_approach_services::ApproachBin srv_approach_bin;
  srv_approach_bin.request.approach_bin.data= true;
  bool approachSuccess = false;
  int binApproachRecoveryCount = 0;
  while(!approachSuccess && binApproachRecoveryCount<3){
    if (clt_approach_bin.call(srv_approach_bin))
    {
      ROS_INFO("HAULER: Called service ApproachBin");
      ROS_INFO_STREAM("Success finding the Bin? "<< srv_approach_bin.response.success.data);
      if(srv_approach_bin.response.success.data){
        // homingRecovery(); //TODO: bin recovery behavior/fine align
        // }
        // else

        approachSuccess=true;
        ROS_INFO("HAULER: approach bin with classifier successful");
      }
    }
    else
    {
      ROS_ERROR("HAULER: Failed  to call service ApproachBin");
    }
    binApproachRecoveryCount=binApproachRecoveryCount+1;
  }


  // localize bin after approaching bin
  if (approachSuccess){
      range_to_base::LocationOfBin srv_location_of_bin;
      srv_location_of_bin.request.location_of_bin.data=true;
      bool binLocationSuccess = true;
      progress = 1.0;

      // int binLocationRecoveryCount = 0;
      clt_location_of_bin.call(srv_location_of_bin);

      if(!srv_location_of_bin.response.success.data){
        ROS_INFO("HAULER: location of bin not reliable");
        progress = -1.0;

      }

      //ROS_INFO_STREAM("Hauler location: " << current_pose_);
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
  Brake(100.0);
  // std_msgs::Int64 state_msg;
  // state_msg.data = _hauler_dumping;
  // sm_status_pub.publish(state_msg);

  state_machine::RobotStatus status_msg;
  status_msg.progress.data = progress;
  status_msg.state.data = (uint8_t) _hauler_dumping;
  sm_status_pub.publish(status_msg);


  DriveCmdVel(-1.0,0.0,0.0,5);

// }
  // flag_arrived_at_waypoint = false;
  flag_volatile_handler = false;
  flag_dumping = false;

}

//------------------------------------------------------------------------------------------------------------------------



// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmHauler::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = msg->data;
  if (flag_localized_base) {
    ROS_WARN_ONCE("Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

// void SmHauler::mobilityCallback(const std_msgs::Int64::ConstPtr& msg)
// {
// flag_mobility = msg->data;
// if (flag_mobility == 0) {
//   ROS_ERROR("ROVER IMMOBILIZATION!  = %i", flag_mobility);
//   immobilityRecovery(1);
// } else {
//   ROS_WARN_ONCE("Rover is traversing = %i", flag_mobility);
// }
// }

void SmHauler::waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable = msg->data;
}

void SmHauler::arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint = msg->data;
}

void SmHauler::volatileDetectedCallback(const std_msgs::Float32::ConstPtr& msg)
{

  prev_volatile_detected_distance = volatile_detected_distance;
  volatile_detected_distance = msg->data;

}

void SmHauler::volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg)
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

void SmHauler::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
}
void SmHauler::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg){
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
  ROS_INFO_STREAM_THROTTLE(2,"HAULER: Minimum range average: " << min_range);

  if (min_range < LASER_THRESH)
  {
    if (ros::Time::now() - last_time_laser_collision < ros::Duration(20))
    {
      ROS_WARN("HAULER: Close to wall.");
      counter_laser_collision_++;
      ROS_INFO_STREAM("HAULER: Counter laser:" << counter_laser_collision_);
    }
    else
    {
      counter_laser_collision_ = 0;
      ROS_INFO_STREAM("HAULER: Counter laser RESET!");
    }
    last_time_laser_collision = ros::Time::now();
  }

  if (counter_laser_collision_ > LASER_COUNTER_THRESH)
  {
    counter_laser_collision_ =0;
    ROS_ERROR("HAULER: LASER COUNTER > 20 ! Starting Recovery.");
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





//********************************************************************************************************
    poseGoal.target_pose.header.frame_id = "small_hauler_1_odom";
    poseGoal.target_pose.pose.position.x = x;
    poseGoal.target_pose.pose.position.y = y;
    poseGoal.target_pose.pose.position.z = 0.0;
    poseGoal.target_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    poseGoal.target_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    poseGoal.target_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    poseGoal.target_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;
}

void SmHauler::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone = true;
    // ROS_INFO("Goal done");
}
void SmHauler::activeCallback()
{
    // ROS_INFO("Goal went active");
}
void SmHauler::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  //  ROS_INFO("Got feedback");
}

void SmHauler::plannerInterruptCallback(const std_msgs::Bool::ConstPtr &msg)
{
    flag_interrupt_plan = msg->data;
  // ROS_INFO_STREAM("HAULER: Interrupt flag updated." << *msg);
}

void SmHauler::RotateToHeading(double desired_yaw)
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

void SmHauler::homingRecovery()
{

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Homing Recovery.");

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

void SmHauler::ClearCostmaps()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("HAULER: Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR("HAULER: Failed calling clear_costmaps service.");
  }
}

void SmHauler::GetTruePose()
{
  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;
  if (clt_sf_true_pose.call(srv_sf_true_pose))
  {
    ROS_INFO("HAULER: Called service TruePose");
    ROS_INFO_STREAM("Status of SF True Pose: "<< srv_sf_true_pose.response.success);
    flag_have_true_pose = true;
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Pose Update");
  }
}

void SmHauler::Lights(double intensity)
{
  // Turn on the Lights
  srcp2_msgs::SpotLightSrv srv_lights;
  srv_lights.request.range  = intensity;
  if (clt_lights.call(srv_lights))
  {
    ROS_INFO("HAULER: Called service SpotLight");
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service SpotLight");
  }
}

void SmHauler::RotateInPlace(double speed_ratio, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.speed_ratio  = speed_ratio;
  if (clt_rip.call(srv_turn))
  {
    ROS_INFO_THROTTLE(5,"HAULER: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("HAULER: Failed to call service RotateInPlace");
  }
}

void SmHauler::Stop(double time)
{
  driving_tools::Stop srv_stop;
  srv_stop.request.enable  = true;
  if (clt_stop.call(srv_stop))
  {
    ROS_INFO("HAULER: Called service Stop");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("HAULER: Failed  to call service Stop");
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
    //ROS_INFO_STREAM("HAULER: Called service SRCP2 Brake. Engaged?: " << flag_brake_engaged);
  }
  else
  {
      ROS_ERROR("HAULER: Failed  to call service Brake");
  }
}

void SmHauler::BrakeRamp(double max_intensity, double time, int aggressivity)
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
  ROS_INFO_STREAM("HAULER: Called service SRCP2 Brake. Engaged? " << flag_brake_engaged);
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
    ROS_INFO("HAULER: Called service Drive");
    while(ros::Time::now() - start_time < ros::Duration(time))
    {
      std_msgs::Int64 mode;
      mode.data = 2;
      driving_mode_pub.publish(mode);
    }
  }
  else
  {
    ROS_ERROR("HAULER: Failed to call service Drive");
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
  ROS_ERROR("Drive Cmd Vel publisher.");
  while (ros::Time::now() - start_time < timeout)
  {
    cmd_vel_pub.publish(cmd_vel);
  }
}

// void SmHauler::ToggleDetector(bool flag)
// {
//   volatile_handler::ToggleDetector srv_vol_detect;
//   srv_vol_detect.request.on  = flag;
//   if (clt_vol_detect_.call(srv_vol_detect))
//   {
//     ROS_INFO_STREAM("HAULER: Called service ToggleDetector. Turned on? " << flag);
//   }
//   else
//   {
//     ROS_ERROR("HAULER: Failed  to call service ToggleDetector");
//   }
// }

void SmHauler::RoverStatic(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static.call(srv_rover_static))
  {
    ROS_INFO_STREAM("HAULER: Called service RoverStatic. Turned on? " << flag);
  }
  else
  {
    ROS_ERROR("HAULER: Failed to call service RoverStatic");
  }

}

bool SmHauler::setMobility(state_machine::SetMobility::Request &req, state_machine::SetMobility::Response &res){
  ROS_ERROR(" GOT MOBILITY IN SM %d", req.mobility);
  flag_mobility = req.mobility;
  immobilityRecovery(1);
  //ros::Duration(2).sleep();
  res.success = true;
  return true;
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
    ROS_INFO_THROTTLE(5,"HAULER: Called service Plan");
  }
  else
  {
    ROS_INFO("HAULER: Failed to call service RotateInPlace");
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

  // switch (srv_plan.response.code.data)
  // {
  // case _initialize:
  //   flag_have_true_pose = false;
  //   break;

  // case _planning:
  //   flag_interrupt_plan = true;
  //   flag_arrived_at_waypoint = true;
  //   flag_recovering_localization = false;
  //   flag_localizing_volatile = false;
  //   flag_dumping = false;
  //   break;

  // case _traverse:
  //   flag_interrupt_plan = false;
  //   flag_arrived_at_waypoint = false;
  //   flag_recovering_localization = false;
  //   flag_localizing_volatile = false;
  //   flag_dumping = false;
  //   break;

  // case _volatile_handler:
  //   flag_interrupt_plan = false;
  //   flag_arrived_at_waypoint = false;
  //   flag_recovering_localization = false;
  //   flag_localizing_volatile = true;
  //   flag_dumping = false;
  //   break;

  // case _lost:
  //   flag_interrupt_plan = false;
  //   flag_arrived_at_waypoint = false;
  //   flag_recovering_localization = true;
  //   flag_localizing_volatile = false;
  //   flag_dumping = false;
  //   break;

  // case _hauler_dumping:
  //     flag_interrupt_plan = false;
  //     flag_arrived_at_waypoint = false;
  //     flag_recovering_localization = false;
  //     flag_localizing_volatile = false;
  //     flag_dumping = true;
  //     break;

  // default:
  //   flag_interrupt_plan = true;
  //   flag_arrived_at_waypoint = true;
  //   flag_recovering_localization = false;
  //   flag_localizing_volatile = false;
  //   flag_dumping = false;
  //   break;
  // }

  // srv_plan.response.id;
  flag_interrupt_plan = false;
}

//------------------------------------------------------------------------------------------------------------------------
