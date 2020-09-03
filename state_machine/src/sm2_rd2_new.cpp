#include <state_machine/sm_rd2.hpp>

SmRd2::SmRd2() :
  ac_excavator_("/excavator_1/move_base", true),
  ac_excavator_("/hauler_1/move_base", true),
  move_base_state_excavator_(actionlib::SimpleClientGoalState::LOST),
  move_base_state_hauler_(actionlib::SimpleClientGoalState::LOST)
{
  // Initialize ROS, Subs, and Pubs *******************************************

  std::string excavator_1 = "excavator_1";
  std::string hauler_1 = "hauler_1";


  // Publishers
  sm_state_hauler_pub = nh.advertise<std_msgs::Int64>("/hauler_1/state_machine/state", 10);
  sm_state_excavator_pub = nh.advertise<std_msgs::Int64>("/excavator_1/state_machine/state", 10);
  manipulation_state_excavator_pub = nh.advertise<std_msgs::Int64>("/excavator_1/manipulation/state", 10);
  manipulation_volatile_pose_pub = nh.advertise<geometry_msgs::Pose>("/excavator_1/manipulation/volatile_pose", 10);
  
  // Subscribers
  odometry_excavator_sub = nh.subscribe("/excavator_1/localization/odometry/sensor_fusion", 1, &SmRd2::odometryExcavatorCallback, this);
  localized_base_excavator_sub = nh.subscribe("/excavator_1/state_machine/localized_base_excavator", 1, &SmRd2::localizedBaseExcavatorCallback, this);
  waypoint_unreachable_excavator_sub = nh.subscribe("/excavator_1/state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableExcavatorCallback, this);
  arrived_at_waypoint_excavator_sub = nh.subscribe("/excavator_1/state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointExcavatorCallback, this);
  localization_failure_excavator_sub = nh.subscribe("/excavator_1/state_machine/localization_failure_excavator", 1, &SmRd2::localizationFailureExcavatorCallback, this);
  manipulation_feedback_excavator_sub = nh.subscribe("/excavator_1/manipulation/feedback", 1, &SmRd2::manipulationFeedbackCallback, this);
  driving_mode_sub =nh.subscribe("driving/driving_mode",1, &SmRd1::drivingModeCallback, this);

  // Subscribers
  odometry_hauler_sub = nh.subscribe("/hauler_1/localization/odometry/sensor_fusion", 1, &SmRd2::odometryHaulerCallback, this);
  localized_base_hauler_sub = nh.subscribe("/hauler_1/state_machine/localized_base_hauler", 1, &SmRd2::localizedBaseHaulerCallback, this);
  waypoint_unreachable_hauler_sub = nh.subscribe("/hauler_1/state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableHaulerCallback, this);
  arrived_at_waypoint_hauler_sub = nh.subscribe("/hauler_1/state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointHaulerCallback, this);
  localization_failure_hauler_sub = nh.subscribe("/hauler_1/state_machine/localization_failure_hauler", 1, &SmRd2::localizationFailureHaulerCallback, this);
  driving_mode_hauler_sub =nh.subscribe("/hauler_1/driving/driving_mode",1, &SmRd1::drivingModeHaulerCallback, this);

  // Clients
  clt_next_vol_excavator_ = nh.serviceClient<round2_volatile_handler::NextVolatileLocation>("/excavator_1/volatile/next");
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
  clt_srcp2_brake_rover_excavator_= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/excavator_1/brake_rover");

  // Clients
  clt_wp_nav_set_goal_hauler_ = nh.serviceClient<waypoint_nav::SetGoal>("/hauler_1/navigation/set_goal");
  clt_wp_nav_interrupt_hauler_ = nh.serviceClient<waypoint_nav::Interrupt>("/hauler_1/navigation/interrupt");
  clt_stop_hauler_ = nh.serviceClient<driving_tools::Stop>("/hauler_1/driving/stop");
  clt_rip_hauler_ = nh.serviceClient<driving_tools::RotateInPlace>("/hauler_1/driving/rotate_in_place");
  clt_drive_hauler_ = nh.serviceClient<driving_tools::MoveForward>("/hauler_1/driving/move_forward");
  clt_lights_hauler_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("/hauler_1/toggle_light");
  clt_brake_hauler_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/hauler_1/brake_rover");
  clt_approach_base_hauler_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("/hauler_1/approach_base_station");
  clt_rover_static_hauler_ = nh.serviceClient<sensor_fusion::RoverStatic>("/hauler_1/sensor_fusion/toggle_rover_static");
  clt_homing_hauler_ = nh.serviceClient<sensor_fusion::HomingUpdate>("/hauler_1/homing");
  clt_sf_true_pose_hauler_ = nh.serviceClient<sensor_fusion::GetTruePose>("/hauler_1/true_pose");
  clt_waypoint_checker_hauler_ = nh.serviceClient<waypoint_checker::CheckCollision>("/hauler_1/waypoint_checker");
  clt_srcp2_brake_rover_hauler_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("/hauler_1/brake_rover");

  // Init variables
  driving_mode_=0;

  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();

 }

void SmRd2::run()
{
  ros::Rate loop_rate(2); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // EXCAVATOR
    ROS_INFO("flag_localized_base_excavator: %i",flag_localized_base_excavator);
    ROS_INFO("flag_have_true_pose_excavator: %i",flag_have_true_pose_excavator);
    // ROS_INFO("flag_waypoint_unreachable: %i",flag_waypoint_unreachable);
    ROS_INFO("flag_arrived_at_waypoint_excavator: %i",flag_arrived_at_waypoint_excavator);
    ROS_INFO("volatile_detected_distance_excavator: %f",volatile_detected_distance_excavator);
    // ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    ROS_INFO("flag_volatile_recorded_excavator: %i",flag_volatile_recorded_excavator);
    // ROS_INFO("flag_volatile_unreachable: %i",flag_volatile_unreachable);
    // ROS_INFO("flag_localization_failure: %i",flag_localization_failure);
    ROS_INFO("flag_brake_engaged_excavator: %i",flag_brake_engaged_excavator);
    ROS_INFO("flag_fallthrough_condition_excavator: %i",flag_fallthrough_condition_excavator);
    
    // HAULER
    ROS_INFO("flag_localized_base_hauler: %i",flag_localized_base_hauler);
    ROS_INFO("flag_have_true_pose_hauler: %i",flag_have_true_pose_hauler);
    // ROS_INFO("flag_waypoint_unreachable: %i",flag_waypoint_unreachable);
    ROS_INFO("flag_arrived_at_waypoint_hauler: %i",flag_arrived_at_waypoint_hauler);
    // ROS_INFO("volatile_detected_distance_hauler: %f",volatile_detected_distance_hauler);
    // ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    // ROS_INFO("flag_volatile_recorded_hauler: %i",flag_volatile_recorded_hauler);
    // ROS_INFO("flag_volatile_unreachable: %i",flag_volatile_unreachable);
    // ROS_INFO("flag_localization_failure: %i",flag_localization_failure);
    ROS_INFO("flag_brake_engaged_hauler: %i",flag_brake_engaged_hauler);
    ROS_INFO("flag_fallthrough_condition_hauler: %i",flag_fallthrough_condition_hauler);
    //---------------------------------------------------------------------------------------------------------------------


    // Conditional flag logic (Preemptive conditions) +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if((volatile_detected_distance_excavator>0.0) && !flag_localizing_volatile_excavator && !flag_localization_failure_excavator && flag_have_true_pose_excavator)
    {
      flag_arrived_at_waypoint_excavator = true;
      flag_waypoint_unreachable_excavator = false;
    }
    if(flag_localization_failure_excavator && !flag_recovering_localization_excavator)
    {
      flag_arrived_at_waypoint_excavator = true;
      flag_waypoint_unreachable_excavator = false;
    }
    //---------------------------------------------------------------------------------------------------------------------


    // State machine truth table ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    state_to_exec.clear();
    state_to_exec.resize(num_states,0);
    if(!flag_have_true_pose_excavator && (flag_arrived_at_waypoint_excavator || flag_waypoint_unreachable_excavator))
    {
      state_to_exec.at(_initialize) = 1;
    }
    else if((flag_arrived_at_waypoint_excavator || flag_waypoint_unreachable_excavator) && (flag_localization_failure_excavator || flag_recovering_localization_excavator))
    {
      state_to_exec.at(_lost) = 1;
    }
    else if(flag_arrived_at_waypoint_excavator && flag_volatile_dug_excavator && !flag_brake_engaged_excavator)
    {
      state_to_exec.at(_planning) = 1;
    }
    else if((!flag_arrived_at_waypoint_excavator && !flag_waypoint_unreachable_excavator) && !flag_brake_engaged_excavator)
    {
      state_to_exec.at(_traverse) = 1;
    }
    else if((!flag_volatile_dug_excavator) && !flag_brake_engaged_excavator)
    {
      state_to_exec.at(_volatile_handler) = 1;
    }
    else
    {
      flag_arrived_at_waypoint_excavator = true;
      flag_waypoint_unreachable_excavator = false;
      flag_localizing_volatile_excavator = false;
      flag_fallthrough_condition_excavator = true;
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

    if(!flag_fallthrough_condition_excavator || state_to_exec_count == 1)
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
      ROS_WARN("State fallthough, flag_fallthrough_condition = %i, state_to_exec_count = %i",flag_fallthrough_condition_excavator, state_to_exec_count);
    }
    // -------------------------------------------------------------------------------------------------------------------

    ros::spinOnce();
    loop_rate.sleep();
  }
}

// State function definitions ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void SmRd2::stateInitialize()
{
  ROS_INFO("Initialize!\n");
  flag_arrived_at_waypoint_excavator = false;
  flag_waypoint_unreachable_excavator = false;

  flag_arrived_at_waypoint_hauler = false;
  flag_waypoint_unreachable_hauler = false;

  approachBaseStationHauler();
  approachBaseStationExcavator();

  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_excavator_pub.publish(state_msg);
}

void SmRd2::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint_excavator = false;
  flag_waypoint_unreachable_excavator = false;

  flag_arrived_at_waypoint_hauler = false;
  flag_waypoint_unreachable_hauler = false;


  std::string vol_type;
  geometry_msgs::Pose goal_pose;
  ROS_INFO_STREAM("EXCAVATOR: Goal pose: " << goal_pose);
  // Generate Goal
  round2_volatile_handler::NextVolatileLocation srv_next_vol;
  srv_next_vol.request.rover_position  = current_pose_excavator_;
  ROS_INFO_STREAM("EXCAVATOR: Rover Pose: "<< current_pose_excavator_);
  if (clt_next_vol_excavator_.call(srv_next_vol))
  {
    ROS_INFO_STREAM("EXCAVATOR: Next Volatile Pose: "<< srv_next_vol.response.vol_position);
    goal_pose_ = srv_next_vol.response.vol_position;
    vol_type = srv_next_vol.response.type;
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service Next Volatile Pose");
  }

  // Publish volatile pose
  manipulation_volatile_pose_pub.publish(goal_pose);

  std_msgs::Int64 state_msg;
  state_msg.data = _planning;
  sm_state_excavator_pub.publish(state_msg);
}

void SmRd2::stateTraverse()
{
  ROS_INFO("Traverse!\n");

  if(flag_localized_base_excavator && !flag_have_true_pose_excavator)
  {
    flag_have_true_pose_excavator = true;
    flag_arrived_at_waypoint_excavator = true;
    flag_waypoint_unreachable_excavator = false;
  }

  if(flag_localized_base_hauler && !flag_have_true_pose_hauler)
  {
    flag_have_true_pose_hauler = true;
    flag_arrived_at_waypoint_hauler = true;
    flag_waypoint_unreachable_hauler = false;
  }
  // if(flag_volatile_recorded_excavator)
  // {
  //   volatile_detected_distance_excavator = -1.0;
  //   flag_localizing_volatile_excavator = false;
  //   flag_volatile_recorded_excavator = false;
  //   flag_arrived_at_waypoint_excavator = true;
  //   flag_waypoint_unreachable_excavator = false;
  // }
  if(flag_recovering_localization_excavator && !flag_localization_failure_excavator)
  {
    flag_arrived_at_waypoint_excavator = true;
    flag_waypoint_unreachable_excavator = false;
    flag_recovering_localization_excavator = false;
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_excavator_pub.publish(state_msg);

}

void SmRd2::stateVolatileHandler()
{
 /* if(volatile_detected_distance>1.0){
	  ROS_INFO(" Not Calling Volatile Report Service Because Not Close Enough %f",volatile_detected_distance);
	  return;

	  // JNG Question, should some sort of status be published here?
   }*/
  // ros::Duration(2.0).sleep();
  // // Get True Pose
  // waypoint_nav::Interrupt srv_wp_nav;
  // srv_wp_nav.request.interrupt = true;
  // if (clt_wp_nav_interrupt_excavator_.call(srv_wp_nav))
  // {
  //   ROS_INFO_STREAM("I called service interrupt ");
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service Interrupt Nav");
  // }
  // ros::Duration(2.0).sleep();
  // Turn on brake
   driving_tools::Stop srv_stop;
   srv_stop.request.enableStop  = true;
   if (clt_stop_excavator_.call(srv_stop))
   {
     ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
   }
   else
   {
     ROS_ERROR("Failed to call service Stop");
   }

   std_msgs::Int64 manipulation_state_msg;
   manipulation_state_msg.data  = 9;
   manipulation_state_excavator_pub.publish(manipulation_state_msg);

   ros::Duration(10.0).sleep();
   // Set Goal
   waypoint_nav::SetGoal srv_wp_nav;
   srv_wp_nav.request.start = true;
   srv_wp_nav.request.goal = goal_pose_;
   if (clt_wp_nav_set_goal_hauler_.call(srv_wp_nav))
   {
     //s ROS_INFO_STREAM("Success? "<< srv_wp_nav.response.success);
     if (!srv_wp_nav.response.success)
     {
        if (!search_candidates.size())
        {

          std::vector<double> temp_P({P_[0], P_[1], P_[6], P_[7]});
          std::pair<double, double> origin({goal_pose_.position.x, goal_pose_.position.y});

          search_candidates = sm_utils::circlePacking(origin, temp_P, 2, 0.5);
          goal_pose_.position.x = search_candidates[0].first;
          goal_pose_.position.y = search_candidates[1].second;

        } else
        {
          goal_pose_.position.x = search_candidates[0].first;
          goal_pose_.position.y = search_candidates[1].second;
        }
        search_candidates.erase(search_candidates.begin());

        waypoint_nav::SetGoal srv_wp_nav;
        srv_wp_nav.request.start = true;
        srv_wp_nav.request.goal = goal_pose_;
        if (clt_wp_nav_set_goal_excavator_.call(srv_wp_nav))
        {
          //s ROS_INFO_STREAM("Success? "<< srv_wp_nav.response.success);
        }
        else
        {
          ROS_ERROR("EXCAVATOR Failed to call service Drive to Waypoint");
        }

        // Publish volatile pose
        manipulation_volatile_pose_pub.publish(goal_pose_);

        //if last item in list and item can't be found set as unreachable
      }

   }
   else
   {
     ROS_ERROR("EXCAVATOR Failed to call service Drive to Waypoint");
   }



   ROS_INFO("VolatileHandler!\n");
   //flag_arrived_at_waypoint = false;
   flag_waypoint_unreachable_excavator = false;
   //flag_localizing_volatile = true;
   // flag_arrived_at_waypoint = true; // This is temporary for hackathon, since volatile reporting doesn't have all info and doesn't command a waypoint yet.
   std_msgs::Int64 state_msg;
   // state_msg.data = _volatile_handler;
   state_msg.data = false;
   sm_state_excavator_pub.publish(state_msg);
}

void SmRd2::stateLost()
{
  ROS_INFO("Lost!\n");
  flag_recovering_localization_excavator = true;
  flag_localizing_volatile_excavator = false;
  flag_arrived_at_waypoint_excavator = false;
  flag_waypoint_unreachable_excavator = false;

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _lost;
  sm_state_excavator_pub.publish(state_msg);
}
//------------------------------------------------------------------------------------------------------------------------


// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// void SmRd2::localizedBaseCallback(const std_msgs::Bool::ConstPtr& msg)

void SmRd2::odometryExcavatorCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (first_odom_excavator_ == false)
    {
        first_odom_excavator_ = true;
    }
    current_pose_excavator_ = msg->pose.pose;
    current_vel_excavator_ = msg->twist.twist;
}
void SmRd2::odometryHaulerCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (first_odom_hauler_ == false)
    {
        first_odom_hauler_ = true;
    }
    current_pose_hauler_ = msg->pose.pose;
    current_vel_hauler_ = msg->twist.twist;
}


void SmRd2::localizedBaseExcavatorCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base_excavator = msg->data;
  if (flag_localized_base_excavator) {
    ROS_WARN_ONCE("EXCAVATOR Initial Localization Successful = %i",flag_localized_base_excavator);

  }
  else {
    ROS_INFO("EXCAVATOR Waiting for Initial Localization  = %i",flag_localized_base_excavator);
  }
}

void SmRd2::manipulationFeedbackCallback(const move_excavator::ExcavationStatus::ConstPtr& msg)
{
    isFinished_ = msg->isFinished;
    collectedMass_ = msg->collectedMass;
    if(isFinished_)
    {
      ROS_INFO_STREAM("Let's finish manipulation.");
      flag_volatile_dug_excavator = true;
    }
}

void SmRd2::waypointUnreachableExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable_excavator = msg->data;
}

void SmRd2::arrivedAtWaypointExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint_excavator = msg->data;

  if (flag_arrived_at_waypoint_excavator)
  {
    ROS_INFO_STREAM("Let's start manipulation.");
    flag_volatile_dug_excavator = false;
  }
}

// void SmRd2::volatileDetectedExcavatorCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//   volatile_detected_distance_excavator = msg->data;
// }
//
// void SmRd2::volatileRecordedExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//   flag_volatile_recorded_excavator = msg->data;
//   if (flag_volatile_recorded_excavator == true)
//   {
//    volatile_detected_distance_excavator = -1.0;
//   }
// }

void SmRd2::localizationFailureExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure_excavator = msg->data;
}


// ------------------*******************---------------

void SmRd2::localizedBaseHaulerCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base_hauler = msg->data;
  if (flag_localized_base_hauler) {
    ROS_WARN_ONCE("HAULER Initial Localization Successful = %i",flag_localized_base_hauler);

  }
  else {
    ROS_INFO("HAULER Waiting for Initial Localization  = %i",flag_localized_base_hauler);
  }
}
void SmRd2::waypointUnreachableHaulerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable_hauler = msg->data;
}

void SmRd2::arrivedAtWaypointHaulerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint_hauler = msg->data;
}

// void SmRd2::volatileDetectedHaulerCallback(const std_msgs::Float32::ConstPtr& msg)
// {
//   volatile_detected_distance_hauler = msg->data;
// }

// void SmRd2::volatileRecordedHaulerCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//   flag_volatile_recorded_hauler = msg->data;
//   if (flag_volatile_recorded_hauler == true)
//   {
//    volatile_detected_distance_hauler = -1.0;
//   }
// }

void SmRd2::localizationFailureHaulerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure_hauler = msg->data;
}

void SmRd2::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev = yaw;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  z_ = msg->pose.pose.position.x;
  P_.clear();
  for (int i = 0; i < 36; ++i)
  {
    P_.push_back(msg->pose.covariance[i]);
  }
}

void SmRd1::doneExcavatorCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_ = true;
    ROS_INFO("EXCAVATOR: Goal done");
}
void SmRd1::activeExcavatorCallback()
{
    ROS_INFO("EXCAVATOR: Goal went active");
}
void SmRd1::feedbackExcavatorCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    ROS_INFO("EXCAVATOR: Got feedback");
}

void SmRd1::doneHaulerCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_ = true;
    ROS_INFO("HAULER: Goal done");
}
void SmRd1::activeHaulerCallback()
{
    ROS_INFO("HAULER: Goal went active");
}
void SmRd1::feedbackHaulerCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    ROS_INFO("HAULER: Got feedback");
}

void SmRd2::approachBaseStationExcavator()
{
  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = "0.8";
  if (clt_lights_excavator_.call(srv_lights))
  {
    // ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service ToggleLight");
  }

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  while (!clt_approach_base_excavator_.waitForExistence())
  {
    ROS_ERROR("EXCAVATOR: waiting for approach_base");
  }
  if (clt_approach_base_excavator_.call(srv_approach_base))
  {
    // ROS_INFO_STREAM("Success? "<< srv_approach_base.response.success.data);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service ApproachBaseStation");
  }

  // Break Rover
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    // ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service Stop");
  }

  srcp2_msgs::BrakeRoverSrv srv_brake;

  srv_brake.request.brake_force  = 100.0;
  if (clt_srcp2_brake_rover_excavator_.call(srv_brake))
  {
     ROS_INFO_STREAM("EXCAVATOR: Brake Enabled? "<< srv_brake.response.finished);

  }
  else
  {
      ROS_ERROR("EXCAVATOR: Failed to call service Brake");
  }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;

  while (!clt_sf_true_pose_excavator_.waitForExistence())
  {
    ROS_ERROR("EXCAVATOR: Waiting for true pose");
  }
  if (clt_sf_true_pose_excavator_.call(srv_sf_true_pose))
  {
    ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_sf_true_pose.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service Pose Update");
  }

  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = true;
  if (clt_rover_static_excavator_.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_rover_static.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service RoverStatic");
  }

  // Homing - Initialize Base Station Landmark
  sensor_fusion::HomingUpdate srv_homing;
  srv_homing.request.initializeLandmark = true;
  if (clt_homing_excavator_.call(srv_homing))
  {
    base_location_ = srv_homing.response.base_location;
    // ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_homing.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service HomingUpdate");
  }

  // Stop attitude constraints for static rover
  srv_rover_static.request.rover_static  = false;
  if (clt_rover_static_excavator_.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_rover_static.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service RoverStatic");
  }

  // Turn on the Lights
  srv_lights.request.data  = "0.2";
  if (clt_lights_excavator_.call(srv_lights))
  {
    // ROS_INFO_STREAM("EXCAVATOR: Success? "<< srv_lights.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service ToggleLight");
  }

  srv_brake.request.brake_force  = 0.0;
  if (clt_srcp2_brake_rover_excavator_.call(srv_brake))
  {
     ROS_INFO_STREAM("EXCAVATOR: Brake Enabled? "<< srv_brake.response.finished);

  }
  else
  {
      ROS_ERROR("EXCAVATOR: Failed to call service Brake");
  }
  // Move Backward first
  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle  = -0.3;
  if (clt_drive_excavator_.call(srv_drive))
  {
          ros::Duration(2.0).sleep();
          ROS_INFO_STREAM("EXCAVATOR: Backward Drive Enabled? "<< srv_drive.response);
  }
  else
  {
          ROS_ERROR("EXCAVATOR: Failed to call service Drive");
  }

  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    ros::Duration(2.0).sleep();
  }
  else
  {
    ROS_ERROR("EXCAVATOR: Failed to call service Stop");
  }

 // Then Rotate in Place

  driving_tools::RotateInPlace srv_turn;

  srv_turn.request.throttle  = 0.2;
   ros::Duration(1.0).sleep();
  if (clt_rip_excavator_.call(srv_turn))
  {
          ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
          ros::Duration(5.0).sleep();
  }
  else
  {
          ROS_ERROR("Failed to call service Stop");
  }

  // driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    ros::Duration(2.0).sleep();// ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  // Clear the costmap
  clearCostmapsExcavator();
}

void SmRd2::approachBaseStationHauler()
{

  // Turn on the Lights
  srcp2_msgs::ToggleLightSrv srv_lights;
  srv_lights.request.data  = "0.8";
  if (clt_lights_.call(srv_lights))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service ToggleLight");
  }

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  while (!clt_approach_base_hauler_.waitForExistence())
  {
    ROS_ERROR("WAITING for approach_base");
  }
  if (clt_approach_base_.call(srv_approach_base))
  {
    // ROS_INFO_STREAM("Success? "<< srv_approach_base.response.success.data);


  }
  else
  {
    ROS_ERROR("Failed to call service ApproachBaseStation");
  }

  // Break Rover
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_hauler_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  srcp2_msgs::BrakeRoverSrv srv_brake;

  srv_brake.request.brake_force  = 100.0;
  if (clt_srcp2_brake_rover_hauler_.call(srv_brake))
  {
     ROS_INFO_STREAM("SM: Brake Enabled? "<< srv_brake.response.finished);

  }
  else
  {
      ROS_ERROR("Failed to call service Brake");
  }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;

  while (!clt_sf_true_pose_hauler_.waitForExistence())
  {
    ROS_ERROR("WAITING FOR TRUE POSE");
  }
  if (clt_sf_true_pose_hauler_.call(srv_sf_true_pose))
  {
    ROS_INFO_STREAM("Success STATUS OF srv_sf_true_pose? "<< srv_sf_true_pose.response.success);
  }
  else
  {
    ROS_INFO_STREAM("STATUS OF srv_sf_true_pose" << srv_sf_true_pose.response.success);
    ROS_ERROR("Failed to call service Pose Update");
  }

  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = true;
  if (clt_rover_static_hauler_.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("Success? "<< srv_rover_static.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service RoverStatic");
  }

  // Stop attitude constraints for static rover
  srv_rover_static.request.rover_static  = false;
  if (clt_rover_static_hauler_.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("Success? "<< srv_rover_static.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service RoverStatic");
  }

  // Turn on the Lights
  srv_lights.request.data  = "0.2";
  if (clt_lights_hauler_.call(srv_lights))
  {
    // ROS_INFO_STREAM("Success? "<< srv_lights.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service ToggleLight");
  }

  srv_brake.request.brake_force  = 0.0;
  if (clt_srcp2_brake_rover_hauler_.call(srv_brake))
  {
     ROS_INFO_STREAM("SM: Brake Enabled? "<< srv_brake.response.finished);

  }
  else
  {
      ROS_ERROR("Failed to call service Brake");
  }
  // Move Backward first
  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle  = -0.3;
  if (clt_drive_hauler_.call(srv_drive))
  {
          ros::Duration(2.0).sleep();
          ROS_INFO_STREAM("SM: Backward Drive Enabled? "<< srv_drive.response);
  }
  else
  {
          ROS_ERROR("Failed to call service Drive");
  }

  srv_stop.request.enableStop  = true;
  if (clt_stop_hauler_.call(srv_stop))
  {
    ros::Duration(2.0).sleep();
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

 // Then Rotate in Place

  driving_tools::RotateInPlace srv_turn;

  srv_turn.request.throttle  = 0.2;
   ros::Duration(1.0).sleep();
  if (clt_rip_hauler_.call(srv_turn))
  {
          ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
          ros::Duration(2.5).sleep();
  }
  else
  {
          ROS_ERROR("Failed to call service Stop");
  }

  srv_drive.request.throttle  = 0.3;
  if (clt_drive_hauler_.call(srv_drive))
  {
          ros::Duration(4.0).sleep();
          ROS_INFO_STREAM("SM: Backward Drive Enabled? "<< srv_drive.response);
  }
  else
  {
          ROS_ERROR("Failed to call service Drive");
  }

  srv_turn.request.throttle  = 0.2;
   ros::Duration(1.0).sleep();
  if (clt_rip_hauler_.call(srv_turn))
  {
          ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
          ros::Duration(2.5).sleep();
  }
  else
  {
          ROS_ERROR("Failed to call service Stop");
  }

  // driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_hauler_.call(srv_stop))
  {
    ros::Duration(2.0).sleep();// ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  // Clear the costmap
  clearCostmapsHauler();
}

//------------------------------------------------------------------------------------------------------------------------

void SmRd2::clearCostmapsExcavator()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/excavator_1/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/excavator_1/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("EXCAVATOR: costmap layers are cleared");
  }
  else
  {
     ROS_ERROR("EXCAVATOR: failed calling clear_costmaps service");
  }
}

void SmRd2::clearCostmapsHauler()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/hauler_1/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/hauler_1/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("HAULER: costmap layers are cleared");
  }
  else
  {
     ROS_ERROR("HAULER: failed calling clear_costmaps service");
  }
}


void SmRd2::prepareForWaypointExcavator()
{
  ac_excavator_.waitForServer();
  ac_excavator_.cancelGoal();
  ac_excavator_.waitForResult(ros::Duration(2.0));

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  srcp2_msgs::BrakeRoverSrv srv_brake;
  srv_brake.request.brake_force  = 100.0;
  if (clt_srcp2_brake_rover_excavator_.call(srv_brake))
  {
     ROS_INFO_STREAM("SM: Brake Enabled? "<< srv_brake.response.finished);

  }
  else
  {
      ROS_ERROR("Failed to call service Brake");
  }

  // Generate Goal
  waypoint_gen::GenerateWaypoint srv_wp_gen;
  srv_wp_gen.request.start  = true;
  if(flag_completed_homing_excavator ){
    flag_completed_homing_excavator = false;
    srv_wp_gen.request.next  = true;
  }
  else if(flag_waypoint_unreachable_excavator)
  {
    flag_waypoint_unreachable_excavator =false;
    srv_wp_gen.request.next  = true;
  }
  else{
  srv_wp_gen.request.next  = false;
  }
  if (clt_wp_gen_excavator_.call(srv_wp_gen))
  {
    // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);

    goal_pose_excavator_ = srv_wp_gen.response.goal;
    waypoint_type_excavator_ = srv_wp_gen.response.type;
  }
  else
  {
    ROS_ERROR("Failed to call service Generate Waypoint");
  }
  ROS_INFO_STREAM("goal pose: " << goal_pose_excavator_);

  // check waypoint

  waypoint_checker::CheckCollision srv_wp_check;
  bool is_colliding = true;
  while (is_colliding)
  {
    srv_wp_check.request.x  = goal_pose_.position.x;
    srv_wp_check.request.y = goal_pose_.position.y;
    if (clt_waypoint_checker_excavator_.call(srv_wp_check))
    {
      // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
      is_colliding = srv_wp_check.response.collision;
      if(is_colliding)
      {
        srv_wp_gen.request.start  = true;
        srv_wp_gen.request.next  = true;
        if (clt_wp_gen_excavator_.call(srv_wp_gen))
        {
          // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
          goal_pose_excavator_ = srv_wp_gen.response.goal;
	        waypoint_type_excavator_ = srv_wp_gen.response.type;

        }
        else
        {
          ROS_ERROR("Failed to call service Generate Waypoint");
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to call service Waypoint checker");
    }
  }

  goal_yaw_excavator_ = atan2(goal_pose_.position.y - current_pose_excavator_.position.y, goal_pose_excavator_.position.x - current_pose_excavator_.position.x);

  srv_brake.request.brake_force  = 0.0;
  if (clt_srcp2_brake_rover_excavator_.call(srv_brake))
  {
     ROS_INFO_STREAM("SM: Brake Enabled? "<< srv_brake.response.finished);

  }
  else
  {
      ROS_ERROR("Failed to call service Brake");
  }
  rotateToHeadingExcavator(goal_yaw_excavator_);

}


void SmRd1::rotateToHeadingExcavator(double desired_yaw)
{
  ros::Rate rateRotateToHeading(20);

  ROS_INFO("Starting yaw control.");

  double yaw_thres = 0.1;

  ros::spinOnce();

  ROS_INFO_STREAM("BEFORE WHILE Yaw: "<<yaw_excavator_<<", desired yaw: "<< desired_yaw);

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
  flag_heading_fail_excavator=false;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeoutHeading(10.0); // Timeout of 20 seconds

  while(fabs(yaw_error) > yaw_thres)
  {
    driving_tools::RotateInPlace srv_turn;
    srv_turn.request.throttle  = copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error);

    if (clt_rip_excavator_.call(srv_turn))
    {
      ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
    }
    else
    {
      ROS_ERROR("Failed to call service Stop");
    }

    rateRotateToHeading.sleep();
    ros::spinOnce();
    ROS_WARN("Trying to control yaw to desired angles.");
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
    ROS_INFO_STREAM("Yaw error: "<<yaw_error);
    ROS_ERROR_STREAM("TIME: "<<ros::Time::now() - start_time);
    ROS_ERROR_STREAM("TIMEOUT: "<< timeoutHeading);
    if (ros::Time::now() - start_time > timeoutHeading)
    {
      ROS_ERROR_STREAM("Yaw Control Failed, Possible Stuck, breaking: "<<yaw_error);
      flag_heading_fail_excavator = true;
      break;
    }
  }

  if (flag_heading_fail_excavator)
  {
    ROS_WARN("RECOVERY ACTION INITIATED in YAW CONTROL!");
    driving_tools::Stop srv_stop;
    srv_stop.request.enableStop  = true;
    if (clt_stop_excavator_.call(srv_stop))
    {
      ros::Duration(2).sleep();
    }
    else
    {
      ROS_ERROR("Failed to call service Stop");
    }
    driving_tools::MoveForward srv_drive;
    srv_drive.request.throttle  = -0.3;
    if (clt_drive_excavator_.call(srv_drive))
    {
            ros::Duration(4.0).sleep();
            ROS_INFO_STREAM("SM: Backward Drive Enabled? "<< srv_drive.response);
    }
    else
    {
            ROS_ERROR("Failed to call service Drive");
    }

    flag_heading_fail_excavator=false;
  }
  else
  {
    driving_tools::Stop srv_stop;
    srv_stop.request.enableStop  = true;
    if (clt_stop_excavator_.call(srv_stop))
    {
      ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
    }
    else
    {
      ROS_ERROR("Failed to call service Stop");
    }
  }
}


void SmRd1::rotateToHeadingHauler(double desired_yaw)
{
  ros::Rate rateRotateToHeading(20);

  ROS_INFO("Starting yaw control.");

  double yaw_thres = 0.1;

  ros::spinOnce();

  ROS_INFO_STREAM("BEFORE WHILE Yaw: "<<yaw_hauler_<<", desired yaw: "<< desired_yaw);

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
  flag_heading_fail_hauler=false;
  ros::Time start_time = ros::Time::now();
  ros::Duration timeoutHeading(10.0); // Timeout of 20 seconds

  while(fabs(yaw_error) > yaw_thres)
  {
    driving_tools::RotateInPlace srv_turn;
    srv_turn.request.throttle  = copysign(0.1*(1 + fabs(yaw_error)/M_PI), -yaw_error);

    if (clt_rip_hauler_.call(srv_turn))
    {
      ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
    }
    else
    {
      ROS_ERROR("Failed to call service Stop");
    }

    rateRotateToHeading.sleep();
    ros::spinOnce();
    ROS_WARN("Trying to control yaw to desired angles.");
    yaw_error = desired_yaw - yaw_hauler_;
    ROS_INFO_STREAM("Yaw error: "<<yaw_error);
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
    ROS_ERROR_STREAM("TIME: "<<ros::Time::now() - start_time);
    ROS_ERROR_STREAM("TIMEOUT: "<< timeoutHeading);
    if (ros::Time::now() - start_time > timeoutHeading)
    {
      ROS_ERROR_STREAM("Yaw Control Failed, Possible Stuck, breaking: "<<yaw_error);
      flag_heading_fail_hauler = true;
      break;
    }
  }

  if (flag_heading_fail_hauler)
  {
    ROS_WARN("RECOVERY ACTION INITIATED in YAW CONTROL!");
    driving_tools::Stop srv_stop;
    srv_stop.request.enableStop  = true;
    if (clt_stop_hauler_.call(srv_stop))
    {
      ros::Duration(2).sleep();
    }
    else
    {
      ROS_ERROR("Failed to call service Stop");
    }
    driving_tools::MoveForward srv_drive;
    srv_drive.request.throttle  = -0.3;
    if (clt_drive_hauler_.call(srv_drive))
    {
            ros::Duration(4.0).sleep();
            ROS_INFO_STREAM("SM: Backward Drive Enabled? "<< srv_drive.response);
    }
    else
    {
            ROS_ERROR("Failed to call service Drive");
    }

    flag_heading_fail_hauler=false;
  }
  else
  {
    driving_tools::Stop srv_stop;
    srv_stop.request.enableStop  = true;
    if (clt_stop_hauler_.call(srv_stop))
    {
      ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
    }
    else
    {
      ROS_ERROR("Failed to call service Stop");
    }
  }
}
