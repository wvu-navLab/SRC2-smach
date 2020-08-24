#include <state_machine/sm_rd2.hpp>

SmRd2::SmRd2()
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_hauler_pub = nh.advertise<std_msgs::Int64>("/hauler_1/state_machine/state", 10);
  sm_state_excavator_pub = nh.advertise<std_msgs::Int64>("/excavator_1/state_machine/state", 10);
  
  // Subscribers
  odometry_excavator_sub = nh.subscribe("/excavator_1/localization/odometry/sensor_fusion", 1, &SmRd2::odometryExcavatorCallback, this);
  localized_base_excavator_sub = nh.subscribe("/excavator_1/state_machine/localized_base_excavator", 1, &SmRd2::localizedBaseExcavatorCallback, this);
  waypoint_unreachable_excavator_sub = nh.subscribe("/excavator_1/state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableExcavatorCallback, this);
  arrived_at_waypoint_excavator_sub = nh.subscribe("/excavator_1/state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointExcavatorCallback, this);
  volatile_detected_excavator_sub = nh.subscribe("/excavator_1/state_machine/volatile_detected_excavator", 1, &SmRd2::volatileDetectedExcavatorCallback, this);
  volatile_recorded_excavator_sub = nh.subscribe("/excavator_1/state_machine/volatile_recorded_excavator", 1, &SmRd2::volatileRecordedExcavatorCallback, this);
  localization_failure_excavator_sub = nh.subscribe("/excavator_1/state_machine/localization_failure_excavator", 1, &SmRd2::localizationFailureExcavatorCallback, this);

  // Subscribers
  odometry_hauler_sub = nh.subscribe("/hauler_1/localization/odometry/sensor_fusion", 1, &SmRd2::odometryHaulerCallback, this);
  localized_base_hauler_sub = nh.subscribe("/hauler_1/state_machine/localized_base_hauler", 1, &SmRd2::localizedBaseHaulerCallback, this);
  waypoint_unreachable_hauler_sub = nh.subscribe("/hauler_1/state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableHaulerCallback, this);
  arrived_at_waypoint_hauler_sub = nh.subscribe("/hauler_1/state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointHaulerCallback, this);
  volatile_detected_hauler_sub = nh.subscribe("/hauler_1/state_machine/volatile_detected_hauler", 1, &SmRd2::volatileDetectedHaulerCallback, this);
  volatile_recorded_hauler_sub = nh.subscribe("/hauler_1/state_machine/volatile_recorded_hauler", 1, &SmRd2::volatileRecordedHaulerCallback, this);
  localization_failure_hauler_sub = nh.subscribe("/hauler_1/state_machine/localization_failure_hauler", 1, &SmRd2::localizationFailureHaulerCallback, this);
//////////////////////////////////

std::string excavator_1 = "excavator_1";
std::string hauler_1 = "hauler_1";

//////////////////////////////
  // Clients
  // clt_true_pose_excavator_ = nh.serviceClient<pose_update::PoseUpdate>(excavator_1+"/localization/true_pose_update");
  // clt_wp_gen_excavator_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>(excavator_1+"/navigation/generate_goal");
  // clt_wp_nav_set_goal_excavator_ = nh.serviceClient<waypoint_nav::SetGoal>(excavator_1+"/navigation/set_goal");
  // clt_wp_nav_interrupt_excavator_ = nh.serviceClient<waypoint_nav::Interrupt>(excavator_1+"/navigation/interrupt");
  // clt_stop_excavator_ = nh.serviceClient<driving_tools::Stop>(excavator_1+"/driving/stop");
  // // clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>(excavator_1+"/volatile/report");
  //
  // // Clients
  // clt_true_pose_hauler_ = nh.serviceClient<pose_update::PoseUpdate>(hauler_1+"/localization/true_pose_update");
  // clt_wp_gen_hauler_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>(hauler_1+"/navigation/generate_goal");
  // clt_wp_nav_set_goal_hauler_ = nh.serviceClient<waypoint_nav::SetGoal>(hauler_1+"/navigation/set_goal");
  // clt_wp_nav_interrupt_hauler_ = nh.serviceClient<waypoint_nav::Interrupt>(hauler_1+"/navigation/interrupt");
  // clt_stop_hauler_ = nh.serviceClient<driving_tools::Stop>(hauler_1+"/driving/stop");
  // clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>(hauler_1+"/volatile/report");


  clt_true_pose_excavator_ = nh.serviceClient<pose_update::PoseUpdate>("/excavator_1/localization/true_pose_update");
  clt_next_vol_excavator_ = nh.serviceClient<round2_volatile_handler::NextVolatileLocation>("/excavator_1/volatile/next");
  clt_wp_nav_set_goal_excavator_ = nh.serviceClient<waypoint_nav::SetGoal>("/excavator_1/navigation/set_goal");
  clt_wp_nav_interrupt_excavator_ = nh.serviceClient<waypoint_nav::Interrupt>("/excavator_1/navigation/interrupt");
  clt_stop_excavator_ = nh.serviceClient<driving_tools::Stop>("/excavator_1/driving/stop");
  // clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>(excavator_1+"/volatile/report");

  // Clients
  clt_true_pose_hauler_ = nh.serviceClient<pose_update::PoseUpdate>("/hauler_1/localization/true_pose_update");
  clt_wp_nav_set_goal_hauler_ = nh.serviceClient<waypoint_nav::SetGoal>("/hauler_1/navigation/set_goal");
  clt_wp_nav_interrupt_hauler_ = nh.serviceClient<waypoint_nav::Interrupt>("/hauler_1/navigation/interrupt");
  clt_stop_hauler_ = nh.serviceClient<driving_tools::Stop>("/hauler_1/driving/stop");
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
    ROS_INFO("volatile_detected_distance_hauler: %f",volatile_detected_distance_hauler);
    // ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    ROS_INFO("flag_volatile_recorded_hauler: %i",flag_volatile_recorded_hauler);
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
    else if((flag_arrived_at_waypoint_excavator || flag_waypoint_unreachable_excavator) && (volatile_detected_distance_excavator==-1.0) && !flag_localizing_volatile_excavator && !flag_brake_engaged_excavator)
    {
      state_to_exec.at(_planning) = 1;
    }
    else if((!flag_arrived_at_waypoint_excavator && !flag_waypoint_unreachable_excavator) && !flag_brake_engaged_excavator)
    {
      state_to_exec.at(_traverse) = 1;
    }
    else if(((volatile_detected_distance_excavator>=0)  || flag_localizing_volatile_excavator) && !flag_brake_engaged_excavator)
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

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_hauler_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("HAULER Failed to call service Stop");
  }
  if (clt_stop_excavator_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR Failed to call service Stop");
  }
  // Get True Pose
  pose_update::PoseUpdate srv_upd_pose;
  srv_upd_pose.request.start  = true;
  if (clt_true_pose_hauler_.call(srv_upd_pose))
  {
    // ROS_INFO_STREAM("Success? "<< srv_upd_pose.response.success);
  }
  else
  {
    ROS_ERROR("HAULER Failed to call service Pose Update");
  }
  if (clt_true_pose_excavator_.call(srv_upd_pose))
  {
    // ROS_INFO_STREAM("Success? "<< srv_upd_pose.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR Failed to call service Pose Update");
  }
  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_excavator_pub.publish(state_msg);
}

void SmRd2::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint_excavator = false;
  flag_waypoint_unreachable_excavator = false;

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_excavator_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR Failed to call service Stop");
  }

  // geometry_msgs::Pose goal_pose;
  // ROS_INFO_STREAM("EXCAVATOR goal pose: " << goal_pose);
  // // Generate Goal
  // waypoint_gen::GenerateWaypoint srv_wp_gen;
  // srv_wp_gen.request.start  = true;
  // if (clt_wp_gen_excavator_.call(srv_wp_gen))
  // {
  //   // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
  //   goal_pose = srv_wp_gen.response.goal;
  // }
  // else
  // {
  //   ROS_ERROR("EXCAVATOR Failed to call service Generate Waypoint");
  // }

  std::string vol_type;
  geometry_msgs::Pose goal_pose;
  ROS_INFO_STREAM("EXCAVATOR goal pose: " << goal_pose);
  // Generate Goal
  round2_volatile_handler::NextVolatileLocation srv_next_vol;
  srv_next_vol.request.rover_position  = current_pose_excavator_;
  if (clt_next_vol_excavator_.call(srv_next_vol))
  {
    // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
    goal_pose = srv_next_vol.response.vol_position;
    vol_type = srv_next_vol.response.type;
  }
  else
  {
    ROS_ERROR("EXCAVATOR Failed to call service Next Volatile Pose");
  }

  // Get True Pose
  waypoint_nav::SetGoal srv_wp_nav;
  srv_wp_nav.request.start = true;
  srv_wp_nav.request.goal = goal_pose;
  if (clt_wp_nav_set_goal_excavator_.call(srv_wp_nav))
  {
    // ROS_INFO_STREAM("Success? "<< srv_wp_nav.response.success);
  }
  else
  {
    ROS_ERROR("EXCAVATOR Failed to call service Drive to Waypoint");
  }

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
  if(flag_volatile_recorded_excavator)
  {
    volatile_detected_distance_excavator = -1.0;
    flag_localizing_volatile_excavator = false;
    flag_volatile_recorded_excavator = false;
    flag_arrived_at_waypoint_excavator = true;
    flag_waypoint_unreachable_excavator = false;
  }
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
  ros::Duration(2.0).sleep();
  // Get True Pose
  waypoint_nav::Interrupt srv_wp_nav;
  srv_wp_nav.request.interrupt = true;
  if (clt_wp_nav_interrupt_excavator_.call(srv_wp_nav))
  {
    ROS_INFO_STREAM("I called service interrupt ");
  }
  else
  {
    ROS_ERROR("Failed to call service Interrupt Nav");
  }
  ros::Duration(2.0).sleep();
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


   // volatile_handler::VolatileReport srv_vol_rep;
   // srv_vol_rep.request.start = true;
   // if (clt_vol_report_.call(srv_vol_rep))
   // {
   //   ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
   //    flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
   //   flag_arrived_at_waypoint = false;
   //   // srv_wp_nav.request.interrupt = false;
   //   // if (clt_wp_nav_interrupt_.call(srv_wp_nav))
   //   // {
   //   //   ROS_INFO_STREAM("AFTER FOUND VOL I called service interrupt ");
   //   // }
   //   // else
   //   // {
   //   //   ROS_ERROR("AFTER FOUDN VOL Failed to call service Interrupt Nav");
   //   // }
   //   // // Turn off brake
   //   // srv_stop.request.enableStop  = false;
   //   // if (clt_stop_.call(srv_stop))
   //   // {
   //   //   ROS_INFO_STREAM("SM: Stopping Disabled? "<< srv_stop.response);
   //   // }
   //   // else
   //   // {
   //   //   ROS_ERROR("Failed to call service Start");
   //   // }
   // }
   // else
   // {
   //   ROS_ERROR("Service Did not Collect Points");
   //   // flag_arrived_at_waypoint = true;
   // }
/*
    // Turn off brake
    srv_stop.request.enableStop  = false;
    if (clt_stop_.call(srv_stop))
    {
      ROS_INFO_STREAM("SM: Stopping Disabled? "<< srv_stop.response);
    }
    else
    {
      ROS_ERROR("Failed to call service Start");
    } */

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

void SmRd2::waypointUnreachableExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable_excavator = msg->data;
}

void SmRd2::arrivedAtWaypointExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint_excavator = msg->data;
}

void SmRd2::volatileDetectedExcavatorCallback(const std_msgs::Float32::ConstPtr& msg)
{
  volatile_detected_distance_excavator = msg->data;
}

void SmRd2::volatileRecordedExcavatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_volatile_recorded_excavator = msg->data;
  if (flag_volatile_recorded_excavator == true)
  {
   volatile_detected_distance_excavator = -1.0;
  }
}

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

void SmRd2::volatileDetectedHaulerCallback(const std_msgs::Float32::ConstPtr& msg)
{
  volatile_detected_distance_hauler = msg->data;
}

void SmRd2::volatileRecordedHaulerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_volatile_recorded_hauler = msg->data;
  if (flag_volatile_recorded_hauler == true)
  {
   volatile_detected_distance_hauler = -1.0;
  }
}

void SmRd2::localizationFailureHaulerCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure_hauler = msg->data;
}

//------------------------------------------------------------------------------------------------------------------------
