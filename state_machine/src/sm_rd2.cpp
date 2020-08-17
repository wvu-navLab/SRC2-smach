#include <state_machine/sm_rd2.hpp>

SmRd2::SmRd2()
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_pub = nh.advertise<std_msgs::Int64>("state_machine/state", 10);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmRd2::localizedBaseCallback, this);
  waypoint_unreachable_sub = nh.subscribe("state_machine/waypoint_unreachable", 1, &SmRd2::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("state_machine/arrived_at_waypoint", 1, &SmRd2::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("state_machine/volatile_detected", 1, &SmRd2::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("state_machine/volatile_recorded", 1, &SmRd2::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("state_machine/localization_failure", 1, &SmRd2::localizationFailureCallback, this);

  // Clients
  clt_true_pose_ = nh.serviceClient<pose_update::PoseUpdate>("localization/true_pose_update");
  clt_wp_gen_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("navigation/generate_goal");
  clt_wp_nav_set_goal_ = nh.serviceClient<waypoint_nav::SetGoal>("navigation/set_goal");
  clt_wp_nav_interrupt_ = nh.serviceClient<waypoint_nav::Interrupt>("navigation/interrupt");
  clt_stop_ = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>("volatile/report");
}

void SmRd2::run()
{
  ros::Rate loop_rate(2); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ROS_INFO("flag_localized_base: %i",flag_localized_base);
    ROS_INFO("flag_have_true_pose: %i",flag_have_true_pose);
    // ROS_INFO("flag_waypoint_unreachable: %i",flag_waypoint_unreachable);
    ROS_INFO("flag_arrived_at_waypoint: %i",flag_arrived_at_waypoint);
    ROS_INFO("volatile_detected_distance: %f",volatile_detected_distance);
    // ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    ROS_INFO("flag_volatile_recorded: %i",flag_volatile_recorded);
    // ROS_INFO("flag_volatile_unreachable: %i",flag_volatile_unreachable);
    // ROS_INFO("flag_localization_failure: %i",flag_localization_failure);
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
  ROS_INFO("Initialize!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  // Get True Pose
  pose_update::PoseUpdate srv_upd_pose;
  srv_upd_pose.request.start  = true;
  if (clt_true_pose_.call(srv_upd_pose))
  {
    // ROS_INFO_STREAM("Success? "<< srv_upd_pose.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Pose Update");
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_pub.publish(state_msg);
}

void SmRd2::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  geometry_msgs::Pose goal_pose;
  ROS_INFO_STREAM("goal pose: " << goal_pose);
  // Generate Goal
  waypoint_gen::GenerateWaypoint srv_wp_gen;
  srv_wp_gen.request.start  = true;
  if (clt_wp_gen_.call(srv_wp_gen))
  {
    // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
    goal_pose = srv_wp_gen.response.goal;
  }
  else
  {
    ROS_ERROR("Failed to call service Generate Waypoint");
  }

  // Get True Pose
  waypoint_nav::SetGoal srv_wp_nav;
  srv_wp_nav.request.start = true;
  srv_wp_nav.request.goal = goal_pose;
  if (clt_wp_nav_set_goal_.call(srv_wp_nav))
  {
    // ROS_INFO_STREAM("Success? "<< srv_wp_nav.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Drive to Waypoint");
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _planning;
  sm_state_pub.publish(state_msg);
}

void SmRd2::stateTraverse()
{
  ROS_INFO("Traverse!\n");

  if(flag_localized_base && !flag_have_true_pose)
  {
    flag_have_true_pose = true;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }
  if(flag_volatile_recorded)
  {
    volatile_detected_distance = -1.0;
    flag_localizing_volatile = false;
    flag_volatile_recorded = false;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }
  if(flag_recovering_localization && !flag_localization_failure)
  {
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
    flag_recovering_localization = false;
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_pub.publish(state_msg);

}

void SmRd2::stateVolatileHandler()
{
 /* if(volatile_detected_distance>1.0){
	  ROS_INFO(" Not Calling Volatile Report Service Because Not Close Enough %f",volatile_detected_distance);
	  return;

	  // JNG Question, should some sort of status be published here?
   }*/
  ros::Duration(1.0).sleep();
  // Get True Pose
  waypoint_nav::Interrupt srv_wp_nav;
  srv_wp_nav.request.interrupt = true;
  if (clt_wp_nav_interrupt_.call(srv_wp_nav))
  {
    ROS_INFO_STREAM("I called service interrupt ");
  }
  else
  {
    ROS_ERROR("Failed to call service Interrupt Nav");
  }
  ros::Duration(1.0).sleep();
  // Turn on brake
   driving_tools::Stop srv_stop;
   srv_stop.request.enableStop  = true;
   if (clt_stop_.call(srv_stop))
   {
     ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
   }
   else
   {
     ROS_ERROR("Failed to call service Stop");
   }


   volatile_handler::VolatileReport srv_vol_rep;
   srv_vol_rep.request.start = true;
   if (clt_vol_report_.call(srv_vol_rep))
   {
     ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
      flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
     flag_arrived_at_waypoint = false;
     // srv_wp_nav.request.interrupt = false;
     // if (clt_wp_nav_interrupt_.call(srv_wp_nav))
     // {
     //   ROS_INFO_STREAM("AFTER FOUND VOL I called service interrupt ");
     // }
     // else
     // {
     //   ROS_ERROR("AFTER FOUDN VOL Failed to call service Interrupt Nav");
     // }
     // // Turn off brake
     // srv_stop.request.enableStop  = false;
     // if (clt_stop_.call(srv_stop))
     // {
     //   ROS_INFO_STREAM("SM: Stopping Disabled? "<< srv_stop.response);
     // }
     // else
     // {
     //   ROS_ERROR("Failed to call service Start");
     // }
   }
   else
   {
     ROS_ERROR("Service Did not Collect Points");
     // flag_arrived_at_waypoint = true;
   }
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
   flag_waypoint_unreachable = false;
   //flag_localizing_volatile = true;
   // flag_arrived_at_waypoint = true; // This is temporary for hackathon, since volatile reporting doesn't have all info and doesn't command a waypoint yet.
   std_msgs::Int64 state_msg;
   state_msg.data = _volatile_handler;
   sm_state_pub.publish(state_msg);
}

void SmRd2::stateLost()
{
  ROS_INFO("Lost!\n");
  flag_recovering_localization = true;
  flag_localizing_volatile = false;
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  // Break
  driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _lost;
  sm_state_pub.publish(state_msg);
}
//------------------------------------------------------------------------------------------------------------------------


// Callbacks +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// void SmRd2::localizedBaseCallback(const std_msgs::Bool::ConstPtr& msg)
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
  volatile_detected_distance = msg->data;
}

void SmRd2::volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_volatile_recorded = msg->data;
  if (flag_volatile_recorded == true)
  {
   volatile_detected_distance = -1.0;
  }
}

void SmRd2::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
}

//------------------------------------------------------------------------------------------------------------------------
