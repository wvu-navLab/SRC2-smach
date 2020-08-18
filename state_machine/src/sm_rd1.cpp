#include <state_machine/sm_rd1.hpp>

SmRd1::SmRd1()
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_pub = nh.advertise<std_msgs::Int64>("state_machine/state", 10);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base", 1, &SmRd1::localizedBaseCallback, this);
  waypoint_unreachable_sub = nh.subscribe("state_machine/waypoint_unreachable", 1, &SmRd1::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("state_machine/arrived_at_waypoint", 1, &SmRd1::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("state_machine/volatile_detected", 1, &SmRd1::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("state_machine/volatile_recorded", 1, &SmRd1::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("state_machine/localization_failure", 1, &SmRd1::localizationFailureCallback, this);

  // Clients
  clt_true_pose_ = nh.serviceClient<pose_update::PoseUpdate>("localization/true_pose_update");
  clt_wp_gen_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("navigation/generate_goal");
  clt_wp_nav_set_goal_ = nh.serviceClient<waypoint_nav::SetGoal>("navigation/set_goal");
  clt_wp_nav_interrupt_ = nh.serviceClient<waypoint_nav::Interrupt>("navigation/interrupt");
  clt_stop_ = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>("volatile/report");
}

void SmRd1::run()
{
  ros::Rate loop_rate(2); // Hz
  while(ros::ok())
  {
    // Debug prints +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    ROS_INFO("flag_localized_base: %i",flag_localized_base);
    ROS_INFO("flag_have_true_pose: %i",flag_have_true_pose);
    // ROS_INFO("flag_waypoint_unreachable: %i",flag_waypoint_unreachable);
    ROS_INFO("flag_arrived_at_waypoint: %i",flag_arrived_at_waypoint);
    // ROS_INFO("flag_volatile_detected: %i",flag_volatile_detected);
    // ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    // ROS_INFO("flag_volatile_recorded: %i",flag_volatile_recorded);
    // ROS_INFO("flag_volatile_unreachable: %i",flag_volatile_unreachable);
    // ROS_INFO("flag_localization_failure: %i",flag_localization_failure);
    // ROS_INFO("flag_brake_engaged: %i",flag_brake_engaged);
    ROS_INFO("flag_fallthrough_condition: %i",flag_fallthrough_condition);
    //---------------------------------------------------------------------------------------------------------------------


    // Conditional flag logic (Preemptive conditions) +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if(flag_volatile_detected && !flag_localizing_volatile && !flag_localization_failure && flag_have_true_pose)
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
    else if((flag_arrived_at_waypoint || flag_waypoint_unreachable) && !flag_volatile_detected && !flag_localizing_volatile && !flag_brake_engaged)
    {
      state_to_exec.at(_planning) = 1;
    }
    else if((!flag_arrived_at_waypoint && !flag_waypoint_unreachable) && !flag_brake_engaged)
    {
      state_to_exec.at(_traverse) = 1;
    }
    else if((flag_volatile_detected || flag_localizing_volatile) && !flag_brake_engaged)
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

void SmRd1::statePlanning()
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

void SmRd1::stateTraverse()
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
    flag_volatile_detected = false;
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

void SmRd1::stateVolatileHandler()
{
  // Get True Pose
  waypoint_nav::Interrupt srv_wp_nav;
  srv_wp_nav.request.interrupt = true;
  if (clt_wp_nav_interrupt_.call(srv_wp_nav))
  {
    ROS_INFO_STREAM("Called service Interrupt.");
  }
  else
  {
    ROS_ERROR("Failed to call service Interrupt.");
  }

  // Turn on brake
   driving_tools::Stop srv_stop;
   srv_stop.request.enableStop  = true;
   if (clt_stop_.call(srv_stop))
   {
     ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
   }
   else
   {
     ROS_ERROR("Failed to call service Stop");
   }


   volatile_handler::VolatileReport srv_vol_rep;
   srv_vol_rep.request.start = true;
   if (clt_vol_report_.call(srv_vol_rep))
   {
     ROS_INFO_STREAM("Volatile Accepted? "<< srv_vol_rep.response.success);
   }
   else
   {
     ROS_ERROR("Failed to call service Report");
   }

   // Turn off brake
   srv_stop.request.enableStop  = false;
   if (clt_stop_.call(srv_stop))
   {
     ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
   }
   else
   {
     ROS_ERROR("Failed to call service Start");
   }

   ROS_INFO("VolatileHandler!\n");
   //flag_arrived_at_waypoint = false;
   flag_waypoint_unreachable = false;
   //flag_localizing_volatile = true;
   flag_arrived_at_waypoint = true; // This is temporary for hackathon, since volatile reporting doesn't have all info and doesn't command a waypoint yet.
   std_msgs::Int64 state_msg;
   state_msg.data = _volatile_handler;
   sm_state_pub.publish(state_msg);
}

void SmRd1::stateLost()
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
    ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
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
// void SmRd1::localizedBaseCallback(const std_msgs::Bool::ConstPtr& msg)
void SmRd1::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = msg->data;
  if (flag_localized_base) {
    ROS_INFO("Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

void SmRd1::waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable = msg->data;
}

void SmRd1::arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint = msg->data;
}

void SmRd1::volatileDetectedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_volatile_detected = msg->data;
}

void SmRd1::volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_volatile_recorded = msg->data;
}

void SmRd1::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
}

//------------------------------------------------------------------------------------------------------------------------
