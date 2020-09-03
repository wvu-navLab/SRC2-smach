#include <state_machine/sm_rd3.hpp>

SmRd3::SmRd3() :
ac("/move_base", true),
move_base_state_(actionlib::SimpleClientGoalState::LOST)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_pub = nh.advertise<std_msgs::Int64>("state_machine/state", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base_scout", 1, &SmRd3::localizedBaseCallback, this);
  mobility_sub = nh.subscribe("state_machine/mobility_scout", 1, &SmRd3::mobilityCallback, this);
  waypoint_unreachable_sub = nh.subscribe("state_machine/waypoint_unreachable", 1, &SmRd3::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("state_machine/arrived_at_waypoint", 1, &SmRd3::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("state_machine/volatile_detected", 1, &SmRd3::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("state_machine/volatile_recorded", 1, &SmRd3::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("state_machine/localization_failure", 1, &SmRd3::localizationFailureCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmRd3::localizationCallback, this);
  driving_mode_sub =nh.subscribe("driving/driving_mode",1, &SmRd3::drivingModeCallback, this);
  lidar_sub =nh.subscribe("laser/scan",1, &SmRd3::lidarCallback, this);

  // Clients
  clt_wp_gen_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("navigation/generate_goal");
  clt_wp_nav_set_goal_ = nh.serviceClient<waypoint_nav::SetGoal>("navigation/set_goal");
  clt_wp_nav_interrupt_ = nh.serviceClient<waypoint_nav::Interrupt>("navigation/interrupt");
  clt_stop_ = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_rip_ = nh.serviceClient<driving_tools::RotateInPlace>("driving/rotate_in_place");
  clt_circ_base_station_ = nh.serviceClient<driving_tools::CirculateBaseStation>("driving/circ_base_station");
  clt_drive_ = nh.serviceClient<driving_tools::MoveForward>("driving/move_forward");
  clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>("volatile/report");
  clt_vol_detect_ = nh.serviceClient<volatile_handler::ToggleDetector>("volatile/toggle_detector");
  clt_lights_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("toggle_light");
  clt_brake_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_approach_base_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("approach_base_station");
  clt_rover_static_ = nh.serviceClient<sensor_fusion::RoverStatic>("sensor_fusion/toggle_rover_static");
  clt_homing_ = nh.serviceClient<sensor_fusion::HomingUpdate>("homing");
  clt_sf_true_pose_ = nh.serviceClient<sensor_fusion::GetTruePose>("true_pose");
  clt_waypoint_checker_ = nh.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  clt_srcp2_brake_rover_= nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  // this is a comment


  driving_mode_=0;

  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();
}

void SmRd3::run()
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
void SmRd3::stateInitialize()
{
  ROS_WARN("Initialization State!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  Lights("0.8");

  while (!clt_approach_base_.waitForExistence())
  {
    ROS_WARN("SCOUT: Waiting for ApproachBaseStation service");
  }

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  if (clt_approach_base_.call(srv_approach_base))
  {
    ROS_INFO("SCOUT: Called service ApproachBaseStation");
    ROS_INFO_STREAM("Success finding the Base? "<< srv_approach_base.response.success.data);
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service ApproachBaseStation");
  }

  Stop(0.0);

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

  // Homing - Initialize Base Station Landmark
  sensor_fusion::HomingUpdate srv_homing;
  srv_homing.request.initializeLandmark = true;
  if (clt_homing_.call(srv_homing))
  {
    ROS_INFO("SCOUT: Called service HomingUpdate");
    base_location_ = srv_homing.response.base_location;
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service HomingUpdate");
  }

  RoverStatic(false);

  Lights("0.2");

  Brake(0.0);

  Drive(-0.3, 2.0);

  Stop(0.0);

  ros::spinOnce();

  ROS_INFO_STREAM("SCOUT: Pose: "<< current_pose_);
  ROS_INFO_STREAM("SCOUT: Base location: "<< base_location_);
  goal_yaw_ = atan2(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);

  Brake (0.0);

  RotateToHeading(goal_yaw_);

  Stop(2.0);
  
  for(int i = 0; i < 5; i++)
  {
<<<<<<< HEAD
    ROS_INFO_STREAM("Try number: " << i);
    MoveAroundBaseStation(8.5, 15);

    RotateInPlace(0.3, 10);

    Stop(2.0);

    // Approach Base Station
    src2_object_detection::ApproachBaseStation srv_approach_base;
    srv_approach_base.request.approach_base_station.data= true;
    if (clt_approach_base_.call(srv_approach_base))
    {
      ROS_INFO("SCOUT: Called service ApproachBaseStation");
      ROS_INFO_STREAM("Success finding the Base? "<< srv_approach_base.response.success.data);
    }
    else
    {
      ROS_ERROR("SCOUT: Failed  to call service ApproachBaseStation");
    }

    Stop(2.0);

    Brake (500.0);
=======
    MoveAroundBaseStation(9.0, 10);

    RotateInPlace(0.3, 3);

    RotateToHeading(goal_yaw_);
>>>>>>> 3dc6c87037a39c8b2c796d0dd7dfb028a0027905

    // Homing - Measurement Update
    sensor_fusion::HomingUpdate srv_homing;
    srv_homing.request.initializeLandmark = false;
    if (clt_homing_.call(srv_homing))
    {
      ROS_INFO("SCOUT: Called service Homing [Update]");
      flag_localization_failure=false;
      flag_arrived_at_waypoint = true;
      flag_completed_homing = true;
    }
    else
    {
      ROS_ERROR("SCOUT: Failed to call service Homing [Update]");
    }

    Lights ("0.2");

    Brake (0.0);
  }

  // make sure we dont latch to a vol we skipped while homing
  volatile_detected_distance = -1.0;
  std_msgs::Int64 state_msg;
  state_msg.data = _initialize;
  sm_state_pub.publish(state_msg);
}

void SmRd3::statePlanning()
{
  ROS_INFO("Planning!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;

  ROS_INFO("SCOUT: Canceling MoveBase goal.");
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  Stop(0.0);

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

          Brake (100.0);
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
  }

  ClearCostmaps();

  Stop (2.0);

  Brake (0.0);

  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw_);
  ROS_INFO_STREAM("SCOUT: Sending goal to MoveBase: " << move_base_goal);
  ac.sendGoal(move_base_goal, boost::bind(&SmRd3::doneCallback, this,_1,_2), boost::bind(&SmRd3::activeCallback, this), boost::bind(&SmRd3::feedbackCallback, this,_1));
  ac.waitForResult(ros::Duration(0.25));

  std_msgs::Int64 state_msg;
  state_msg.data = _planning;
  sm_state_pub.publish(state_msg);
}
void SmRd3::stateTraverse()
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
    }
  }

  if (!flag_mobility)
  {
    ROS_INFO("SCOUT: Recovering maneuver initialized.");
    immobilityRecovery();
    flag_have_true_pose = true;
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
  }

  move_base_state_ = ac.getState();
  int mb_state =(int) move_base_state_.state_;
  ROS_WARN_STREAM("MoveBase status: "<< mb_state);

  if(mb_state==5 || mb_state==7)
  {
    ROS_ERROR("SCOUT: MoveBase has failed to make itself useful.");
    flag_waypoint_unreachable= true;

    Stop (0.0);

    ClearCostmaps();
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_pub.publish(state_msg);

}

void SmRd3::stateVolatileHandler()
{

  ROS_WARN("Volatile Handling State!");
}

void SmRd3::stateLost()
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

  ToggleDetector(false);

  Stop (0.0);

  Lights ("0.8");

  // Approach Base Station
  src2_object_detection::ApproachBaseStation srv_approach_base;
  srv_approach_base.request.approach_base_station.data= true;
  if (clt_approach_base_.call(srv_approach_base))
  {
    ROS_INFO("SCOUT: Called service ApproachBaseStation");
    if(!srv_approach_base.response.success.data)
    {
      ROS_INFO_STREAM("Defining goal from base location");

      goal_yaw_ = atan2(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);

      RotateToHeading(goal_yaw_);

      move_base_msgs::MoveBaseGoal move_base_goal;
      ac.waitForServer();
      setPoseGoal(move_base_goal, base_location_.x, base_location_.y, goal_yaw_);
      ROS_INFO_STREAM("SCOUT: Sending goal to MoveBase: " << move_base_goal);
      ac.sendGoal(move_base_goal, boost::bind(&SmRd3::doneCallback, this,_1,_2), boost::bind(&SmRd3::activeCallback, this), boost::bind(&SmRd3::feedbackCallback, this,_1));
      ac.waitForResult(ros::Duration(0.25));
    }
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service ApproachBaseStation");
  }

  Brake (500.0);

  // Homing - Measurement Update
  sensor_fusion::HomingUpdate srv_homing;
  srv_homing.request.initializeLandmark = false;
  if (clt_homing_.call(srv_homing))
  {
    ROS_INFO("SCOUT: Called service Homing [Update]");
    flag_localization_failure=false;
    flag_arrived_at_waypoint = true;
    flag_completed_homing = true;
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service Homing [Update]");
  }

  Lights ("0.2");

  Brake (0.0);

  Drive (-0.3, 2.0);

  Stop (2.0);

  RotateInPlace (0.2, 1.0);

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
void SmRd3::localizedBaseCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_localized_base = msg->data;
  if (flag_localized_base) {
    ROS_WARN_ONCE("Initial Localization Successful = %i",flag_localized_base);

  }
  else {
    ROS_INFO("Waiting for Initial Localization  = %i",flag_localized_base);
  }
}

void SmRd3::mobilityCallback(const std_msgs::Int64::ConstPtr& msg)
{
  flag_mobility = msg->data;
  if (flag_mobility) {
    ROS_WARN_ONCE("Rover is traversing = %i",flag_mobility);

  }
  else {
    ROS_ERROR("ROVER IMMOBILIZATION!  = %i",flag_mobility);
  }
}

void SmRd3::waypointUnreachableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_waypoint_unreachable = msg->data;
}

void SmRd3::arrivedAtWaypointCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_arrived_at_waypoint = msg->data;
}

void SmRd3::volatileDetectedCallback(const std_msgs::Float32::ConstPtr& msg)
{

  prev_volatile_detected_distance = volatile_detected_distance;
  volatile_detected_distance = msg->data;

}

void SmRd3::volatileRecordedCallback(const std_msgs::Bool::ConstPtr& msg)
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

void SmRd3::localizationFailureCallback(const std_msgs::Bool::ConstPtr& msg)
{
  flag_localization_failure = msg->data;
}
void SmRd3::drivingModeCallback(const std_msgs::Int64::ConstPtr& msg){
  driving_mode_=msg->data;
}
void SmRd3::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev_ = yaw_;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);

  tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);

}


void SmRd3::setPoseGoal(move_base_msgs::MoveBaseGoal &poseGoal, double x, double y, double yaw) // m, m, rad
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

void SmRd3::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
    actionDone_ = true;
    // ROS_INFO("Goal done");
}
void SmRd3::activeCallback()
{
    // ROS_INFO("Goal went active");
}
void SmRd3::feedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
    ROS_INFO("Got feedback");
}

void SmRd3::RotateToHeading(double desired_yaw)
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

    ROS_INFO_STREAM("Trying to control yaw to desired angles. Yaw error: "<<yaw_error);
  }

  Stop(0.0);
}

void SmRd3::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<float> ranges = msg->ranges;
  double min_range = 1000;
  for (int i=0; i<ranges.size(); ++i)
  {
    if (ranges[i] < min_range)
    {
      min_range = ranges[i];
    }
  }
  range_ = min_range;
  ROS_INFO_STREAM("Range to Base Station: " << range_);
}

void SmRd3::MoveAroundBaseStation(double desired_radius, double time)
{
  ros::Rate control_rate(20);

  ROS_INFO("Starting circulate base station control.");

  double radius_threshold = 0.1;

  ros::spinOnce();

  double radius = hypot(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);
  double radius_error = desired_radius - radius;

  ROS_INFO_STREAM("Radius error: " << radius_error);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeoutCirculate(time); // Timeout of 20 seconds

  while(ros::Time::now() - start_time > timeoutCirculate)
  {
<<<<<<< HEAD
    while(abs(radius_error) > radius_threshold)
    {
      ROS_INFO_STREAM("Radius to Base Station: " << radius);
      ROS_INFO_STREAM("Radius CMD to Base Station: " << radius + 1.2*radius_error);
=======
    double radius = hypot(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);

    ROS_INFO_STREAM("Radius to Base Station: " << radius);
    ROS_INFO_STREAM("Radius CMD to Base Station: " << radius + 0.1*range_error);
>>>>>>> 3dc6c87037a39c8b2c796d0dd7dfb028a0027905

      CirculateBaseStation(0.3, radius + 1.2*radius_error, 0.0);


      control_rate.sleep();
      ros::spinOnce();
      radius = hypot(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);
      radius_error = desired_radius - radius;

      ROS_INFO_STREAM("Trying to control yaw to desired angles. Range error: "<<radius_error);

    }
  }

  Stop(0.0);
}


void SmRd3::immobilityRecovery()
{
  ros::Rate rateImmobilityRecovery(0.5);

  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_WARN("Starting Recovery.");

  Stop(2.0);

  Brake(100.0);

  Brake(0.0);

  Drive(-0.4, 2.0);

  Stop(0.0);

  Brake(100.0);

  Brake(0.0);

}

void SmRd3::ClearCostmaps()
{
  // Clear the costmap
  std_srvs::Empty emptymsg;
  ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(3.0));
  if (ros::service::call("/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("SCOUT: Called service to clear costmap layers.");
  }
  else
  {
     ROS_ERROR("SCOUT: Failed calling clear_costmaps service.");
  }
}

void SmRd3::Lights(std::string intensity)
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

void SmRd3::RotateInPlace(double throttle, double time)
{
  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = throttle;
  if (clt_rip_.call(srv_turn))
  {
    ROS_INFO("SCOUT: Called service RotateInPlace");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("SCOUT: Failed to call service RotateInPlace");
  }
}

void SmRd3::CirculateBaseStation(double throttle, double radius, double time)
{
  driving_tools::CirculateBaseStation srv_circ_base_station;
  srv_circ_base_station.request.throttle  = throttle;
  srv_circ_base_station.request.radius  = radius;
  if (clt_circ_base_station_.call(srv_circ_base_station))
  {
    ROS_INFO("SCOUT: Called service CirculateBaseStation");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_INFO("SCOUT: Failed to call service CirculateBaseStation");
  }
}

void SmRd3::Stop(double time)
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

void SmRd3::Brake(double intensity)
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
    ROS_INFO_STREAM("SCOUT: Called service SRCP2 Brake. Engaged?" << flag_brake_engaged);
  }
  else
  {
      ROS_ERROR("SCOUT: Failed  to call service Brake");
  }
}

void SmRd3::Drive(double throttle, double time)
{
  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle = throttle;
  if (clt_drive_.call(srv_drive))
  {
    ROS_INFO("SCOUT: Called service Drive");
    ros::Duration(time).sleep();
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service Drive");
  }
}

void SmRd3::ToggleDetector(bool flag)
{
  volatile_handler::ToggleDetector srv_vol_detect;
  srv_vol_detect.request.on  = flag;
  if (clt_vol_detect_.call(srv_vol_detect))
  {
    ROS_INFO_STREAM("SCOUT: Called service ToggleDetector. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("SCOUT: Failed  to call service ToggleDetector");
  }
}

void SmRd3::RoverStatic(bool flag)
{
  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = flag;
  if (clt_rover_static_.call(srv_rover_static))
  {
    ROS_INFO_STREAM("SCOUT: Called service RoverStatic. Turned on?" << flag);
  }
  else
  {
    ROS_ERROR("SCOUT: Failed to call service RoverStatic");
  }

}





//------------------------------------------------------------------------------------------------------------------------
