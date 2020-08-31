#include <state_machine/sm_rd1.hpp>

SmRd1::SmRd1() :
ac("/move_base", true)
{
  // Initialize ROS, Subs, and Pubs *******************************************
  // Publishers
  sm_state_pub = nh.advertise<std_msgs::Int64>("state_machine/state", 1);

  // Subscribers
  localized_base_sub = nh.subscribe("state_machine/localized_base_scout", 1, &SmRd1::localizedBaseCallback, this);
  waypoint_unreachable_sub = nh.subscribe("state_machine/waypoint_unreachable", 1, &SmRd1::waypointUnreachableCallback, this);
  arrived_at_waypoint_sub = nh.subscribe("state_machine/arrived_at_waypoint", 1, &SmRd1::arrivedAtWaypointCallback, this);
  volatile_detected_sub = nh.subscribe("state_machine/volatile_detected", 1, &SmRd1::volatileDetectedCallback, this);
  volatile_recorded_sub = nh.subscribe("state_machine/volatile_recorded", 1, &SmRd1::volatileRecordedCallback, this);
  localization_failure_sub = nh.subscribe("state_machine/localization_failure", 1, &SmRd1::localizationFailureCallback, this);
  localization_sub  = nh.subscribe("localization/odometry/sensor_fusion", 1, &SmRd1::localizationCallback, this);

  // Clients
  // clt_true_pose_ = nh.serviceClient<pose_update::PoseUpdate>("localization/true_pose_update");

  clt_wp_gen_ = nh.serviceClient<waypoint_gen::GenerateWaypoint>("navigation/generate_goal");
  clt_wp_nav_set_goal_ = nh.serviceClient<waypoint_nav::SetGoal>("navigation/set_goal");
  clt_wp_nav_interrupt_ = nh.serviceClient<waypoint_nav::Interrupt>("navigation/interrupt");
  clt_stop_ = nh.serviceClient<driving_tools::Stop>("driving/stop");
  clt_rip_ = nh.serviceClient<driving_tools::RotateInPlace>("driving/rotate_in_place");
  clt_drive_ = nh.serviceClient<driving_tools::MoveForward>("driving/move_forward");
  clt_vol_report_ = nh.serviceClient<volatile_handler::VolatileReport>("volatile/report");
  clt_lights_ = nh.serviceClient<srcp2_msgs::ToggleLightSrv>("toggle_light");
  clt_brake_ = nh.serviceClient<srcp2_msgs::BrakeRoverSrv>("brake_rover");
  clt_approach_base_ = nh.serviceClient<src2_object_detection::ApproachBaseStation>("approach_base_station");
  clt_rover_static_ = nh.serviceClient<sensor_fusion::RoverStatic>("sensor_fusion/toggle_rover_static");
  clt_homing_ = nh.serviceClient<sensor_fusion::HomingUpdate>("homing");
  clt_sf_true_pose_ = nh.serviceClient<sensor_fusion::GetTruePose>("true_pose");
  clt_waypoint_checker_ = nh.serviceClient<waypoint_checker::CheckCollision>("waypoint_checker");
  // this is a comment




  detection_timer = ros::Time::now();
  not_detected_timer = ros::Time::now();
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
    ROS_INFO("volatile_detected_distance: %f",volatile_detected_distance);
    // ROS_INFO("flag_localizing_volatile: %i",flag_localizing_volatile);
    ROS_INFO("flag_volatile_recorded: %i",flag_volatile_recorded);
    // ROS_INFO("flag_volatile_unreachable: %i",flag_volatile_unreachable);
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
  ROS_INFO("Initialize!\n");
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;


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
  while (!clt_approach_base_.waitForExistence())
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
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  // // Get True Pose
  // pose_update::PoseUpdate srv_upd_pose;
  // srv_upd_pose.request.start  = true;
  // if (clt_true_pose_.call(srv_upd_pose))
  // {
  //   // ROS_INFO_STREAM("Success? "<< srv_upd_pose.response.success);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service Pose Update");
  // }

  // Update SF with True Pose
  sensor_fusion::GetTruePose srv_sf_true_pose;
  srv_sf_true_pose.request.start = true;

  while (!clt_sf_true_pose_.waitForExistence())
  {
    ROS_ERROR("WAITING FOR TRUE POSE");
  }
  if (clt_sf_true_pose_.call(srv_sf_true_pose))
  {
    ROS_INFO_STREAM("Success STATUS OF srv_sf_true_pose? "<< srv_sf_true_pose.response.success);
  }
  else
  {
    ROS_INFO_STREAM("STATUS OF srv_sf_true_pose" << srv_sf_true_pose.response.success);
    ROS_ERROR("Failed to call service Pose Update");
  }

  rotateToHeading(0.5);

  // Start attitude constraints for static rover
  sensor_fusion::RoverStatic srv_rover_static;
  srv_rover_static.request.rover_static  = true;
  if (clt_rover_static_.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("Success? "<< srv_rover_static.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service RoverStatic");
  }

  // Homing - Initialize Base Station Landmark
  sensor_fusion::HomingUpdate srv_homing;
  srv_homing.request.initializeLandmark = true;
  if (clt_homing_.call(srv_homing))
  {
    base_location_ = srv_homing.response.base_location;
    // ROS_INFO_STREAM("Success? "<< srv_homing.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service HomingUpdate");
  }

  // Stop attitude constraints for static rover
  srv_rover_static.request.rover_static  = false;
  if (clt_rover_static_.call(srv_rover_static))
  {
    // ROS_INFO_STREAM("Success? "<< srv_rover_static.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service RoverStatic");
  }

  // Turn on the Lights
  srv_lights.request.data  = "0.2";
  if (clt_lights_.call(srv_lights))
  {
    // ROS_INFO_STREAM("Success? "<< srv_lights.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service ToggleLight");
  }

  std_srvs::Empty emptymsg;


  ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(2.0));
  if (ros::service::call("/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("every costmap layers are cleared except static layer");
  }
  else
  {
     ROS_INFO("failed calling clear_costmaps service");
  }

  driving_tools::RotateInPlace srv_turn;

  srv_turn.request.throttle  = 0.2;
   ros::Duration(1.0).sleep();
  if (clt_rip_.call(srv_turn))
  {
          ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
          ros::Duration(5.0).sleep();
  }
  else
  {
          ROS_ERROR("Failed to call service Stop");
  }
  // Break Rover
  // driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle  = 0.3;
  if (clt_drive_.call(srv_drive))
  {
          ros::Duration(3.0).sleep();
          ROS_INFO_STREAM("SM: Drive Enabled? "<< srv_drive.response);
  }
  else
  {
          ROS_ERROR("Failed to call service Drive");
  }

  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
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

  // ROS_INFO_STREAM("goal pose: " << goal_pose);
  // Generate Goal
  waypoint_gen::GenerateWaypoint srv_wp_gen;
  srv_wp_gen.request.start  = true;
  if(flag_completed_homing){
    flag_completed_homing=false;
    srv_wp_gen.request.next  = true;

  }
  else{
  srv_wp_gen.request.next  = false;
  }
  if (clt_wp_gen_.call(srv_wp_gen))
  {
    // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);

    goal_pose_ = srv_wp_gen.response.goal;
    waypoint_type_ = srv_wp_gen.response.type;
  }
  else
  {
    ROS_ERROR("Failed to call service Generate Waypoint");
  }
  ROS_INFO_STREAM("goal pose: " << goal_pose_);

  // check waypoint

  waypoint_checker::CheckCollision srv_wp_check;
  bool is_colliding = true;
  while (is_colliding)
  {
    srv_wp_check.request.x  = goal_pose_.position.x;
    srv_wp_check.request.y = goal_pose_.position.y;
    if (clt_waypoint_checker_.call(srv_wp_check))
    {
      // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
      is_colliding = srv_wp_check.response.collision;
      if(is_colliding)
      {
        srv_wp_gen.request.start  = true;
        srv_wp_gen.request.next  = true;
        if (clt_wp_gen_.call(srv_wp_gen))
        {
          // ROS_INFO_STREAM("Success? "<< srv_wp_gen.response.success);
          goal_pose_ = srv_wp_gen.response.goal;
	  waypoint_type_ = srv_wp_gen.response.type;

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

  goal_yaw = atan2(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);

  move_base_msgs::MoveBaseGoal move_base_goal;
  ac.waitForServer();
  setPoseGoal(move_base_goal, goal_pose_.position.x, goal_pose_.position.y, goal_yaw);
  ROS_INFO_STREAM("goal pose after SetposeGOAL: " << move_base_goal);
  ac.sendGoal(move_base_goal);
  ac.waitForResult(ros::Duration(0.25));

  // // Get True Pose
  // waypoint_nav::SetGoal srv_wp_nav;
  // srv_wp_nav.request.start = true;
  // srv_wp_nav.request.goal = goal_pose;
  // if (clt_wp_nav_set_goal_.call(srv_wp_nav))
  // {
  //   // ROS_INFO_STREAM("Success? "<< srv_wp_nav.response.success);
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service Drive to Waypoint");
  // }

  std_srvs::Empty emptymsg;
  ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(2.0));
  if (ros::service::call("/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("every costmap layers are cleared except static layer");
  }
  else
  {
     ROS_INFO("failed calling clear_costmaps service");
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
  double distance_to_goal = std::hypot(goal_pose_.position.y - current_pose_.position.y, goal_pose_.position.x - current_pose_.position.x);
  if (distance_to_goal < 2.0)
  {
    flag_arrived_at_waypoint = true;
    flag_waypoint_unreachable = false;
    if(waypoint_type_ ==0 ){

    	flag_localization_failure = false;
    }
    else{
	ROS_INFO(" Reached a Waypoint Designated for Localization Update Type : %f ",waypoint_type_ );
	flag_localization_failure = true;

    }
  }

  std_msgs::Int64 state_msg;
  state_msg.data = _traverse;
  sm_state_pub.publish(state_msg);

}

void SmRd1::stateVolatileHandler()
{
  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

  ROS_INFO_STREAM("VOL HANDLE STATE");

  // waypoint_nav::Interrupt srv_wp_nav;
  // srv_wp_nav.request.interrupt = true;
  // if (clt_wp_nav_interrupt_.call(srv_wp_nav))
  // {
  //   ROS_INFO_STREAM("Called service Interrupt.");
  // }
  // else
  // {
  //   ROS_ERROR("Failed to call service Interrupt.");

  // }
  //ros::Duration(2.0).sleep();

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

    //  ros::Duration(2.0).sleep();


  volatile_handler::VolatileReport srv_vol_rep;
  srv_vol_rep.request.start = false;
   ros::Time serviceWatchDog;
  //if (fabs(pitch) > M_PI/6.0 || fabs(roll) > M_PI/6.0)
  //{
    if(volatile_detected_distance < VOLATILE_MIN_THRESH){
    srv_vol_rep.request.start = true;
    if (clt_vol_report_.call(srv_vol_rep))

    {
            ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
            flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
            flag_arrived_at_waypoint = false;
	           prev_volatile_detected_distance = 30;

    }
    else
    {
            serviceWatchDog =  ros::Time::now();
            ROS_ERROR("Service Did not Collect Points");
    }
  }
  //}else
  if ( !flag_volatile_recorded )
  {



  int count = 0;

  int max_count = 5;
  ros::Rate rateVol(20);
  double diff;
  const double MAX_TIME = 10;


  while(count < max_count && !flag_localization_failure && !flag_volatile_recorded)
  {
          ROS_INFO_STREAM("While: " << count <<  " " << volatile_detected_distance);
          bool rot_in_place = true;
          int step = 0;
          ros::Time timeout = ros::Time::now();

	        double direction =1.0;
          diff = ros::Time::now().toSec() -timeout.toSec();
          double angle_change = 0;
          while(step < 2 && !flag_localization_failure && !flag_volatile_recorded && diff < MAX_TIME && angle_change < 4*M_PI)
          {
                  ROS_INFO_STREAM("step: " << step << " " << volatile_detected_distance);
                  bool isCurrentDistFalse = volatile_detected_distance > 0;
                  bool isPrevDistFalse = prev_volatile_detected_distance > 0;
                  //bool distXOR = ( (isCurrentDistFalse) && (!isPrevDistFalse) );// ||
                  bool distXOR = ( (!isCurrentDistFalse) && (isPrevDistFalse) );
                  bool minDist = fabs(volatile_detected_distance - prev_volatile_detected_distance) > 0.01;
                  //volatile_detected_distance > min_volatile_detected_distance &&
                  if ( volatile_detected_distance > prev_volatile_detected_distance && minDist || distXOR )
                  {
                          if (direction > 0)
                          {
                                  direction = -1.0;
                                  timeout = ros::Time::now();
                                  angle_change = 0;
                          } else
                          {
                                  direction = 1.0;
                                  timeout = ros::Time::now();
                                  angle_change = 0;
                          }
                          ++step;

                  }

                  driving_tools::RotateInPlace srv_turn;

                  srv_turn.request.throttle  = direction*0.1;

                  if (clt_rip_.call(srv_turn))
                  {
                          ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
                  }
                  else
                  {
                          ROS_ERROR("Failed to call service Rotate");
                  }

                  ROS_INFO_STREAM("SM: Direction "<< direction);

                  if (volatile_detected_distance < VOLATILE_MIN_THRESH && volatile_detected_distance > 0)
                  {
                    srv_stop.request.enableStop  = true;

		                  if (clt_stop_.call(srv_stop))
                    {
                       ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service Stop");
                    }
                          ROS_INFO_STREAM("SM: In Vol Range");
// 			                    if( serviceWatchDog.isValid()){
//                         	while( (ros::Time::now().toSec() - serviceWatchDog.toSec() ) < TIMER_THRESH ){
//                               		ROS_WARN_ONCE( " Wating to Call Score Service When Valid %f " , (TIMER_THRESH- (ros::Time::now().toSec() - serviceWatchDog.toSec()) ));
// //ros::spinOnce();
//                        		 }
//}

                          srv_vol_rep.request.start = true;
                          if (clt_vol_report_.call(srv_vol_rep))
                          {
                                  ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
                                  flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
                                  flag_arrived_at_waypoint = false;

                          }
                          else
                          {

				                          serviceWatchDog =  ros::Time::now();

                                  ROS_ERROR("Service Did not Collect Points");
                                  // flag_arrived_at_waypoint = true;
                          }
                            ros::spinOnce();
                  }
                  if ( volatile_detected_distance > prev_volatile_detected_distance && minDist || distXOR )
                  {
                    ros::Duration(2.0).sleep();
                  }

                 rateVol.sleep();
                  ros::spinOnce();
                  diff = ros::Time::now().toSec() -timeout.toSec();
                  angle_change += fabs(yaw - yaw_prev);

          }

          direction = 1.0;
          step = 0;
          ros::Duration(.1).sleep();
          timeout = ros::Time::now();
          while(step < 2 && !flag_localization_failure && !flag_volatile_recorded && diff < MAX_TIME && angle_change < 4*M_PI)
          {
                  ROS_INFO_STREAM("step: " << step);
                  bool isCurrentDistFalse = volatile_detected_distance > 0;
                  bool isPrevDistFalse = prev_volatile_detected_distance > 0;
                  //bool distXOR = ( (isCurrentDistFalse) && (!isPrevDistFalse) );// ||
                  bool distXOR = ( (!isCurrentDistFalse) && (isPrevDistFalse) );
                  bool minDist = fabs(volatile_detected_distance - prev_volatile_detected_distance) > 0.01;
                  //volatile_detected_distance > min_volatile_detected_distance &&
                  if ( volatile_detected_distance > prev_volatile_detected_distance && minDist || distXOR )
                  {
                          if (direction > 0)
                          {
                                  direction = -1.0;
                                  timeout = ros::Time::now();
                          } else
                          {
                                  direction = 1.0;
                                  timeout = ros::Time::now();
                          }
                          ++step;

                  }

                  ROS_INFO_STREAM("SM: Drive");
                  if( volatile_detected_distance > prev_volatile_detected_distance && minDist || distXOR )
                  {
                          driving_tools::MoveForward srv_drive;
                          srv_drive.request.throttle  = direction*0.1;
                          if (clt_drive_.call(srv_drive))
                          {
                                  ROS_INFO_STREAM("SM: Drive Enabled? "<< srv_drive.response);
                          }
                          else
                          {
                                  ROS_ERROR("Failed to call service Stop");
                          }

                          rateVol.sleep();
                          ros::spinOnce();

                  }

                  ROS_INFO_STREAM("SM: Direction "<< direction);

                  if (volatile_detected_distance < VOLATILE_MIN_THRESH && volatile_detected_distance > 0)
                  {
                    srv_stop.request.enableStop  = true;
                    if (clt_stop_.call(srv_stop))
                    {
                       ROS_INFO_STREAM("SM: Stopping Enabled? "<< srv_stop.response);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service Stop");
                    }
                          ROS_INFO_STREAM("SM: In Vol Range");
                          srv_vol_rep.request.start = true;
                          if (clt_vol_report_.call(srv_vol_rep))
                          {
                                  ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
                                  flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
                                  flag_arrived_at_waypoint = false;

                          }
                          else
                          {
                                  ROS_ERROR("Service Did not Collect Points");
                                  // flag_arrived_at_waypoint = true;
                          }

                  }
                  if ( volatile_detected_distance > prev_volatile_detected_distance && minDist || distXOR )
                  {
                    ros::Duration(2.0).sleep();
                  }

                  rateVol.sleep();
                  ros::spinOnce();
                  diff = ros::Time::now().toSec() -timeout.toSec();


          }
          if (volatile_detected_distance < VOLATILE_MIN_THRESH && volatile_detected_distance > 0)
          {
                  ROS_INFO_STREAM("SM: In Vol Range");
                  srv_vol_rep.request.start = true;
                  if (clt_vol_report_.call(srv_vol_rep))
                  {
                          ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
                          flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
                          flag_arrived_at_waypoint = false;

                  }
                  else
                  {
                          ROS_ERROR("Service Did not Collect Points");
                          // flag_arrived_at_waypoint = true;
                  }

          } else
          {

                  ROS_INFO_STREAM("SM: Drive");
                  while(volatile_detected_distance < prev_volatile_detected_distance && volatile_detected_distance > 0 && prev_volatile_detected_distance > 0 && !flag_localization_failure && !flag_volatile_recorded)
                  {
                          driving_tools::MoveForward srv_drive;
                          srv_drive.request.throttle  = 1.0;
                          if (clt_drive_.call(srv_drive))
                          {
                                  ROS_INFO_STREAM("SM: Drive Enabled? "<< srv_drive.response);
                          }
                          else
                          {
                                  ROS_ERROR("Failed to call service Stop");
                          }

                          rateVol.sleep();
                          ros::spinOnce();
                  }
          }


          ++count;
        //  diff = timeout.toSec() - ros::Time::now().toSec();
  }

  // fallback on too many visits. just assume it can't be reached
  if (count > max_count || diff > MAX_TIME)
  {
          if (!srv_vol_rep.request.start)
          {
                  srv_vol_rep.request.start = true;
                  if (clt_vol_report_.call(srv_vol_rep))
                  {
                          ROS_INFO_STREAM("SM: Volatile Accepted? "<< srv_vol_rep.response);
                          flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
                          flag_arrived_at_waypoint = false;
                          volatile_detected_distance = -1.0;

                  }
                  else
                  {
                          ROS_ERROR("Service Did not Collect Points");
                          flag_volatile_recorded=false;
                          flag_arrived_at_waypoint = false;
                          volatile_detected_distance = -1.0;
                          flag_localization_failure =  true;

                  }
          }else
          {
                  flag_volatile_recorded=true; //JNG CHANGED THIS TO UNCOMMENT 8/12/20
                  flag_arrived_at_waypoint = false;
                  volatile_detected_distance = -1.0;

          }
  }
  }

  // srv_wp_nav.request.interrupt = false;
  // if (clt_wp_nav_interrupt_.call(srv_wp_nav))
  // {
  //       ROS_INFO_STREAM("I called service interrupt ");
  // }
  // else
  // {
  //       ROS_ERROR("Failed to call service Interrupt Nav");
  // }

  ROS_INFO("VolatileHandler\n");
  //flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;
  //flag_localizing_volatile = true;
  // flag_arrived_at_waypoint = true; // This is temporary for hackathon, since volatile reporting doesn't have all info and doesn't command a waypoint yet.
  std_msgs::Int64 state_msg;
  state_msg.data = _volatile_handler;
  sm_state_pub.publish(state_msg);
  detection_timer = ros::Time::now();
  min_volatile_detected_distance = 30;
  volatile_detected_distance = -1.0;
  ROS_INFO("VolatileHandler Exit %f\n",   volatile_detected_distance);
  ++timer_counter;
}

void SmRd1::stateLost()
{
  ROS_INFO("Lost!\n");
  flag_recovering_localization = false;

  flag_localizing_volatile = false;
  flag_arrived_at_waypoint = false;
  flag_waypoint_unreachable = false;


  ac.waitForServer();
  ac.cancelGoal();
  ac.waitForResult(ros::Duration(0.25));

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
  if (clt_approach_base_.call(srv_approach_base))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service ApproachBaseStation");

    ROS_INFO_STREAM("Defining goal from base location");

    goal_yaw = atan2(base_location_.y - current_pose_.position.y, base_location_.x - current_pose_.position.x);

    move_base_msgs::MoveBaseGoal move_base_goal;
    ac.waitForServer();
    setPoseGoal(move_base_goal, base_location_.x, base_location_.y, goal_yaw);
    ROS_INFO_STREAM("goal pose after SetposeGOAL: " << move_base_goal);
    ac.sendGoal(move_base_goal);
    ac.waitForResult(ros::Duration(0.25));
  }

  // Homing - Measurement Update
  sensor_fusion::HomingUpdate srv_homing;
  srv_homing.request.initializeLandmark = false;
  if (clt_homing_.call(srv_homing))
  {
    // ROS_INFO_STREAM("Success? "<< srv_upd_pose.response.success);
    flag_localization_failure=false;
    flag_arrived_at_waypoint = true;
    flag_completed_homing = true;
  }
  else
  {
    ROS_ERROR("Failed to call service LocationOfBase");
  }

  // Turn on the Lights
  srv_lights.request.data  = "0.2";
  if (clt_lights_.call(srv_lights))
  {
    // ROS_INFO_STREAM("Success? "<< srv_lights.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service ToggleLight");
  }

  std_srvs::Empty emptymsg;
  ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(2.0));
  if (ros::service::call("/move_base/clear_costmaps",emptymsg))
  {
     ROS_INFO("every costmap layers are cleared except static layer");
  }
  else
  {
     ROS_INFO("failed calling clear_costmaps service");
  }

  driving_tools::RotateInPlace srv_turn;

  srv_turn.request.throttle  = 0.2;
   ros::Duration(1.0).sleep();
  if (clt_rip_.call(srv_turn))
  {
          ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
          ros::Duration(5.0).sleep();
  }
  else
  {
          ROS_ERROR("Failed to call service Stop");
  }
  // Break Rover
  // driving_tools::Stop srv_stop;
  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  driving_tools::MoveForward srv_drive;
  srv_drive.request.throttle  = 0.3;
  if (clt_drive_.call(srv_drive))
  {
          ros::Duration(3.0).sleep();
          ROS_INFO_STREAM("SM: Drive Enabled? "<< srv_drive.response);
  }
  else
  {
          ROS_ERROR("Failed to call service Drive");
  }

  srv_stop.request.enableStop  = true;
  if (clt_stop_.call(srv_stop))
  {
    // ROS_INFO_STREAM("Success? "<< srv_stop.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");

 }
    // make sure we dont latch to a vol we skipped while homing
    volatile_detected_distance = -1.0;


  //flag_completed_homing = true;
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
    ROS_WARN_ONCE("Initial Localization Successful = %i",flag_localized_base);

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

void SmRd1::volatileDetectedCallback(const std_msgs::Float32::ConstPtr& msg)
{
  if (!timer_counter || ros::Time::now().toSec() - detection_timer.toSec() > TIMER_THRESH)
  {
          ROS_INFO("Setting Vol Distance Callback %f",msg->data);
          prev_volatile_detected_distance = volatile_detected_distance;
          volatile_detected_distance = msg->data;


          if (volatile_detected_distance > 0 && volatile_detected_distance < min_volatile_detected_distance)
          {
                  min_volatile_detected_distance = volatile_detected_distance;
          }

  }
  else{
    volatile_detected_distance = -1;
  }

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

void SmRd1::localizationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_ = msg->pose.pose;

  yaw_prev = yaw;
  tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
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

void SmRd1::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
{
    actionDone_ = true;
    ROS_INFO("goal done");
}
void SmRd1::activeCallback()
{
    ROS_INFO("goal went active");
}
void SmRd1::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("got feedback");
}

void SmRd1::rotateToHeading(double desired_yaw)
{
  ros::Rate rateRotateToHeading(20);

  driving_tools::RotateInPlace srv_turn;
  srv_turn.request.throttle  = 0.2;

  if (clt_rip_.call(srv_turn))
  {
    ROS_INFO_STREAM("SM: Rotating Enabled? "<< srv_turn.response);
  }
  else
  {
    ROS_ERROR("Failed to call service Stop");
  }

  double yaw_thres = 0.01;
  while(fabs(yaw-desired_yaw) < yaw_thres)
  {
    rateRotateToHeading.sleep();
    ros::spinOnce();
    ROS_INFO("SM: Trying to control yaw.");
  }

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
}





//------------------------------------------------------------------------------------------------------------------------
