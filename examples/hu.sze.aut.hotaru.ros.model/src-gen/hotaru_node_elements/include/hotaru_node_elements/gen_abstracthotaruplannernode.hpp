#ifndef ABSTRACTHOTARUPLANNERNODE_HEADER_HPP
#define ABSTRACTHOTARUPLANNERNODE_HEADER_HPP

#include <ros/ros.h>
/// ROS msgs
#include <hotaru_msgs/RefinedTrajectory.h>		
#include <geometry_msgs/TwistStamped.h>		
#include <rei_planner_signals/ReplanRequest.h>		
#include <geometry_msgs/PoseStamped.h>		
#include <rei_perception_msgs/DetectedObjects.h>		
#include <std_msgs/Float64.h>		
#include <std_msgs/Int32.h>		
#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>
// State-machine node element
#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>

#include <memory>

namespace hotaru {

struct StateAbstractHotaruPlannerNode
{
	// Flags
	const bool debug;				///< Publish debug parameters
	const bool bypass_behavior;     ///< Bypass behavioral state machines
	// ROS input messages
	hotaru_msgs::RefinedTrajectory msg_port_input_trajectory; ///< port_input_trajectory store to hotaru_msgs::RefinedTrajectory
	geometry_msgs::PoseStamped msg_port_current_pose; ///< port_current_pose store to geometry_msgs::PoseStamped
	geometry_msgs::TwistStamped msg_port_current_velocity; ///< port_current_velocity store to geometry_msgs::TwistStamped
	rei_planner_signals::ReplanRequest msg_port_replan_request_sig; ///< port_replan_request_sig store to rei_planner_signals::ReplanRequest
	rei_perception_msgs::DetectedObjects msg_port_detected_obstacles; ///< port_detected_obstacles store to rei_perception_msgs::DetectedObjects
	std_msgs::Int32 msg_port_closest_waypoint; ///< port_closest_waypoint store to std_msgs::Int32
	/// ROS output messages
	hotaru_msgs::RefinedTrajectory msg_port_refined_trajectory; ///< port_refined_trajectory store to hotaru_msgs::RefinedTrajectory
	std_msgs::Float64 msg_port_calc_planner_time; ///< port_calc_planner_time store to std_msgs::Float64
	
	StateAbstractHotaruPlannerNode(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
};

/**
 *
 * @attribute port_input_trajectory: subscribes to topic input_trajectory 
 * @attribute port_current_pose: subscribes to topic current_pose 
 * @attribute port_current_velocity: subscribes to topic current_velocity 
 * @attribute port_replan_request_sig: subscribes to topic replan_request_sig 
 * @attribute port_detected_obstacles: subscribes to topic rei_perception_monitor/detected_obstacles 
 * @attribute port_closest_waypoint: subscribes to topic closest_waypoint 
 * @attribute port_refined_trajectory: publishes to topic refined_trajectory
 * @attribute port_calc_planner_time: publishes to topic calc_planner_time
 */
class InterfaceRos_AbstractHotaruPlannerNode: public rei::Interface_SimpleRosNode
{
private:
protected:
	/// ROS utils
	std::shared_ptr<ros::NodeHandle> private_nh;
	std::shared_ptr<ros::NodeHandle> nh;
	/// ROS Subscribers
	ros::Subscriber port_input_trajectory; ///< port_input_trajectory subscriber to hotaru_msgs::RefinedTrajectory
	ros::Subscriber port_current_pose; ///< port_current_pose subscriber to geometry_msgs::PoseStamped
	ros::Subscriber port_current_velocity; ///< port_current_velocity subscriber to geometry_msgs::TwistStamped
	ros::Subscriber port_replan_request_sig; ///< port_replan_request_sig subscriber to rei_planner_signals::ReplanRequest
	ros::Subscriber port_detected_obstacles; ///< port_detected_obstacles subscriber to rei_perception_msgs::DetectedObjects
	ros::Subscriber port_closest_waypoint; ///< port_closest_waypoint subscriber to std_msgs::Int32
	/// ROS Publishers
	ros::Publisher port_refined_trajectory; ///< port_refined_trajectory publisher to hotaru_msgs::RefinedTrajectory
	ros::Publisher port_calc_planner_time; ///< port_calc_planner_time publisher to std_msgs::Float64
	std::unique_ptr<StateAbstractHotaruPlannerNode> pubsubstate;
	// State machines
	std::shared_ptr<rei::RosSyncStateMachine> sync_sm_pubsubstate;
	std::shared_ptr<rei::SyncStateMachine> sync_state_machine;
	std::shared_ptr<rei::PortStateMonitorRos> port_state_monitor;
	std::shared_ptr<rei::RosCommunicationGraphNotifier> notifier;
	std::shared_ptr<rei::RosSyncStateGuard> sync_guard;
	std::mutex sm_mutex;
	// Set ALL STATES CB
	virtual void setSyncStateMachineCallbacks() = 0;
public:
	InterfaceRos_AbstractHotaruPlannerNode(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
	
	virtual ~InterfaceRos_AbstractHotaruPlannerNode() = 0;
	
	/*
	 * Override steps to initialize
	 *     STEPS:
	 *           1. Initialize descendant specific functionalities before middleware functionalities (initPre())
	 *           2. Initialize timeout state machine (initTimeoutStateMachine())
	 *           3. Assign guard related to timeout functions (assigSyncGuards())
	 *           4. Initialize middleware functionalities
	 *           5. Initialize descendant node-specific functionalities
	 */
	/*
	 * @brief: Initialize node pre
	 * @returns: Initialization successful
	 */
	virtual bool initPre() = 0;
	/*
	 * @brief: Initialize timeout statemachine
	 */
	virtual bool initTimeoutStateMachine() override;
	/*
	 * @brief: Assign sync guards
	 */
	virtual bool assignSyncGuards() = 0;
	
	/*
	 * @brief: initialize middleware
	 * @param debug: defines whether the debug information should be provided or not.
	 */
	virtual bool initMiddleware(const bool debug, const bool bypass_behavior) override;
	
	/*
	 * @brief: post initialize
	 */
	virtual bool initPost() = 0;
	
	/**
	 * Callback method for input_trajectory
	 */
	void cbPort_input_trajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg); ///< port_input_trajectory subscriber to hotaru_msgs::RefinedTrajectory
	virtual void executeSyncinputtrajectory(const hotaru_msgs::RefinedTrajectory::ConstPtr& msg) = 0;
	/**
	 * Callback method for current_pose
	 */
	void cbPort_current_pose(const geometry_msgs::PoseStamped::ConstPtr& msg); ///< port_current_pose subscriber to geometry_msgs::PoseStamped
	virtual void executeSynccurrentpose(const geometry_msgs::PoseStamped::ConstPtr& msg) = 0;
	/**
	 * Callback method for current_velocity
	 */
	void cbPort_current_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg); ///< port_current_velocity subscriber to geometry_msgs::TwistStamped
	virtual void executeSynccurrentvelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) = 0;
	/**
	 * Callback method for replan_request_sig
	 */
	void cbPort_replan_request_sig(const rei_planner_signals::ReplanRequest::ConstPtr& msg); ///< port_replan_request_sig subscriber to rei_planner_signals::ReplanRequest
	virtual void executeReplanrequestsig(const rei_planner_signals::ReplanRequest::ConstPtr& msg) = 0;
	/**
	 * Callback method for rei_perception_monitor/detected_obstacles
	 */
	void cbPort_detected_obstacles(const rei_perception_msgs::DetectedObjects::ConstPtr& msg); ///< port_detected_obstacles subscriber to rei_perception_msgs::DetectedObjects
	virtual void executeDetectedobstacles(const rei_perception_msgs::DetectedObjects::ConstPtr& msg) = 0;
	/**
	 * Callback method for closest_waypoint
	 */
	void cbPort_closest_waypoint(const std_msgs::Int32::ConstPtr& msg); ///< port_closest_waypoint subscriber to std_msgs::Int32
	virtual void executeClosestwaypoint(const std_msgs::Int32::ConstPtr& msg) = 0;
	
	/**
	 * Publish method to publish message to refined_trajectory
	 */
	void publishRefined_trajectory();
	/**
	 * Publish method to publish message to calc_planner_time
	 */
	void publishCalc_planner_time();
};

}

#endif
