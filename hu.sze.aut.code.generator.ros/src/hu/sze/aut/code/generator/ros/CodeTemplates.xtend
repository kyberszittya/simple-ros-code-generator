package hu.sze.aut.code.generator.ros

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic
import java.util.Map
import java.util.List
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.OutputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.InputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.TopicMessage
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Service

class RosCodeTemplates {
	def static classNameInterfaceRos(Node node)'''InterfaceRos_«node.name.toFirstUpper.replace('_','')»'''
	def static headerFileName(Node node)'''gen_«node.name.toLowerCase».hpp'''
	def static convertMsgDefToCppNamespace(String type){
		return type.replace('/', "::")
	}
	
	def static getMsgInclude(TopicMessage msg)'''#include <«msg.prefix»/«msg.name».h>'''
	def static getMsgNamespace(TopicMessage msg)'''«msg.prefix»::«msg.name»'''
	
	def static genPublisherFunctionName(OutputPort port)'''publish«(port.channel as Topic).name.toFirstUpper.replaceAll('/','_')»'''
	
	def static generateRosInterfaceHeader(Node node, boolean synch)'''
		#ifndef «node.name.toUpperCase»_HEADER_HPP
		#define «node.name.toUpperCase»_HEADER_HPP
		
		#include <ros/ros.h>
		/// ROS msgs
		«FOR m: CodeGenerationUtils::selectAllMessages(node)»
		«getMsgInclude(m)»
		«ENDFOR»
		«IF node.continousstate.size > 0 && synch»
		#include <rei_statemachine_library/ros/ros_sync_state_machine.hpp>
		«ENDIF»
		// State-machine node element
		#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>
		
		#include <memory>
		
		«IF node.namespace!==null»
		namespace «node.namespace» {
		«ENDIF»
		
		struct State«node.name.toFirstUpper.replace('_', '')»
		{
			// Flags
			const bool debug;				///< Publish debug parameters
			const bool bypass_behavior;     ///< Bypass behavioral state machines
			// ROS input messages
			«FOR port: node.inputport»
			«IF port.channel instanceof Topic»
			«getMsgNamespace((port.channel as Topic).type)» msg_«port.id.toFirstLower»; ///< «port.id» store to «getMsgNamespace((port.channel as Topic).type)»
			«ELSEIF port.channel instanceof Service»
			// TODO
			«ENDIF»
			«ENDFOR»
			/// ROS output messages
			«FOR port: node.outputport»
			«getMsgNamespace((port.channel as Topic).type)» msg_«port.id.toFirstLower»; ///< «port.id» store to «getMsgNamespace((port.channel as Topic).type)»
			«ENDFOR»
			
			State«node.name.toFirstUpper.replace('_', '')»(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
		};
		
		«CodeGenerationUtils::commentNodeHeader(node)»
		class «classNameInterfaceRos(node)»: public «IF node.implements==null»rei::Interface_SimpleRosNode«ELSE»«node.implements.namespace»::«classNameInterfaceRos(node.implements)»«ENDIF»
		{
		private:
		protected:
			/// ROS utils
			std::shared_ptr<ros::NodeHandle> private_nh;
			std::shared_ptr<ros::NodeHandle> nh;
			/// ROS Subscribers
			«FOR port: node.inputport»
			ros::Subscriber «port.id.toFirstLower»; ///< «port.id» subscriber to «getMsgNamespace((port.channel as Topic).type)»
			«ENDFOR»
			/// ROS Publishers
			«FOR port: node.outputport»
			ros::Publisher «port.id.toFirstLower»; ///< «port.id» publisher to «getMsgNamespace((port.channel as Topic).type)»
			«ENDFOR»
			std::unique_ptr<State«node.name.toFirstUpper.replace('_', '')»> pubsubstate;
			// State machines
			«FOR state: node.continousstate»
			«IF synch»
			std::shared_ptr<rei::RosSyncStateMachine> sync_sm_«state.name.toLowerCase»;
			std::shared_ptr<rei::SyncStateMachine> sync_state_machine;
			std::shared_ptr<rei::PortStateMonitorRos> port_state_monitor;
			std::shared_ptr<rei::RosCommunicationGraphNotifier> notifier;
			std::shared_ptr<rei::RosSyncStateGuard> sync_guard;
			std::mutex sm_mutex;
			«ENDIF»
			«ENDFOR»
			«IF synch»
			// Set ALL STATES CB
			virtual void setSyncStateMachineCallbacks() = 0;
			«ENDIF»
		public:
			«classNameInterfaceRos(node)»(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
			
			virtual ~«classNameInterfaceRos(node)»() = 0;
			
			«CodeGenerationUtils::generateAbstractMethods(node)»
			
			«FOR port: node.inputport»
			/**
			 * Callback method for «(port.channel as Topic).name»
			 */
			void cb«port.id.toFirstUpper»(const «getMsgNamespace((port.channel as Topic).type)»::ConstPtr& msg); ///< «port.id» subscriber to «getMsgNamespace((port.channel as Topic).type)»
			«IF port.sync_function_name!==null»
			virtual void execute«port.sync_function_name.toFirstUpper»(const «getMsgNamespace((port.channel as Topic).type)»::ConstPtr& msg) = 0;
			«ENDIF»
			«ENDFOR»
			
			«FOR port: node.outputport»
			/**
			 * Publish method to publish message to «(port.channel as Topic).name»
			 */
			void «genPublisherFunctionName(port)»();
			«ENDFOR»
		};
		
		«IF node.namespace!==null»
		}
		«ENDIF»
		
		#endif
		'''
		
	def static generateInterfaceRosSource(Node node, boolean synch)'''
		#include <«node.rospackage»/«headerFileName(node)»>
		
		«IF node.namespace!==null»
		namespace «node.namespace» {
		«ENDIF»
		
		«classNameInterfaceRos(node)»::~«classNameInterfaceRos(node)»() {}
		
		bool «classNameInterfaceRos(node)»::initTimeoutStateMachine()
		{
			«IF synch»
			// Initialize state machines 
			notifier = std::make_shared<rei::RosCommunicationGraphNotifier>("«node.name.toLowerCase»/sync_state/", nh);
			notifier->initialize();
			port_state_monitor= std::make_shared<rei::PortStateMonitorRos>();
			sync_guard = std::make_shared<rei::RosSyncStateGuard>();
			sync_guard->setMonitor(port_state_monitor);
			sync_state_machine = std::make_shared<rei::SyncStateMachine>(notifier, sync_guard);
			«ENDIF»
			// Sync state machine initialization
			«IF synch»
			«FOR state: node.continousstate»
			sync_sm_«state.name.toLowerCase» = std::make_shared<rei::RosSyncStateMachine>(nh,
				sync_state_machine, port_state_monitor, notifier, 
				"«node.name»/«state.name.toLowerCase»");
			if (sync_sm_«state.name.toLowerCase»!=nullptr)
			{
				if (!sync_sm_«state.name.toLowerCase»->initialize())
				{
					return false;
				}
				«FOR portstate: state.synchronizeWith»
				sync_sm_«state.name.toLowerCase»->addTopicGuard("/«(portstate.channel as Topic).name»", «1.0/(portstate.estimated_freq as double)»+«portstate.sample_tolerance»);
				ROS_INFO("Synchronize with topic: «(portstate.channel as Topic).name», with estimated freq «portstate.estimated_freq» Hz");
				«ENDFOR»
			}
			else
			{
				return false;
			}
			«ENDFOR»
			«ENDIF»
			return true;
		}
		
		bool «classNameInterfaceRos(node)»::initMiddleware(const bool debug, const bool bypass_behavior)
		{
			«IF node.implements!=null»
			/// Call superinterface
			«classNameInterfaceRos(node)»::initMiddleware(debug, bypass_behavior);
			«ENDIF»
			/// Initialize internal pubsub state
			pubsubstate = std::unique_ptr<State«node.name.toFirstUpper.replace('_', '')»>(new State«node.name.toFirstUpper.replace('_', '')»(debug, bypass_behavior));
			if (pubsubstate==nullptr)
			{
				return false;
			}
			/// Initialize ROS publishers
			«FOR port: node.outputport»
			«IF port.debug»
			if (pubsubstate->debug) «port.id.toFirstLower» = nh->advertise<«getMsgNamespace((port.channel as Topic).type)»>("/debug/«node.name.toLowerCase»/«(port.channel as Topic).name»", 10);
			«ELSE»
			«port.id.toFirstLower» = nh->advertise<«getMsgNamespace((port.channel as Topic).type)»>("«(port.channel as Topic).name»", 10);
			«ENDIF»
			«ENDFOR»
			/// Initialize ROS subscribers
			«FOR port: node.inputport»
			«IF port.debug»
			if (pubsubstate->debug) «port.id.toFirstLower» = nh->subscribe("/debug/«node.name.toLowerCase»/«(port.channel as Topic).name»", 10, &«classNameInterfaceRos(node)»::cb«port.id.toFirstUpper», this);
			«ELSE»
			«port.id.toFirstLower» = nh->subscribe("«(port.channel as Topic).name»", 10, &«classNameInterfaceRos(node)»::cb«port.id.toFirstUpper», this);
			«ENDIF»
			«ENDFOR»
			return true;
		}
		
		
		
		«FOR port: node.inputport»
		void «classNameInterfaceRos(node)»::cb«port.id.toFirstUpper»(const «getMsgNamespace((port.channel as Topic).type)»::ConstPtr& msg)
		{
			pubsubstate->msg_«port.id.toFirstLower» = *msg;
			«IF port.synchronizesState!==null»
			// Synchronize with state machine: sync_sm_«port.synchronizesState.name»
			sm_mutex.lock();
			«IF port.sync_time_stamp»
			sync_sm_«port.synchronizesState.name»->stepMessageTopic("/«(port.channel as Topic).name»", msg->header);
			«ELSE»
			sync_sm_«port.synchronizesState.name»->stepMessageTopic("/«(port.channel as Topic).name»", ros::Time::now());
			«ENDIF»
			sm_mutex.unlock();
			«IF port.sync_function_name!==null»
			if (sync_sm_«port.synchronizesState.name»->isReady()){
				execute«port.sync_function_name.toFirstUpper»(msg);
			}
			«ENDIF»
			«ELSE»
			«IF port.sync_function_name!==null»
			execute«port.sync_function_name.toFirstUpper»(msg);
			«ENDIF»
			«ENDIF»
			
		}
		«ENDFOR»
		
		«FOR port: node.outputport»
		void «classNameInterfaceRos(node)»::«genPublisherFunctionName(port)»()
		{
			«port.id».publish(pubsubstate->msg_«port.id»);
		}
		«ENDFOR»
		
		«IF node.namespace!==null»
		}
		«ENDIF»
		'''
}