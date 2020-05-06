package hu.sze.aut.code.generator.ros

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic
import java.util.Map
import java.util.List
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.OutputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.InputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.TopicMessage

class RosCodeTemplates {
	def static classNameInterfaceRos(Node node)'''InterfaceRos_«node.name.toFirstUpper.replace('_','')»'''
	def static headerFileName(Node node)'''interfaceros_«node.name.toLowerCase».h'''
	def static convertMsgDefToCppNamespace(String type){
		return type.replace('/', "::")
	}
	
	def static getMsgInclude(TopicMessage msg)'''#include <«msg.prefix»/«msg.name».h>'''
	def static getMsgNamespace(TopicMessage msg)'''«msg.prefix»::«msg.name»'''
	
	
	
	def static generateRosInterfaceHeader(Node node)'''
		#ifndef «node.name.toUpperCase»_HEADER_HPP
		#define «node.name.toUpperCase»_HEADER_HPP
		
		#include <ros/ros.h>
		/// ROS msgs
		«FOR m: CodeGenerationUtils::selectAllMessages(node)»
		«getMsgInclude(m)»		
		«ENDFOR»
		«IF node.continousstate.size > 0»
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
			«getMsgNamespace(port.topic.type)» msg_«port.id.toFirstLower»; ///< «port.id» store to «getMsgNamespace(port.topic.type)»
			«ENDFOR»
			/// ROS output messages
			«FOR port: node.outputport»
			«getMsgNamespace(port.topic.type)» msg_«port.id.toFirstLower»; ///< «port.id» store to «getMsgNamespace(port.topic.type)»
			«ENDFOR»
			
			State«node.name.toFirstUpper.replace('_', '')»(const bool debug, const bool bypass_behavior): debug(debug), bypass_behavior(bypass_behavior) {}
		};
		
		«CodeGenerationUtils::commentNodeHeader(node)»
		class «classNameInterfaceRos(node)»: public rei::Interface_SimpleRosNode
		{
		private:
		protected:
			/// ROS utils
			std::shared_ptr<ros::NodeHandle> private_nh;
			std::shared_ptr<ros::NodeHandle> nh;
			/// ROS Subscribers
			«FOR port: node.inputport»
			ros::Subscriber «port.id.toFirstLower»; ///< «port.id» subscriber to «getMsgNamespace(port.topic.type)»
			«ENDFOR»
			/// ROS Publishers
			«FOR port: node.outputport»
			ros::Publisher «port.id.toFirstLower»; ///< «port.id» publisher to «getMsgNamespace(port.topic.type)»
			«ENDFOR»
			std::unique_ptr<State«node.name.toFirstUpper.replace('_', '')»> pubsubstate;
			// State machines
			«FOR state: node.continousstate»
			std::shared_ptr<rei::RosSyncStateMachine> sync_sm_«state.name.toLowerCase»;
			std::shared_ptr<rei::SyncStateMachine> sync_state_machine;
			std::shared_ptr<rei::PortStateMonitorRos> port_state_monitor;
			std::shared_ptr<rei::RosCommunicationGraphNotifier> notifier;
			std::shared_ptr<rei::RosSyncStateGuard> sync_guard;
			std::mutex sm_mutex;
			«ENDFOR»
			// Set ALL STATES CB
			virtual void setSyncStateMachineCallbacks() = 0;
		public:
			«classNameInterfaceRos(node)»(std::shared_ptr<ros::NodeHandle> private_nh, std::shared_ptr<ros::NodeHandle> nh): private_nh(private_nh), nh(nh) {}
			
			virtual ~«classNameInterfaceRos(node)»() = 0;
			
			«CodeGenerationUtils::generateAbstractMethods(node)»
			
			«FOR port: node.inputport»
			/**
			 * Callback method for «port.topic.name»
			 */
			void cb«port.id.toFirstUpper»(const «getMsgNamespace(port.topic.type)»::ConstPtr& msg); ///< «port.id» subscriber to «getMsgNamespace(port.topic.type)»
			«IF port.sync_function_name!==null»
			virtual void execute«port.sync_function_name.toFirstUpper»(const «getMsgNamespace(port.topic.type)»::ConstPtr& msg) = 0;
			«ENDIF»
			«ENDFOR»
			
			«FOR port: node.outputport»
			/**
			 * Publish method to publish message to «port.topic.name»
			 */
			void publish«port.topic.name.toFirstUpper»();
			«ENDFOR»
		};
		
		«IF node.namespace!==null»
		}
		«ENDIF»
		
		#endif
		'''
		
	def static generateInterfaceRosSource(Node node)'''
		#include <«node.rospackage»/gen_«headerFileName(node)»>
		
		«IF node.namespace!==null»
		namespace «node.namespace» {
		«ENDIF»
		
		«classNameInterfaceRos(node)»::~«classNameInterfaceRos(node)»() {}
		
		bool «classNameInterfaceRos(node)»::initTimeoutStateMachine()
		{
			// Initialize state machines 
			notifier = std::make_shared<rei::RosCommunicationGraphNotifier>("«node.name.toLowerCase»/sync_state/", nh);
			notifier->initialize();
			port_state_monitor= std::make_shared<rei::PortStateMonitorRos>();
			sync_guard = std::make_shared<rei::RosSyncStateGuard>();
			sync_guard->setMonitor(port_state_monitor);
			sync_state_machine = std::make_shared<rei::SyncStateMachine>(notifier, sync_guard);
			// Sync state machine initialization
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
				sync_sm_«state.name.toLowerCase»->addTopicGuard("/«portstate.topic.name»", «1.0/(portstate.estimated_freq as double)»+«portstate.sample_tolerance»);
				ROS_INFO("Synchronize with topic: «portstate.topic.name», with estimated freq «portstate.estimated_freq» Hz");
				«ENDFOR»
			}
			else
			{
				return false;
			}
			«ENDFOR»
			return true;
		}
		
		bool «classNameInterfaceRos(node)»::initMiddleware(const bool debug, const bool bypass_behavior)
		{
			/// Initialize internal pubsub state
			pubsubstate = std::unique_ptr<State«node.name.toFirstUpper.replace('_', '')»>(new State«node.name.toFirstUpper.replace('_', '')»(debug, bypass_behavior));
			if (pubsubstate==nullptr)
			{
				return false;
			}
			/// Initialize ROS publishers
			«FOR port: node.outputport»
			«IF port.debug»
			if (pubsubstate->debug) «port.id.toFirstLower» = nh->advertise<«getMsgNamespace(port.topic.type)»>("/debug/«node.name.toLowerCase»/«port.topic.name»", 10);
			«ELSE»
			«port.id.toFirstLower» = nh->advertise<«getMsgNamespace(port.topic.type)»>("«port.topic.name»", 10);
			«ENDIF»
			«ENDFOR»
			/// Initialize ROS subscribers
			«FOR port: node.inputport»
			«IF port.debug»
			if (pubsubstate->debug) «port.id.toFirstLower» = nh->subscribe("/debug/«node.name.toLowerCase»/«port.topic.name»", 10, &«classNameInterfaceRos(node)»::cb«port.id.toFirstUpper», this);
			«ELSE»
			«port.id.toFirstLower» = nh->subscribe("«port.topic.name»", 10, &«classNameInterfaceRos(node)»::cb«port.id.toFirstUpper», this);
			«ENDIF»
			«ENDFOR»
			return true;
		}
		
		
		
		«FOR port: node.inputport»
		void «classNameInterfaceRos(node)»::cb«port.id.toFirstUpper»(const «getMsgNamespace(port.topic.type)»::ConstPtr& msg)
		{
			pubsubstate->msg_«port.id.toFirstLower» = *msg;
			«IF port.synchronizesState!==null»
			// Synchronize with state machine: sync_sm_«port.synchronizesState.name»
			sm_mutex.lock();
			«IF port.sync_time_stamp»
			sync_sm_«port.synchronizesState.name»->stepMessageTopic("/«port.topic.name»", msg->header);
			«ELSE»
			sync_sm_«port.synchronizesState.name»->stepMessageTopic("/«port.topic.name»", ros::Time::now());
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
		void «classNameInterfaceRos(node)»::publish«port.topic.name.toFirstUpper»()
		{
			«port.id».publish(pubsubstate->msg_«port.id»);
		}
		«ENDFOR»
		
		«IF node.namespace!==null»
		}
		«ENDIF»
		'''
}