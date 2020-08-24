package hu.sze.aut.code.generator.ros2

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.TopicMessage
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.InputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.ContinousState
import hu.sze.aut.code.generator.ros.CodeGenerationUtils
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic

public class CodeTemplateCppRos2 {
	
	def static interfaceName(Node node)'''Interface_«node.name.toFirstUpper»'''
	
	def static generateMsgType(TopicMessage type)'''«type.prefix»::msg::«type.name»'''
	
	def static generateCallbackName(InputPort port)'''cb«port.id»'''
	
	def static generateContinuousStateName(ContinousState s)'''ContinuousState«s.name.toFirstUpper»'''
	
	def static generateRos2InterfaceHeader(Node node)'''
	#include <chrono>
	#include <functional>
	#include <memory>
	#include <string>
	
	// REI
	#include <rei_common/gen_node_elements/interface_simple_ros_node.hpp>
	
	#include <rclcpp/rclcpp.hpp>
	
	«IF node.namespace!==null»
	namespace «node.namespace» 
	{
	«ENDIF»
	
	«FOR s: node.continousstate»
	struct «generateContinuousStateName(s)»
	{
	  const bool debug;
	  const bool bypass_behavior;
	  // Input messages
	  «FOR port: s.receivesFrom.filter[it.channel instanceof Topic]»
	  «generateMsgType((port.channel as Topic).type)» msg_«port.id.toLowerCase»;
	  «ENDFOR»
	  // Output messages
	  «FOR port: s.producesTo.filter[it.channel instanceof Topic]»
	  «generateMsgType((port.channel as Topic).type)» msg_«port.id.toLowerCase»;
	  «ENDFOR»
	}
	«ENDFOR»
	
	«FOR m: CodeGenerationUtils::selectAllMessages(node)»
	#include <«m.prefix»/«m.name»>
	«ENDFOR»	
	
	class «interfaceName(node)»: public rclcpp::Node, public rei::Interface_SimpleRos2Node
	{
	private:
	  «FOR s: node.continousstate»
	  std::unique_ptr<«generateContinuousStateName(s)»> _«s.name»;
	  «ENDFOR»
	  «FOR port: node.inputport.filter[it.channel instanceof Topic]»
	  ros::Subscription<«generateMsgType((port.channel as Topic).type)»>::SharedPtr «port.id»;
	  «ENDFOR»
	  «FOR port: node.outputport.filter[it.channel instanceof Topic]»
	  ros::Publisher<«generateMsgType((port.channel as Topic).type)»>::SharedPtr «port.id»;
	  «ENDFOR»
	  std::mutex sm_mutex;
	public:
	  Interface_«node.name.toFirstUpper»(std::string node_name);
	  virtual ~Interface_«node.name.toFirstUpper»() = 0;
	
	  «CodeGenerationUtils::generateAbstractMethods(node)»
	  
	  virtual bool initMiddleware(const bool debug, const bool bypass_behavior);
	  
	  // Subscriber callbacks
	  «FOR port: node.inputport.filter[it.channel instanceof Topic]»
	  void cb«port.id.toFirstUpper»(const «generateMsgType((port.channel as Topic).type)»::SharedPtr msg);
	  «IF port.sync_function_name!==null»
	  virtual void «port.sync_function_name»(const «generateMsgType((port.channel as Topic).type)»::SharedPtr msg) = 0;
	  «ENDIF»
	  «ENDFOR»
	  // Publisher functions
	  «FOR port: node.outputport»
	  void publish«port.id.toFirstUpper»();
	  «ENDFOR»
	};
	
	«IF node.namespace!==null»
	}
	«ENDIF»
	'''	
	
	def static generateRos2Source(Node node)'''
	#include "«node.rospackage»/gen_«node.name.toLowerCase».hpp"
	
	bool «interfaceName(node)»::initTimeoutStateMachine()
	{
	  return true;
	}
	
	bool «interfaceName(node)»::initMiddleware(const bool debug, const bool bypass_behavior)
	{
	  // Initialize continuous state
	  «FOR state: node.continousstate»
	  _«state.name» = std::make_unique<«generateContinuousStateName(state)»>(debug, bypass_behavior);
	  «ENDFOR»
	  // Initialize ROS 2 publishers
	  «FOR port: node.outputport.filter[it.channel instanceof Topic]»
	  «port.id» = create_publisher<«generateMsgType((port.channel as Topic).type)»>("«port.channel.name»", 10);
	  «ENDFOR»
	  // Initialize ROS 2 subscribers
	  «FOR port: node.inputport.filter[it.channel instanceof Topic]»
	  «port.id» = create_subscription<«generateMsgType((port.channel as Topic).type)»>("«port.channel.name»", 10, std::bind(&«interfaceName(node)»::«generateCallbackName(port)», this, _1));
	  «ENDFOR»
	}
	
	«FOR port: node.inputport.filter[it.channel instanceof Topic]»	
	void «interfaceName(node)»::«generateCallbackName(port)»(const «generateMsgType((port.channel as Topic).type)»::SharedPtr msg)
	{
	  «IF port.continousState!==null»
	  _«port.continousState.name»->msg_«port.id.toLowerCase» = msg;
	  «ENDIF»
	  «IF port.sync_function_name!==null»
	  «port.sync_function_name»(msg);
	  «ENDIF»
	}
	«ENDFOR»
	
	//
	// Publisher functions
	«FOR port: node.outputport»
	void publish«port.id.toFirstUpper»()
	{
	  «IF port.continousState!==null»
	  «port.id»->publish(«port.continousState.name»->msg_«port.id.toLowerCase»);
	  «ENDIF»
	}
	«ENDFOR»
	'''
}