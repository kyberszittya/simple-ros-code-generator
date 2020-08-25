package hu.sze.aut.code.generator.ros

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.TopicMessage
import java.util.HashSet
import java.util.Set
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.InputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.OutputPort

class CodeGenerationUtils {
	def static selectAllMessages(Node node)
	{
		val Set<TopicMessage> messages = new HashSet
		for (port: node.port.filter[it instanceof InputPort])
		{
			messages.add(((port as InputPort).channel as Topic).type)
		}
		for (port: node.port.filter[it instanceof OutputPort])
		{
			messages.add(((port as OutputPort).channel as Topic).type)
		}
		return messages
	}
	
	def static commentNodeHeader(Node node)'''
	/**
	 *
	 «FOR port: node.port.filter[it instanceof InputPort]»
	 * @attribute «port.id»: subscribes to topic «((port as InputPort).channel as Topic).name» 
	 «ENDFOR»
	 «FOR port: node.port.filter[it instanceof OutputPort]»
	 * @attribute «port.id»: publishes to topic «((port as OutputPort).channel as Topic).name»
	 «ENDFOR»
	 */
	'''
	
	def static generateAbstractMethods(Node node)'''
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
	'''
}