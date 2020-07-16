package hu.sze.aut.code.generator.ros2

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.aut.code.generator.ros.CodeGenerationUtils
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.ContinousState

class CodeTemplatePython {
	
	def static generatePythonClassName(String name)'''«name.split('_').join»'''
	
	def static generatePythonContinuousStateName(ContinousState state)'''«generatePythonClassName(state.name.toFirstUpper)»'''
	
	def static generatePythonNode(Node node)'''
	import rclpy
	from rclpy.node import Node
	
	«FOR msg: CodeGenerationUtils::selectAllMessages(node)»
	from «msg.prefix».msg import «msg.name»
	«ENDFOR»
	
	«FOR s: node.continousstate»
	class «generatePythonContinuousStateName(s)»(object):
		«FOR m: s.receivesFrom»
		msg_«m.id» = None
		«ENDFOR»
		«FOR m: s.producesTo»
		msg_«m.id» = None
		«ENDFOR»
	«ENDFOR»
	
	
	class «generatePythonClassName('''AbstractGen«node.name.toFirstUpper»''')»(Node):
		
		def __init__(self):
			super().__init__("«node.name.toLowerCase»")
			«FOR st: node.continousstate»
			self.«st.name» = «generatePythonContinuousStateName(st)»()
			«ENDFOR»
			
		def init(self):
			«FOR pub: node.outputport.filter[it.channel instanceof Topic]»
			self.«pub.id» = self.create_publisher(«(pub.channel as Topic).type.name», "«pub.channel.name»", 10)
			«ENDFOR»
			«FOR sub: node.inputport»
			self.«sub.id» = self.create_subscription(«(sub.channel as Topic).type.name», "«sub.channel.name»", self.cb«sub.id.toFirstUpper», 10)
			«ENDFOR»
		
		«FOR sub: node.inputport»
		«IF sub.sync_function_name!==null»
		def «sub.sync_function_name»(self, msg):
			raise NotImplementedError
		«ENDIF»
		
		def cb«sub.id.toFirstUpper»(self, msg):
			«IF sub.continousState!==null»
			self.«sub.continousState.name».msg_«sub.id» = msg
			«ENDIF»
			«IF sub.sync_function_name.length>0»
			self.«sub.sync_function_name»(msg)
			«ENDIF»
		«ENDFOR»
	'''
}