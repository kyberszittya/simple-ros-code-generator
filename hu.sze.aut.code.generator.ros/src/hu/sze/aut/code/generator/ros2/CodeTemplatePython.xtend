package hu.sze.aut.code.generator.ros2

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.aut.code.generator.ros.CodeGenerationUtils

class CodeTemplatePython {
	
	
	
	def static generatePythonNode(Node node)'''
	import rclpy
	from rclpy.node import Node
	
	«FOR msg: CodeGenerationUtils::selectAllMessages(node)»
	from «msg.prefix».msg import «msg.name»
	«ENDFOR»
	
	«FOR s: node.continousstate»
	class «s.name.toFirstUpper»():
		«FOR m: s.receivesFrom»
		msg_«m.id» = None
		«ENDFOR»
		«FOR m: s.producesTo»
		msg_«m.id» = None
		«ENDFOR»
	«ENDFOR»
	
	
	class AbstractGen«node.name.toFirstUpper»(Node):
		
		def __init__(self):
			super().__init__("«node.name.toLowerCase»")
			«FOR st: node.continousstate»
			self.«st.name» = «st.name.toFirstUpper»()
			«ENDFOR»
			
		def init(self):
			«FOR pub: node.outputport»
			self.«pub.id» = self.create_publisher(«pub.topic.type.name», "«pub.topic.name»")
			«ENDFOR»
			«FOR sub: node.inputport»
			self.«sub.id» = self.create_subscription(«sub.topic.type.name», "«sub.topic.name»", self.cb«sub.id.toFirstUpper»)
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