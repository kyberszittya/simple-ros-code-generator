package hu.sze.aut.code.generator.ros

class CodeGeneratorProgram {
	def static void main(String[] args) {
		val RosCodeGenerator code_generator = new RosCodeGenerator()
		code_generator.loadMiddlewareNetworkFragment(args.get(0))
		code_generator.loadRosNetworkModel(args.get(1))
		//code_generator.createTopicMappings
		code_generator.generateRosInterfaces
	}
}