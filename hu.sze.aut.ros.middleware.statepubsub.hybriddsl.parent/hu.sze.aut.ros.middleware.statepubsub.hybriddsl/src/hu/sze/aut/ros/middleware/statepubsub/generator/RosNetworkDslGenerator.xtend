/*
 * generated by Xtext 2.21.0
 */
package hu.sze.aut.ros.middleware.statepubsub.generator

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.AbstractGenerator
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.LanguageType
import hu.sze.aut.code.generator.ros2.CodeTemplatePython
import hu.sze.aut.code.generator.ros2.CodeTemplateCppRos2
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.MiddlewareNetwork
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.NetworkType
import hu.sze.aut.code.generator.ros.RosCodeTemplates
import hu.sze.aut.code.generator.ros.GenerateYamlConfiguration
import static guru.nidi.graphviz.attribute.Records.*;
import static guru.nidi.graphviz.model.Compass.*;
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.InputPort
import java.util.Set
import java.util.HashSet

/**
 * Generates code from your model files on save.
 * 
 * See https://www.eclipse.org/Xtext/documentation/303_runtime_concepts.html#code-generation
 */
class RosNetworkDslGenerator extends AbstractGenerator {
	/*
	def static generateGraphViewOfNode(Node n)
	{
		val Graph g = graph("example1").directed()
        .graphAttr().with(Rank.dir(LEFT_TO_RIGHT))
        .with(
                node("a").with(Color.RED).link(node("b")),
                node("b").link(to(node("c")).with(Style.DASHED))
        );
		Graphviz.fromGraph(g).height(100).render(Format.PNG).toFile(new File("example/ex1.png"));
	}
	*/
	def static void generateRos2Program(IFileSystemAccess2 fsa, Node n)
	{
		switch (n.language)
		{
			case LanguageType::PYTHON:
			{
				fsa.generateFile('''«n.rospackage»/scripts/«n.name».py''', CodeTemplatePython::generatePythonNode(n))				
			}
			case LanguageType::CPP:
			{
				fsa.generateFile('''«n.rospackage»/include/«n.rospackage»/gen_«n.name.toLowerCase».hpp''', CodeTemplateCppRos2::generateRos2InterfaceHeader(n))
				fsa.generateFile('''«n.rospackage»/src/gen_«n.name.toLowerCase».cpp''', CodeTemplateCppRos2::generateRos2Source(n))
			}
		}
	}
	
	def static Set<InputPort> getSyncInputPorts(Node n)
	{
		val Set<InputPort> ports = new HashSet(); 
		for (p: n.inputport)
		{
			if (p.synchronizesState !== null)
			{
				ports.add(p)
			}
		}
		return ports;
	}
	
	def static void generateRosProgram(IFileSystemAccess2 fsa, Node n)
	{
		switch (n.language)
		{
			case LanguageType::CPP:
			{
				if (n.nodeparameters.length > 0)
				{
					for (g: GenerateYamlConfiguration::selectGeneratedStructures(n))
					{
						fsa.generateFile(
							'''«n.rospackage»/include/«n.rospackage»/«g.name.toLowerCase»_struct.hpp''',						
							GenerateYamlConfiguration::generateParamStructure(n, g)							
						)
					}
					fsa.generateFile('''«n.rospackage»/include/«n.rospackage»/default_config_«n.name.toLowerCase».hpp''', GenerateYamlConfiguration::generateDefaultParameterValueHeader(n))
					fsa.generateFile('''«n.rospackage»/src/gen_config_«n.name.toLowerCase».cpp''', GenerateYamlConfiguration::generateYamlConfigReaderRos(n))
				}
				val ps = getSyncInputPorts(n)
				fsa.generateFile('''«n.rospackage»/include/«n.rospackage»/gen_«n.name.toLowerCase».hpp''', RosCodeTemplates::generateRosInterfaceHeader(n, ps.size > 0))
				fsa.generateFile('''«n.rospackage»/src/gen_«n.name.toLowerCase».cpp''', RosCodeTemplates::generateInterfaceRosSource(n, ps.size > 0))
			}
		}
	}
	
	override void doGenerate(Resource resource, IFileSystemAccess2 fsa, IGeneratorContext context) {
	
		val MiddlewareNetwork network = resource.allContents.filter[it instanceof MiddlewareNetwork].head as MiddlewareNetwork
		val mid_type = network.networktype
		
		switch(mid_type)
		{
			case NetworkType::ROS2:
			{
				resource.allContents.filter[it instanceof Node].forEach[
					val n = it as Node
					generateRos2Program(fsa, n)
							
				]
			}
			case NetworkType::ROS1:
			{
				resource.allContents.filter[it instanceof Node].forEach[
					val n = it as Node
					generateRosProgram(fsa, n)
					fsa.generateFile(
						'''«n.rospackage»/src/gen_«n.name.toLowerCase»_dyncfg_callback.cpp''',
						GenerateYamlConfiguration::generateCallbackGenerateDynamicCfgCallback(n)
					)
						
				]
			}
		}
		network.node.filter[it instanceof Node].forEach[
			val n = it as Node
			fsa.generateFile('''«n.rospackage»/param/default.yaml''', 
				GenerateYamlConfiguration::generateNodeYamlConfiguration(n)
			)			
			fsa.generateFile('''«n.rospackage»/dyncfg/«it.name.toFirstUpper».cfg''',
				GenerateYamlConfiguration::generateDynamicReconfigureFromParameters(it)
			)
			
		]
		// Generate graph
		network.node.filter[it instanceof Node].forEach[
			// TODO: Not working yet
			//RosNetworkDslGenerator::generateGraphViewOfNode(it)
		]		
	
//		fsa.generateFile('greetings.txt', 'People to greet: ' + 
//			resource.allContents
//				.filter(Greeting)
//				.map[name]
//				.join(', '))
	}
}
