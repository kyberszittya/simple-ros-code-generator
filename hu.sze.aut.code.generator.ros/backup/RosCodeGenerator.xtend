package hu.sze.aut.code.generator.ros

import org.eclipse.emf.ecore.resource.ResourceSet
import org.eclipse.emf.ecore.resource.impl.ResourceSetImpl
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.ecore.xmi.impl.XMIResourceFactoryImpl
import hu.sze.aut.ros.network.model.rosnetworkmodel.RosnetworkmodelPackage
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.StatepubsubmodelPackage
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.MiddlewareNetwork
//import hu.sze.aut.ros.network.model.rosnetworkmodel.Network
import org.eclipse.emf.common.util.URI
import java.io.File
//import hu.sze.aut.ros.network.model.rosnetworkmodel.Topic
import java.util.Map
import java.util.HashMap
import java.util.List
import java.util.ArrayList
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.InputPort
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.OutputPort
import java.util.HashSet
import java.util.Set
import java.io.PrintWriter
import java.io.IOException

class RosCodeGenerator {
	
	ResourceSet resset
	var MiddlewareNetwork middlewareNetwork
	//Network rosnetwork
	// Map topics from middleware network to ROS topics 
	//Map<hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic, Topic> topicmappings;
	
	new(){
		resset = new ResourceSetImpl()
		resset.getResourceFactoryRegistry().getExtensionToFactoryMap()
				.put(Resource.Factory.Registry.DEFAULT_EXTENSION, new XMIResourceFactoryImpl());
		// Set network model package
		resset.packageRegistry.put(RosnetworkmodelPackage.eNS_URI, RosnetworkmodelPackage.eINSTANCE)
		// Set middleware network model
		resset.packageRegistry.put(StatepubsubmodelPackage.eNS_URI, StatepubsubmodelPackage.eINSTANCE)
		//topicmappings = new HashMap
	}
	
	def void createTopicMappings()
	{
		val Map<String, Topic> rostopics = new HashMap; 
		for (topic: rosnetwork.topic){
			rostopics.put(topic.name, topic)
		}
		for (topic: middlewareNetwork.topic){
			val v = rostopics.get(topic.name)
			if (v!==null)
			{
				topicmappings.put(topic, v)
				System.out.println('''Added topic definition for topic «topic.name» to ROS topic «v.name»''')
			}
		}
	}
	
	
	
	def getRosInterfaces(){
		val Set<Node> implementingInterfaces = new HashSet
		for (n: middlewareNetwork.node)	{
			for (i: n.implements){
				implementingInterfaces.add(n)				
			}
		}
		return implementingInterfaces
	}
	
	def void collectTopics(Node node, List<InputPort> inputPorts, List<OutputPort> outputPorts){
		for (port: node.inputport){
			inputPorts.add(port)					
		}
		for (port: node.outputport){
			outputPorts.add(port)
		}
	}
	
	def static boolean checkEveryMessageDefined(
		List<InputPort> inputPorts, List<OutputPort> outputPorts,
		Map<hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic, Topic> topicMappings
	){
		for (port: inputPorts)
		{
			if (topicMappings.get(port.topic)===null){
				System.err.println('''Undefined messagepair for topic: «port.topic.name»''');
				return false;
			}
		}
		for (port: outputPorts)
		{
			if (topicMappings.get(port.topic)===null){
				System.err.println('''Undefined messagepair for topic: «port.topic.name»''');
				return false;
			}
		}
		return true;
	}
	
	def void generateRosInterfaces()	{
		val List<hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic> topics = new ArrayList
		// TODO: I don't know, if it would be better to decompose to components or treated as a whole. Now we treat it as a whole
		val implementinginterfaces = rosInterfaces
				 
		for (n: middlewareNetwork.node)	{
			val List<InputPort> inputPorts = new ArrayList
			val List<OutputPort> outputPorts = new ArrayList
			collectTopics(n, inputPorts, outputPorts)
			for (i: n.implements)
			{
				collectTopics(i, inputPorts, outputPorts)
			}			
			try (val PrintWriter pw_header = new PrintWriter(new File('''src-gen/include/«n.rospackage»/gen_'''+RosCodeTemplates.headerFileName(n).toString))){
				if (checkEveryMessageDefined(inputPorts, outputPorts, topicmappings))
				{
					pw_header.println(RosCodeTemplates.generateRosInterfaceHeader(n, inputPorts, outputPorts, topicmappings))					
				}
				else
				{
					
				}				
			}
			catch(IOException ioex){
				ioex.printStackTrace
			}
			try (val PrintWriter pw_source = new PrintWriter(new File('''src-gen/src/gen_«n.name.toLowerCase»_impl.cpp'''))){
				pw_source.println(RosCodeTemplates.generateInterfaceRosSource(n, inputPorts, outputPorts, topicmappings));
			}
			catch(IOException ioex){
				ioex.printStackTrace
			}
		}
		// Generate sources
		
	}
	
	public def void loadRosNetworkModel(String path)	{
		val File file = new File(path);
		val URI uri = URI.createFileURI(file.getAbsolutePath())
		val Resource resource = resset.getResource(uri, true);
		//rosnetwork = resource.allContents.filter[it instanceof Network].head as Network
		
	}
	
	public def void loadMiddlewareNetworkFragment(String path)	{
		val File file = new File(path);
		val URI uri = URI.createFileURI(file.getAbsolutePath())
		val Resource resource = resset.getResource(uri, true);
		middlewareNetwork = resource.allContents.filter[it instanceof MiddlewareNetwork].head as MiddlewareNetwork
	}
}