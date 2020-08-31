package hu.sze.aut.ros.middleware.statepubsub.generator

import com.google.inject.Injector
import org.eclipse.xtext.resource.XtextResourceSet
import hu.sze.aut.ros.middleware.statepubsub.RosNetworkDslStandaloneSetupGenerated
import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.emf.common.util.URI
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.StatepubsubmodelPackage
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Stack
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Topic

class ExampleParser {
	def static void main(String[] args) {
		
		val Injector injector = new RosNetworkDslStandaloneSetupGenerated().createInjectorAndDoEMFRegistration();
		val XtextResourceSet resourceSet = injector.getInstance(XtextResourceSet)
		resourceSet.getPackageRegistry().put(StatepubsubmodelPackage.eNS_URI, StatepubsubmodelPackage.eINSTANCE);
		val Resource resource = resourceSet.getResource(URI.createFileURI("/home/kyberszittya/sze-aut-ws/code-gen/runtime-EclipseApplication/hu.sze.aut.rei.enviornment.representation.description/src/representation_node.rosnetwork"), true);
		
		val Stack s = resource.contents.get(0) as Stack
		
		s.filepackage.forEach[System.out.println(it)]
		s.channels.forEach[
			System.out.println(it)
			if (it instanceof Topic)
			{
				val topic = it as Topic 
				System.out.println(topic.name)
			} 
			
		]
	}
}