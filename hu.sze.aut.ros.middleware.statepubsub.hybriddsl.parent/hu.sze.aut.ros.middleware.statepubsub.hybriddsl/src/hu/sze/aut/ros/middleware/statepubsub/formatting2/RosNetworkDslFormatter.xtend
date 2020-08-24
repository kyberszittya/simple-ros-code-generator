/*
 * generated by Xtext 2.22.0
 */
package hu.sze.aut.ros.middleware.statepubsub.formatting2

import com.google.inject.Inject
import hu.sze.aut.ros.middleware.statepubsub.services.RosNetworkDslGrammarAccess
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.FilePackage
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Stack
import org.eclipse.xtext.formatting2.AbstractFormatter2
import org.eclipse.xtext.formatting2.IFormattableDocument

class RosNetworkDslFormatter extends AbstractFormatter2 {
	
	@Inject extension RosNetworkDslGrammarAccess

	def dispatch void format(Stack stack, extension IFormattableDocument document) {
		// TODO: format HiddenRegions around keywords, attributes, cross references, etc. 
		for (filePackage : stack.filepackage) {
			filePackage.format
		}
		for (channel : stack.channels) {
			channel.format
		}
	}

	def dispatch void format(FilePackage filePackage, extension IFormattableDocument document) {
		// TODO: format HiddenRegions around keywords, attributes, cross references, etc. 
		for (abstractRosFileElement : filePackage.abstractroselement) {
			abstractRosFileElement.format
		}
	}
	
	// TODO: implement for NodeParameterGroup, Node
}
