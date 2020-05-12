/*
 * generated by Xtext 2.21.0
 */
package hu.sze.aut.ros.middleware.statepubsub.ide;

import com.google.inject.Guice;
import com.google.inject.Injector;
import hu.sze.aut.ros.middleware.statepubsub.RosNetworkDslRuntimeModule;
import hu.sze.aut.ros.middleware.statepubsub.RosNetworkDslStandaloneSetup;
import org.eclipse.xtext.util.Modules2;

/**
 * Initialization support for running Xtext languages as language servers.
 */
public class RosNetworkDslIdeSetup extends RosNetworkDslStandaloneSetup {

	@Override
	public Injector createInjector() {
		return Guice.createInjector(Modules2.mixin(new RosNetworkDslRuntimeModule(), new RosNetworkDslIdeModule()));
	}
	
}