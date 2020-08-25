package hu.sze.aut.code.generator.ros

import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.Node
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.NodeParameter
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.NodeParameterGroup
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.AbstractNodeParameter
import java.util.Set
import java.util.HashSet
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.DoubleParameter
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.BooleanParameter
import hu.sze.jkk.middleware.statepubsub.model.statepubsubmodel.IntegerParameter

class GenerateYamlConfiguration {
	
	def static generateDynConfValues(NodeParameter param)'''«
	   IF param instanceof DoubleParameter»«param.defaultval», «param.minval», «param.maxval»«
	   ELSEIF param instanceof IntegerParameter»«param.defaultval»,«param.minval»,«param.maxval»«
	   ELSEIF param instanceof BooleanParameter»«IF param.defaultval»True«ELSE»False«ENDIF»«ENDIF»'''
	
	def static generateAddDynCfg(NodeParameter param, String grp_name)'''«grp_name».add("«param.name»", «param.dynCfgType», 0,"", «generateDynConfValues(param)»)'''
	
	def static CharSequence generateDynamicReconfigureParameter(AbstractNodeParameter param, String grpname)'''«
	IF param instanceof NodeParameter»«generateAddDynCfg(param, grpname)»«
	ELSEIF param instanceof NodeParameterGroup»
	grp_«param.name» = gen.add_group("«param.name»", type="tab")
	«FOR p: param.nodeparameter»
	«generateDynamicReconfigureParameter(p, "grp_"+param.name)»
	«ENDFOR»
	«ENDIF»'''
	
	def static CharSequence generateDynCfgCallbackUpdate(AbstractNodeParameter param)'''
	«IF param instanceof NodeParameter»
	«IF param.targetvar!==null»
	if («(param as NodeParameter).targetvar» != config.«(param as NodeParameter).name»)
	{
		ROS_INFO_STREAM("Updated message [«param.targetvar»]: «param.name»" << config.«(param as NodeParameter).name»);
	}
    «(param as NodeParameter).targetvar» = config.«(param as NodeParameter).name»;
    «ENDIF»
    «ELSEIF param instanceof NodeParameterGroup»
    «FOR p: param.nodeparameter»
    «generateDynCfgCallbackUpdate(p)»
    «ENDFOR»
    «ENDIF»
	'''
	
	def static generateCallbackGenerateDynamicCfgCallback(Node n)'''
	#include <«n.filepackage.name»/«n.name.toLowerCase».hpp>
	
	namespace «n.namespace»
	{
	
	void «n.name»::callbackReconfigure(«n.filepackage.name»::«n.name»Config & config,uint32_t level)
	{
		«FOR param: n.nodeparameters»
		«generateDynCfgCallbackUpdate(param)»
		«ENDFOR»
	}
	
	}
	'''
	
	def static CharSequence generateDynamicReconfigureFromParameters(Node n)'''
	#!/usr/bin/env python
	from dynamic_reconfigure.parameter_generator_catkin import *
	gen = ParameterGenerator()
	«FOR p: n.nodeparameters»
	«generateDynamicReconfigureParameter(p, "gen")»
	«ENDFOR»
	exit(gen.generate("«n.filepackage.name»", "«n.name»", "«n.name.toFirstUpper»"))
	'''
	
	def static CharSequence generateNodeParameter(AbstractNodeParameter param)'''
	«IF param instanceof NodeParameterGroup»
	«param.name»:
	  «FOR p: param.nodeparameter»
	  «generateNodeParameter(p)»
	  «ENDFOR»
	«ELSEIF param instanceof NodeParameter»	
	«param.name»: «param.parameterValue»
	«ENDIF»
	'''
	
	def static selectGeneratedStructures(Node n)
	{
		val Set<NodeParameterGroup> res = new HashSet
		for (p: n.nodeparameters.filter[it instanceof NodeParameterGroup])
		{
			val g = p as NodeParameterGroup
			if (g.generate_struct)
			{
				res.add(g)
			}
			
		}
		return res
	}
	
	def static generateNodeYamlConfiguration(Node node)'''
	«FOR p: node.nodeparameters»
	«generateNodeParameter(p)»
	«ENDFOR»
	'''
	
	def static getParameterValue(NodeParameter param)'''«
	IF param instanceof DoubleParameter»«(param as DoubleParameter).defaultval.toString»«
	ELSEIF param instanceof BooleanParameter»«IF (param as BooleanParameter).defaultval»true«ELSE»false«ENDIF»«
	ELSEIF param instanceof IntegerParameter»«(param as IntegerParameter).defaultval.toString»«ENDIF»
	'''
	
	def static getDynCfgType(NodeParameter param)'''«
    IF param instanceof DoubleParameter»double_t«
    ELSEIF param instanceof BooleanParameter»bool_t«
    ELSEIF param instanceof IntegerParameter»int_t«ENDIF»'''
	
	def static getCType(NodeParameter param)'''«
	IF param instanceof DoubleParameter»double«
	ELSEIF param instanceof BooleanParameter»bool«
	ELSEIF param instanceof IntegerParameter»int«ENDIF»'''
	
	def static CharSequence generateParameterConstExpr(AbstractNodeParameter param)
	'''«IF param instanceof NodeParameter»constexpr «getCType(param)» DEFAULT_«param.name.toUpperCase» = «getParameterValue(param)»;«ELSEIF 
	param instanceof NodeParameterGroup»
	namespace «param.name»
	{
		«FOR p: param.nodeparameter»
		«generateParameterConstExpr(p)»
		«ENDFOR»
	}
	«ENDIF»'''
	
	def static generateDefaultParameterValueHeader(Node node)'''
	#ifndef INCLUDE_«node.namespace.toUpperCase»_«node.name.toUpperCase»_HPP_
	#define INCLUDE_«node.namespace.toUpperCase»_«node.name.toUpperCase»_HPP_
	
	namespace «node.namespace»
	{
		«FOR param: node.nodeparameters»
		«generateParameterConstExpr(param)»
		«ENDFOR»
	}
	
	#endif
	«»
	'''
	
	def static CharSequence generateParamStructure(Node n, NodeParameterGroup group)'''
	#ifndef _INCLUDE_«n.name.toUpperCase»_HPP
	#define _INCLUDE_«n.name.toUpperCase»_HPP
	
	namespace «n.namespace»
	{
	
	struct ConfigStruct«group.name.toFirstUpper»
	{
		«FOR p: group.nodeparameter»
		«p.CType» «p.name»;
		«ENDFOR»
	};
	
	}
	#endif
	'''
	
	
	def static CharSequence generateParamReader(Node n, AbstractNodeParameter param, String ns)
	'''«IF param instanceof NodeParameterGroup»
	//
	// GROUP «param.name»
	«FOR p: param.nodeparameter»
	«generateParamReader(n, p, ns+param.name+"/")»
	«ENDFOR»
	«ELSEIF param instanceof NodeParameter»
	«IF param.targetvar!==null»
	// Load param «param.name»
	if (!private_nh->getParam("«ns»«param.name»", «param.targetvar»))
	{
		«param.targetvar» = «ns.replace("/", "::")»DEFAULT_«param.name.toUpperCase»;
	}
	ROS_INFO_STREAM("Using «ns»«param.name»:=" << «param.targetvar»);
	«ENDIF»
	«ENDIF»	
	'''
	
	def static generateYamlConfigReaderRos(Node n)'''
	#include "«n.filepackage.name.toLowerCase»/«n.name.toLowerCase».hpp"
	#include "«n.filepackage.name.toLowerCase»/default_config_«n.name.toLowerCase».hpp"
	«FOR v: selectGeneratedStructures(n)»
	#include <«n.filepackage.name»/«v.name.toLowerCase»_struct.hpp>
	«ENDFOR»
	
	namespace «n.namespace»
	{
	
	void «n.name»::genParamConfig()
	{
		«FOR p: n.nodeparameters»
		«generateParamReader(n, p, "")»
		«ENDFOR»
	}
	
	}
	'''
}