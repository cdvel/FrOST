//************************************************

// This is the main DLL file.
//#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "REAP1Policy.h"
#include <algorithm>
#include <iomanip>
#include <random>
#include <float.h>

/*
MS bug and workaround: use std::vector  http://support.microsoft.com/kb/243444
MUST include <vector>
*/

// Compile Options:  /GX
namespace std {
	#include <cstdlib>
};
#include <vector>

using namespace std;

namespace REAP1{

	ReAP1Policy::ReAP1Policy(){
		
		initQValues(0.0000000000000000001 * rand());
		std::vector<int> queueSt;
		queueSt.clear();queueSt.push_back(0);queueSt.push_back(0);queueSt.push_back(0);
		tState = getStateInstance(queueSt, 0, 0);	// TODO: improve initial state
		nStates = Q.size();
		nActions = 3;
	
	}

/* ---------------------------------------------------------------------
* learning & logic
* --------------------------------------------------------------------- */
	
	ReAP1Policy::REAP1STATE ReAP1Policy::getStateInstance(std::vector<int> pQueues, int iPhase, int rGreen)
	{
		ReAP1Policy::REAP1STATE state;
		state.queueLenghts.swap(pQueues);	// TODO: test this swap
		state.phaseIndex = iPhase;
		state.greenRemaining = rGreen;
		return state;
		
	}
	
	ReAP1Policy::REAP1STATE ReAP1Policy::setState(ReAP1Policy::REAP1STATE state)
	{
		tState = state;
		return tState;
	}
	
	void ReAP1Policy::initQValues(double iValue){
		std::vector<int> queueSt;
		// TODO: try without initialisating- When updating set rnd value and add key-value pair
		for (int cph=0; cph<3; cph++)	/*	build all possible state representations	*/
		{
			for (int gr=0; gr<50; gr++)
			{
				for (int qu0=0; qu0<100; qu0++)		/*----------	queue lengths combinations per phase */
				{
					for (int qu1=0; qu1<100; qu1++)
					{
						for (int qu2=0; qu2<100; qu2++)
						{
							queueSt.clear();
							//std::vector<int> queueSt;
							queueSt.push_back(qu0);queueSt.push_back(qu1);queueSt.push_back(qu2);
							REAP1STATE nState = getStateInstance(queueSt, gr, cph);
																					
							std::vector<double> qValues;	/*	add to Q values	; number of actions = 3 */
							qValues.push_back(iValue);qValues.push_back(iValue);qValues.push_back(iValue);
							Q[nState] = qValues;
						}
					}
				}	/*------------	end queue loops	*/
			}
		}
		//queueSt.clear();queueSt.push_back(0);queueSt.push_back(0);queueSt.push_back(0);
		//tState = getNextState(getStateInstance(queueSt, 0, 0));
	}

	
	std::vector< double> ReAP1Policy::getQvalues(REAP1STATE state){
		std::vector< double> newQ;
		newQ.reserve(Q[state].size());
		newQ.swap (Q[state]);
		return ( newQ );		//TODO: verify passing by value
	}

	void ReAP1Policy::setQvalue(REAP1STATE state, int action, double newQ){
		Q[state][action] = newQ;
	}

	double ReAP1Policy::getQvalue(REAP1STATE state, int action){
		return Q[state][action];
	}

	double ReAP1Policy::getMaxQvalue(REAP1STATE state){
		double maxQ = -DBL_MAX;
		std::vector< double> qValues = Q[state];
		for (unsigned int a= 0; a< qValues.size(); a++)
		{
			if (qValues[a] > maxQ)
				maxQ = qValues[a];
		}
		return maxQ;
	}
}