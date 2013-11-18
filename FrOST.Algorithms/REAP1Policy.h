#ifdef FROSTALGORITHMS_EXPORTS
#define  REAP1POLICY_API __declspec(dllexport) 
#else
#define REAP1POLICY_API  __declspec(dllimport) 
#endif

#ifndef FROST_ALGORITHMS_REAP1POLICY
#define FROST_ALGORITHMS_REAP1POLICY

// Compile Options:  /GX
namespace std {
#include <cstdlib>

};	
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <set>


//using namespace System;

namespace REAP1 {

	class ReAP1Policy
	{
	public: 
		REAP1POLICY_API ReAP1Policy();

		/* ---------------------------------------------------------------------
		* State variables
		* --------------------------------------------------------------------- */

		struct REAP1STATE_s	// i.e. a road section connected to the intersection
		{	
			std::vector<int> queueLenghts;		/*		per phase		*/
			int phaseIndex;
			int greenRemaining;
		};

		typedef struct REAP1STATE_s	REAP1STATE;

		ReAP1Policy::REAP1STATE tState;
		void ReAP1Policy::updateState(REAP1STATE nState);

		int nStates;		/*	(15000) queue length (capped) * phase * remaining (max green)  = 100(3-ph) * 3 * 50	*/
		int nActions;		/*	(3) 0: extend current; 1: apply next phase; 2: skip next and apply 2nd next	*/
		std::vector<double> tQvalues;	/* q-values for the set actions at  the current state */

		/* ---------------------------------------------------------------------
		* Reward definition
		* --------------------------------------------------------------------- */

		int ReAP1Policy::reward;

		/* ---------------------------------------------------------------------
		* Q structures
		* --------------------------------------------------------------------- */
		struct comparatorState {
			inline bool operator()(const REAP1STATE& a, const REAP1STATE& b) const {

				bool isEqual = false;

				for(int ph = 0; ph < 3; ph++ )	/*	replace magic number	*/
				{
					isEqual = isEqual && (a.queueLenghts[ph] == b.queueLenghts[ph]);
				}
				
				return isEqual && (a.phaseIndex == b.greenRemaining) && (a.greenRemaining == b.greenRemaining);
			}
		};

		std::map <REAP1STATE, std::vector< double>, comparatorState> Q;
		REAP1POLICY_API void initQValues(double iValue);	/*	1	*/
		REAP1POLICY_API REAP1STATE setState(REAP1STATE state);		/*	2	*/
		REAP1POLICY_API std::vector< double> getQvalues(REAP1STATE state);
		REAP1POLICY_API void setQvalue(REAP1STATE state, int action, double newQ);
		REAP1POLICY_API double getQvalue(REAP1STATE state, int action);
		REAP1POLICY_API double getMaxQvalue(REAP1STATE state);
		REAP1POLICY_API ReAP1Policy::REAP1STATE getStateInstance(std::vector<int> pQueues, int iPhase, int rGreen);
		

		//private:

		///* ---------------------------------------------------------------------
		//* State variables
		//* --------------------------------------------------------------------- */

	};
}

#endif