#ifdef FROSTALGORITHMS_EXPORTS
#define  REAP1_API __declspec(dllexport) 
#else
#define REAP1_API  __declspec(dllimport) 
#endif

#ifndef FROST_ALGORITHMS_REAP1
#define FROST_ALGORITHMS_REAP1

// Compile Options:  /GX
namespace std {
#include <cstdlib>

};	
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

//using namespace System;

namespace REAP1 {

	class ReAP1
	{
	public: 
		//ReAP1();
		REAP1_API ReAP1(char*, int);	//load from file
		REAP1_API ReAP1(std::vector<int>, int, int);	//load from vector
		REAP1_API ReAP1(char*, int, int, int); //load from string
		REAP1_API ReAP1(char*, int, int); //load from file with Horizon
		REAP1_API ReAP1::ReAP1(std::vector<std::vector<int> >, int iphase, int horizon); // load from multiarray
		REAP1_API ReAP1::ReAP1(int iphase, int horizon);
		REAP1_API std::vector<int> getFeasibleGreens(int, int);
		REAP1_API int getInitialPhase();
		REAP1_API  int getRed();
		REAP1_API int getArrivals(int, int, int);
		REAP1_API std::vector<int> getOptimalControl(); 
		REAP1_API float getSaturationFlow(int); 
		REAP1_API void setSaturationFlow(int phi, float satFlow);
		REAP1_API void setStartupLostTime(float time);
		REAP1_API void setInitialPhase(int p);
		REAP1_API void setMinGreenTime(int mingreen);
		REAP1_API void setMaxGreenTime(int maxgreen);
		REAP1_API void setRedTime(int redd);
		REAP1_API void setLanePhases(int phi, int lanes);
		REAP1_API void setHorizon(int h);
		REAP1_API void setMaxPhCompute(int mp);
		REAP1_API void setArrivals(std::vector<std::vector<int> > arrivals);
		REAP1_API int getArrivalEarliest(int, int, int, int); //NEW

		REAP1_API void resizeArrivals();
		REAP1_API void initMatrices(int);
		REAP1_API void printVector(std::vector<int> );
		REAP1_API void printMatrix(std::vector<std::vector<int> > );
		REAP1_API std::vector<int> printSequence(int[], int);
		REAP1_API void printArrivals();

		
		void ReAP1::updateReward(int nReward);
		void ReAP1::selectNextAction();

		REAP1_API std::vector<int> RunREAP();
		REAP1_API bool loadFromFile(char*);
		REAP1_API bool loadFromSeq(char*, unsigned int, int);
		REAP1_API bool loadFromVector(std::vector<int>, int);
		REAP1_API void initParameters();

		REAP1_API void setOutput(bool);
		void setNextPhase(int phaseIndex);

		/* ---------------------------------------------------------------------
		* State variables
		* --------------------------------------------------------------------- */

		struct REAP1STATE_s	// i.e. a road section connected to the intersection
		{	
			std::vector<int> queueLenghts;		/*		per each intersection phase		*/
			int phaseIndex;
			int greenRemaining;
		};

		typedef struct REAP1STATE_s	REAP1STATE;

		ReAP1::REAP1STATE tState;
		void ReAP1::updateState(REAP1STATE nState);

		/* ---------------------------------------------------------------------
		* Reward definition
		* --------------------------------------------------------------------- */

		int ReAP1::reward;

	private:

		enum PIEnum {
			QUEUES, STOPS, DELAY
		};

		int PI;
		int red;
		int mingreen;
		int maxgreen;
		float startupLostTime; //time from total halt to free-flow (2 secs)
		unsigned int T; //planning horizon
		unsigned int M; //maximum number of phases to compute
		int initialPhase; // = 2;								//-----------------> state rep
		int idxCurrentPh; //= initialPhase; // set initial phase to C (2)
		//float satHeadway; // avg headway between vehicles during saturated flow
		//float satFlowRate; // = 0.0; No.Lanes / satHeadway
		float satFlows[3]; //per phase
		int lanePhases[3]; //per phase
		bool output;
		char phaseSeq[3]; // A, B, C
		std::vector<int> phases; // A = 0, B = 1, C = 2
		std::vector<int> optControlSequence; // A = 0, B = 1, C = 2
		std::vector< std::vector<int> > arrivalData;		//---------------------> state rep
		std::vector< std::vector<int> > v; //v_j(s_j);
		std::vector< std::vector<int> > x_star; // optimal solutions x*_j(s_j)

		/* ---------------------------------------------------------------------
		* State variables
		* --------------------------------------------------------------------- */

	};
}

#endif