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
#include <map>
#include <set>
#include "REAP1Policy.h"

//using namespace System;

namespace REAP1 {

	class ReAP1
	{
	public: 
		//ReAP1();
		REAP1_API ReAP1();
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
		//void setNextPhase(int phaseIndex);
	
		REAP1_API REAP1::ReAP1Policy getPolicy();
		REAP1_API bool getRandomFlag();
		REAP1_API void setAlpha(double a);
		REAP1_API double getAlpha();
		REAP1_API void setGamma(double g);
		REAP1_API double getGamma();
		REAP1_API void setEpsilon(double e);
		REAP1_API double getEpsilon();
		REAP1_API void initPolicy();
		REAP1_API bool validAction(int action);		/* check whether action is permitted */
		
		/*	Invoked by thread in the controller	*/
		REAP1_API int selectAction(REAP1::ReAP1Policy::REAP1STATE iState);

		REAP1_API void setNewState(REAP1::ReAP1Policy::REAP1STATE iState);	
		REAP1_API void setObservedReward(double oReward);
		REAP1_API void setObservedStateReward(REAP1::ReAP1Policy::REAP1STATE iState, double oReward);

		REAP1_API void updateQ();
		REAP1_API void updateState();

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
		* RL
		* --------------------------------------------------------------------- */
		
		double epsilon;		/*	for epsilon-greedy	*/
	    double temp;

		double alpha;		/*	learning rate	*/
		double gamma;		/*	discount factor	*/
		double lambda;

		REAP1::ReAP1Policy::REAP1STATE state;
		REAP1::ReAP1Policy::REAP1STATE newState;
		REAP1::ReAP1Policy policy;
		int action;				//TODO: Use enum instead
		double reward;
		bool random;

	};
}

#endif