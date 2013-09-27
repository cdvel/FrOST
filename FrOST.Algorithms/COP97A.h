#ifdef FROSTALGORITHMS_EXPORTS
#define  COP97A_API __declspec(dllexport) 
#else
#define COP97A_API  __declspec(dllimport) 
#endif

#ifndef FROST_ALGORITHMS_COP97A
#define FROST_ALGORITHMS_COP97A

// Compile Options:  /GX
namespace std {
#include <cstdlib>

};	
#include <vector>
#include <string>
#include <sstream>
#include <iostream>

//using namespace System;

namespace COP97A {

	class Cop97A
	{
	public: 
		//Cop97A();
		 COP97A_API Cop97A(char*, int);	//load from file
		 COP97A_API Cop97A(std::vector<int>, int, int);	//load from vector
		 COP97A_API Cop97A(char*, int, int, int); //load from string
		 COP97A_API Cop97A(char*, int, int); //load from file with Horizon
		 COP97A_API Cop97A::Cop97A(std::vector<std::vector<int> >, int iphase, int horizon); // load from multiarray
		 COP97A_API Cop97A::Cop97A(int iphase, int horizon);
		 COP97A_API std::vector<int> getFeasibleGreens(int, int);
		 COP97A_API int getInitialPhase();
		 COP97A_API  int getRed();
		 COP97A_API int getQ(int, int, int);
		 COP97A_API int getArrivals(int, int, int);
		 COP97A_API int getB(int, int, int);
		 COP97A_API int getM(int, int); 
		 COP97A_API int getT(int, int); 
		 COP97A_API std::vector<int> getOptimalControl(); 
		 COP97A_API float getSaturationFlow(int); 
		 COP97A_API void setSaturationFlow(int phi, float satFlow);
		 COP97A_API void setStartupLostTime(float time);
		 COP97A_API void setInitialPhase(int p);
		 COP97A_API void setMinGreenTime(int mingreen);
		 COP97A_API void setMaxGreenTime(int maxgreen);
		 COP97A_API void setRedTime(int redd);
		 COP97A_API void setLanePhases(int phi, int lanes);
		 COP97A_API void setHorizon(int h);
		 COP97A_API void setMaxPhCompute(int mp);
		 COP97A_API void setArrivals(std::vector<std::vector<int> > arrivals);
		 COP97A_API int getArrivalEarliest(int, int, int, int); //NEW

		 COP97A_API void resizeArrivals();
		 COP97A_API void initMatrices(int);
		 COP97A_API void printVector(std::vector<int> );
		 COP97A_API void printMatrix(std::vector<std::vector<int> > );
		 COP97A_API std::vector<int> printSequence(int[], int);
		 COP97A_API void printArrivals();

		 COP97A_API std::vector<int> RunCOP();
		 COP97A_API bool loadFromFile(char*);
		 COP97A_API bool loadFromSeq(char*, unsigned int, int);
		 COP97A_API bool loadFromVector(std::vector<int>, int);
		 COP97A_API void initParameters();

		 COP97A_API void setOutput(bool);

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
		int initialPhase; // = 2;
		int idxCurrentPh; //= initialPhase; // set initial phase to C (2)
		//float satHeadway; // avg headway between vehicles during saturated flow
		//float satFlowRate; // = 0.0; No.Lanes / satHeadway
		float satFlows[3]; //per phase
		int lanePhases[3]; //per phase
		bool output;
		char phaseSeq[3]; // A, B, C
		std::vector<int> phases; // A = 0, B = 1, C = 2
		std::vector<int> optControlSequence; // A = 0, B = 1, C = 2
		std::vector< std::vector<int> > arrivalData;
		std::vector< std::vector<int> > v; //v_j(s_j);
		std::vector< std::vector<int> > x_star; // optimal solutions x*_j(s_j)
		std::vector<std::vector<std::vector<int> > > Q; // permanent queue lengths Q_{phi, j}(s_j)
		std::vector<std::vector<std::vector<int> > > L; // temporary queue lengths Q_{phi, j}(s_j, x_j)
		std::vector<std::vector<std::vector<int> > > S; // temporary stopped  L_{sigma, j}(s_j, x_j)

	};
}

#endif