//#ifdef COP97ADLL_EXPORTS
//#define  __declspec(dllexport) 
//#else
//#define  __declspec(dllimport) 
//#endif

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
		 Cop97A(char*, int);	//load from file
		 Cop97A(std::vector<int>, int, int);	//load from vector
		 Cop97A(char*, int, int, int); //load from string
		 Cop97A(char*, int, int); //load from file with Horizon
		 Cop97A::Cop97A(std::vector<std::vector<int> >, int iphase, int horizon); // load from multiarray
		 Cop97A::Cop97A(int iphase, int horizon);
		 std::vector<int> getFeasibleGreens(int, int);
		 int getInitialPhase();
		 int getRed();
		 int getQ(int, int, int);
		 int getArrivals(int, int, int);
		 int getB(int, int, int);
		 int getM(int, int); 
		 int getT(int, int); 
		 std::vector<int> getOptimalControl(); 
		 float getSaturationFlow(int); 
		 void setSaturationFlow(int phi, float satFlow);
		 void setStartupLostTime(float time);
		 void setInitialPhase(int p);
		 void setMinGreenTime(int mingreen);
		 void setMaxGreenTime(int maxgreen);
		 void setRedTime(int redd);
		 void setLanePhases(int phi, int lanes);
		 void setHorizon(int h);
		 void setMaxPhCompute(int mp);
		 void setArrivals(std::vector<std::vector<int> > arrivals);
		 int getArrivalEarliest(int, int, int, int); //NEW

		 void resizeArrivals();
		 void initMatrices(int);
		 void printVector(std::vector<int> );
		 void printMatrix(std::vector<std::vector<int> > );
		 std::vector<int> printSequence(int[], int);
		 void printArrivals();

		 std::vector<int> RunCOP();
		 bool loadFromFile(char*);
		 bool loadFromSeq(char*, unsigned int, int);
		 bool loadFromVector(std::vector<int>, int);
		 void initParameters();

		 void setOutput(bool);

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
