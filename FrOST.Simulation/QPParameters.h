#ifndef FROST_SIMULATION_QP_PARAMETERS
#define FROST_SIMULATION_QP_PARAMETERS

namespace QPSim{

#define PHASE_COUNT 3       /* the number of phases */
#define HORIZON_SIZE 70      
#define INITIAL_PHASE_INDEX 2      
#define MIN_GREEN 2      
#define UPSTREAM_DETECTOR_DISTANCE 700       /* metres */

	extern char phases [3];  //	A = WE - SE ; B = WN - ES (protected left turn) ; C = NS - SN

	extern LOOPDATA loopDetectorData[8]; //4 upstrDetectors, 2 loops each
	extern DETECTOR* upstrDetectors[4]; // 4-arm intersection / per approach
	extern DETECTOR* stoplDetectors[4]; // 4-arm intersection / per approach
	extern int numLanes;

	extern std::vector<ARRIVALDATA> detectedArrivals;
	extern std::vector<std::vector<int> > arrivalsHorizon;

	//simplified turning proportions, must agree OD Matrix

	extern double leftTurnProportion;
	extern double rightTurnProportion;

	extern std::vector<COP97A::Cop97A> instances;
	extern std::vector<int> control;

	extern HANDLE hThread;
	extern unsigned threadID;

	extern unsigned Counter; 

	extern bool  isThreadRunning;
	extern bool  closeHandle;

}

#endif 