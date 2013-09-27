#include "stdafx.h"

namespace QPSim{

#define PHASE_COUNT 3       /* the number of phases */
#define HORIZON_SIZE 70      
#define INITIAL_PHASE_INDEX 2      
#define MIN_GREEN 2      
#define UPSTREAM_DETECTOR_DISTANCE 700       /* metres */

char phases [3] = {'A', 'B', 'C'};  //	A = WE - SE ; B = WN - ES (protected left turn) ; C = NS - SN

LOOPDATA loopDetectorData[8]; //4 upstrDetectors, 2 loops each
DETECTOR* upstrDetectors[4]; // 4-arm intersection / per approach
DETECTOR* stoplDetectors[4]; // 4-arm intersection / per approach
int numLanes = 2;

std::vector<ARRIVALDATA> detectedArrivals;
std::vector<std::vector<int> > arrivalsHorizon;

//simplified turning proportions, must agree OD Matrix

double leftTurnProportion = 0.2;
double rightTurnProportion = 0.1;

std::vector<COP97A::Cop97A> instances;
std::vector<int> control;

HANDLE hThread = NULL;
unsigned threadID;

unsigned Counter; 

bool  isThreadRunning = false;
bool  closeHandle = false;

}