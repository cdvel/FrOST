// FrOST.Simulation.cpp : Defines the exported functions for the DLL application.
//
#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <iterator>
#include <string>
#include <time.h> 
#include <windows.h>
#include <process.h>
#include<random>

extern "C" {
#include "programmer.h"
}

using namespace std;

#include "Cop97A.h"


/*
* Simple prediction plugin that obtains arrival data using upstream
* loop detector information
* 
*/

/*

phase A = WE - SE
phase B = WN - ES (protected left turn)
phase C = NS - SN

*/


#define PHASE_COUNT 3       /* the number of phases */
#define HORIZON_SIZE 70      
#define INITIAL_PHASE_INDEX 2      
#define MIN_GREEN 2      
#define UPSTREAM_DETECTOR_DISTANCE 700       /* metres */

const char phases [3] = {'A', 'B', 'C'};
int prevCount1 = 0;
int prevCount2 = 0;

Bool busy = PFALSE;

int horizon1[HORIZON_SIZE][PHASE_COUNT]; 

typedef struct LOOPDATA_s    LOOPDATA;

struct LOOPDATA_s
{
	LOOP* upstreamDecLoop;
	DETECTOR * upstreamDetector;
	int lane;
	int lastCount;
	int phase;				//A = 0, B = 1, C = 2
};

typedef struct ARRIVALDATA_s    ARRIVALDATA;

struct ARRIVALDATA_s
{
	float arrivalTime;
	float detectionTime;
	float speed;
	int phase;				//A = 0, B = 1, C = 2
};

static LOOPDATA loopDetectorData[8]; //4 upstrDetectors, 2 loops each
static DETECTOR* upstrDetectors[4]; // 4-arm intersection / per approach
static DETECTOR* stoplDetectors[4]; // 4-arm intersection / per approach
static int numLanes = 2;

std::vector<ARRIVALDATA> detectedArrivals;
std::vector<std::vector<int> > arrivalsHorizon;

//simplified turning proportions, must agree OD Matrix

double leftTurnProportion = 0.2;
double rightTurnProportion = 0.1;

vector<COP97A::Cop97A> instances;
vector<int> control;

HANDLE hThread = NULL;
unsigned threadID;

unsigned Counter; 

bool  isThreadRunning = false;
bool  closeHandle = false;

unsigned __stdcall COPThreadFunc( void* data )
{

	clock_t tStart = clock();
	control = instances[0].RunCOP();
	double ttaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
	//Sleep( 5000L - ttaken ); // to run COP at a predetermined frequency, add milliseconds to ttaken and sleep, e.g, 5 secs

	// Print sequence to console
	string str;
	std::stringstream message;
	int cPhase = INITIAL_PHASE_INDEX;

	for (vector<int>::iterator i = control.begin(); i != control.end(); ++i)
	{
		message << phases[cPhase] << ":" << *i << "\t";
		cPhase = (cPhase == 2) ? 0: cPhase+1;			//updatePhase
	}

	float hh = qpg_CFG_simulationTime();
	float mm =  fmod(hh, 60);
	hh = hh / 60;


	qps_GUI_printf("\a %im %4.2fs \t%4.2fs \t %s ",(int)hh ,mm, ttaken, message.str().c_str());

	isThreadRunning = false;
	_endthreadex( 0 );
	return 0;
} 

// Called once after the network is loaded.
void qpx_NET_postOpen(void)
{
	//NEMA ordering i.e. clockwise
	arrivalsHorizon.resize(HORIZON_SIZE);
	for (int h= 0; h < HORIZON_SIZE; h++)
	{
		arrivalsHorizon[h].resize(PHASE_COUNT);

		for (int p=0; p < PHASE_COUNT; p++)
		{
			arrivalsHorizon[h][p]= 0; //Init all to zero
		}
	}

	upstrDetectors[0] = qpg_NET_detector("NS_UPSTREAM_DETECTOR");
	upstrDetectors[1] = qpg_NET_detector("EW_UPSTREAM_DETECTOR");
	upstrDetectors[2] = qpg_NET_detector("SN_UPSTREAM_DETECTOR");
	upstrDetectors[3] = qpg_NET_detector("WE_UPSTREAM_DETECTOR");

	stoplDetectors[0] = qpg_NET_detector("NS_STOPLINE_DETECTOR");
	stoplDetectors[1] = qpg_NET_detector("EW_STOPLINE_DETECTOR");
	stoplDetectors[2] = qpg_NET_detector("SN_STOPLINE_DETECTOR");
	stoplDetectors[3] = qpg_NET_detector("WE_STOPLINE_DETECTOR");

	//TODO: This is not NEMA
	int idxApproach = 0;
	for (int i = 0; i < 8 ; i ++)
	{
		int laneDet = i%numLanes + 1;  // 1 or 2
		loopDetectorData[i].upstreamDetector = upstrDetectors[idxApproach];
		loopDetectorData[i].upstreamDecLoop = qpg_DTC_multipleLoop(upstrDetectors[idxApproach], laneDet);
		loopDetectorData[i].lane = laneDet; 
		loopDetectorData[i].lastCount = 0;

		if (idxApproach == 0  || idxApproach == 2)	// Phase C 
		{
			loopDetectorData[i].phase = 2;
		}
		else // Approaches 1 and 3
		{
			if (laneDet == 1)
				loopDetectorData[i].phase = 0;	//Phase A
			else // TODO: Assumption- all vehicles on the inner lane are turning left, not used now, using probability dist
				loopDetectorData[i].phase = 1;	//Phase B
		}

		idxApproach = idxApproach + i%2; // +1 every 2 cycles
	}

	/*
	loop 0	1	2	3	4	5	6	7	
	appr 0	0	1	1	2	2	3	3
	*/

	// COP instance(s) here

	instances.push_back(COP97A::Cop97A(2, 70));  //empty with initial phase and horizon

	instances[0].setMaxPhCompute(10);
	instances[0].setOutput(false);
	//instances[0].setInitialPhase(2);
	instances[0].setStartupLostTime(2.0);
	instances[0].setMinGreenTime(5);
	instances[0].setRedTime(3);

	instances[0].setSaturationFlow(0, 1800); //in vehicles-per-hour per-lane 
	instances[0].setSaturationFlow(1, 1400);
	instances[0].setSaturationFlow(2, 1800);

	instances[0].setLanePhases(0, 1); //no. of lanes for each phase, used for saturation flow
	instances[0].setLanePhases(1, 1);
	instances[0].setLanePhases(2, 2);

}


// At draw time, highlight the selected object based on our selections.
// NOTE that the User Picks checkbox should be set to on in the GUI.
void qpx_DRW_modelView(void)
{

}

float getEstimatedArrivalTime (float time, float speed, int distance)
{
	return time + distance/speed; //sec + metres / metres/sec
}

//NOTE: Only for resolution 0.5s!
float adjustToStep(float num)
{
	float decimal = num - floor(num);

	if (decimal >= 0  && decimal <= 3)
	{
		return floor(num);
	}
	else
	{
		if(decimal >= 4  && decimal <= 6)
		{
			return (float)(floor(num) + 0.5);
		}
		else
		{
			if (decimal >= 7) //include 0 next int
				return ceil(num);
		}
	}
	return floor(num);
}

float getHorizonStep(ARRIVALDATA arrival, float simulationTime)
{
	float elapsedTime = simulationTime - arrival.detectionTime;
	float distanceToStopline = UPSTREAM_DETECTOR_DISTANCE - arrival.speed * elapsedTime; // toDetector - travelled
	float timeToStopline = distanceToStopline / arrival.speed;
	return adjustToStep(timeToStopline); // floor, .5 or ceiling
}

void clearHorizon()
{
	for (int h=0; h< HORIZON_SIZE; h++)
	{
		for (int p= 0; p <PHASE_COUNT; p++)	
		{
			arrivalsHorizon[h][p]= 0; //Init all to zero
		}
	}
}

void updateHorizon( float simulationTime)
{
	clearHorizon();

	for(unsigned int i=0; i<detectedArrivals.size(); i++) // for all detected vehicles
	{
		if (detectedArrivals[i].arrivalTime < simulationTime) // if estimated arrival has passed
		{
			std::vector<ARRIVALDATA>::iterator it = detectedArrivals.begin();
			std::advance(it, i);
			detectedArrivals.erase(it);		// remove vehicle from the vector!
		}
		else	// if vehicle still in the link, add it to the current horizon
		{
			float arrivalTimeStep  = getHorizonStep(detectedArrivals[i], simulationTime);
			int horizonTime= (int)(arrivalTimeStep * 2); //multiples of 0.5
			/*
			0 0.5 1 1.5 2 2.5 ... 34
			0  1  2  3  4  5 ... 69
			*/

			if (horizonTime < HORIZON_SIZE) //70-s (0-69)
			{
				arrivalsHorizon[horizonTime][detectedArrivals[i].phase]+=1;
			}
		}
	}
}

void estimateQueues(float currentTime)
{
	//TODO: using diff btwn num. of expected arrival and actually departed vehicles...
}

void printVectorToFile()
{
	ofstream myfile;
	myfile.open ("C:\\temp\\arrival-horizon.txt");
	if (myfile.is_open())
	{
		for(int h=0; h<70; h++){
			for(int p=0; p<3; p++)
			{
				myfile << arrivalsHorizon[h][p] << " ";
			}
			myfile << "\n";
		}
	}
	myfile.close();
}

int getPhase(int detectorIndex)
{
	//a uniformly distributed random value
	// TODO: Use better distribution like this

	/*std::linear_congruential<int, 16807, 0, (int)((1U << 31) - 1)> eng;
	eng.seed(eng);
	std::tr1::uniform_real<double> unif(0, 1);
	double udr = unif(eng);*/

	double udr = ((double) rand() / (RAND_MAX+1));

	int pphase = udr > leftTurnProportion ? 0 : 1; //A=0;  B = 1; C=2,
	// rnd number from zero to ten 
	int p;
	switch (detectorIndex)
	{
	case 0: p = 2; break; //A
	case 1: p = 2; break; //A
	case 2: p = pphase; break; //B or C
	case 3: p = pphase; break; //B or C
	case 4: p = 2; break; //A
	case 5: p = 2; break; //A
	case 6: p = pphase; break; //B or C
	case 7: p = pphase; break; //B or C
	}

	/*
	loop 0	1	2	3	4	5	6	7	
	appr 0	0	1	1	2	2	3	3
	*/
	return p;
}

void getNextControl()
{
	clock_t tStart = clock();
	// /* Do your stuff here */
	instances[0].setArrivals(arrivalsHorizon);
	vector<int> controlSequence = instances[0].RunCOP();
	if (controlSequence.size() >2)
	{
		double ttaken = (double)(clock() - tStart)/CLOCKS_PER_SEC;
		qps_GUI_printf("COP: etime %.2fs : C %i : A%i : B%i ",ttaken , controlSequence[0], controlSequence[1], controlSequence[2]);
	}

}

//NOTE: use default timestep = 0.5s i.e. evaluate this blocks 2 times per second
void qpx_NET_timeStep()
{
	float step = qpg_CFG_timeStep();
	//2 evaluations (every simulated second)

	float currentTime = qpg_CFG_simulationTime();

	for(int i=0; i < 8 ; i ++) // check all upstr loop Detectors 2 x approach
	{
		LOOP* theLoop = loopDetectorData[i].upstreamDecLoop;
		int currentCount = qpg_DTL_count(theLoop, 0); // zero to count all vehicle types

		if (currentCount != loopDetectorData[i].lastCount) // add new vehicles detected
		{
			float speed = qpg_DTL_speed(theLoop, APILOOP_COMPLETE);

			ARRIVALDATA dta;
			dta.arrivalTime = getEstimatedArrivalTime(currentTime, speed, UPSTREAM_DETECTOR_DISTANCE);
			dta.detectionTime = currentTime;
			dta.speed = speed;
			dta.phase = getPhase(i);

			detectedArrivals.push_back(dta);
			loopDetectorData[i].lastCount = currentCount;
		}
	}

	updateHorizon(currentTime);
	estimateQueues(currentTime); 	//TODO: Unimplemented

	if(!isThreadRunning)
	{	
		if(hThread != NULL)
			CloseHandle (hThread);	

		isThreadRunning = true;
		instances[0].setArrivals(arrivalsHorizon);
		hThread = (HANDLE)_beginthreadex( NULL, 0, &COPThreadFunc, NULL, 0, &threadID);
	}
}


void qpx_GUI_keyPress(int key, int ctrl, int shift, int left, int middle, int right)
{
	// print to file when middle mouse has been clicked

	if(key == 0x33 && middle) // 3 key + mid click
	{
		printVectorToFile();
		qps_GUI_printf("********* PRINTED ************");
	}
}
