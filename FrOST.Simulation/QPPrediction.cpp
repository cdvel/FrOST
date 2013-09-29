
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

#include "COP97A.h"
#include "QPStates.h"
#include "QPParameters.h"
#include "QPPrediction.h"
#include "QPController.h"
using namespace std;

namespace QPSim{

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

}