#include "stdafx.h"

using namespace std;

namespace QPSim{

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

	void qpx_DRW_modelView(void)
	{

	}

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
				dta.arrivalTime =  getEstimatedArrivalTime(currentTime, speed, UPSTREAM_DETECTOR_DISTANCE);
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

}