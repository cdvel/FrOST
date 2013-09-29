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

}