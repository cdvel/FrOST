//************************************************

// This is the main DLL file.
//#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "REAP1.h"
#include <algorithm>
#include <iomanip>

/*
MS bug and workaround: use std::vector  http://support.microsoft.com/kb/243444
MUST include <vector>
*/

// Compile Options:  /GX
namespace std {
	#include <cstdlib>
};
#include <vector>

using namespace std;

namespace REAP1{

/* ---------------------------------------------------------------------
 * access
 * --------------------------------------------------------------------- */

	void ReAP1::setOutput(bool option){
		output = option;
	}

	void ReAP1::setInitialPhase(int ph){
		idxCurrentPh = initialPhase = ph;
	}
	
	void  ReAP1::setSaturationFlow(int phi, float satFlow) {
		satFlows[phi] = satFlow;
	}

	void  ReAP1::setLanePhases(int phi, int lanes) {
		lanePhases[phi] = lanes;
	}
	
	void ReAP1::setStartupLostTime(float time){
		startupLostTime = time;
	}
		
	void ReAP1::setMinGreenTime(int mgreen){
		mingreen = mgreen;
	}
	
	void ReAP1::setMaxGreenTime(int mxgreen){
		maxgreen = mxgreen;
	}

	void ReAP1::setRedTime(int rd){
		red= rd;
	}

	void ReAP1::setHorizon(int h){
		T= h;
		resizeArrivals();
	}
	
	void ReAP1::setMaxPhCompute(int mp){
		M= mp;
	}

	void ReAP1::setArrivals(std::vector<std::vector<int> > arrivals){
		arrivalData = arrivals;
	};

	int ReAP1::getArrivals(int a, int b, int phi) {

		if (a == b)
			return 0;

		int vehicles = 0;
		int cc = a;

		do {
			vehicles += arrivalData[cc][phi];
			cc++;
		} while (cc < b); // for [a,b), with a!=b

		return vehicles;
	}
		
	float  ReAP1::getSaturationFlow(int phi) { // sat-flow rate per phase, not per lane. in vehicles per sec
		// NOTE: Simplicity assumption return 0;
		float sf = satFlows[phi];
		if (sf <0) return 0;
		
		return (satFlows[phi]/3600)*lanePhases[phi]; // in vphpl, to vpspl
	}

	std::vector<int> ReAP1::getOptimalControl(){
		return optControlSequence;
	}; 

	int ReAP1::getInitialPhase(){
		return initialPhase;
	}

	int ReAP1::getRed(){
		return red;
	}

/* ---------------------------------------------------------------------
 * initialisation
 * --------------------------------------------------------------------- */

	void ReAP1::initParameters(){
		PI = DELAY;
		red = 1; //1
		mingreen = 2; //2
		maxgreen = 50;
		startupLostTime = 0;
		T = 10; //planning horizon
		M = 9; //maximum number of phases to compute (1 to M-1)
		phaseSeq[0]= 'A';phaseSeq[1]= 'B';phaseSeq[2]= 'C';
		setSaturationFlow(0, -1.0); setSaturationFlow(1, -1.0); setSaturationFlow(2, -1.0);
		phases = std::vector<int>(phaseSeq, phaseSeq + sizeof (phaseSeq) /sizeof (phaseSeq[0]));
		output = false;
		resizeArrivals();
	}

	void ReAP1::resizeArrivals()
	{
		arrivalData.resize(T);
		for (unsigned int i = 0; i < T; ++i) {
			arrivalData[i].resize(phases.size());
		}
	}

	void  ReAP1::initMatrices(int init) {
			for (unsigned int i = 0; i < v.size(); ++i) {
				for (unsigned int j = 0; j < v[i].size(); ++j) {
					if (i == 0) {
						x_star[i][j] = v[i][j] = 0;

					} else {
						x_star[i][j] = v[i][j] = init;
					}
				}
			}
		}
		
/* ---------------------------------------------------------------------
 * constructors
 * --------------------------------------------------------------------- */

	ReAP1::ReAP1(char* file, int iphase){
		initParameters();
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(char* file, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(char* data, int size, int nphases, int iphase){
		initParameters();
		if(!loadFromSeq(data, size, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(std::vector<int> data, int nphases, int iphase){
		initParameters();
		if(!loadFromVector(data, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(std::vector<std::vector<int> > data, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		arrivalData = data;
		idxCurrentPh = initialPhase = iphase;
	};

	ReAP1::ReAP1(int iphase, int horizon){
		initParameters();
		setInitialPhase(iphase);
		setHorizon(horizon);
		idxCurrentPh = initialPhase;
	
	};

/* ---------------------------------------------------------------------
 * in/out
 * --------------------------------------------------------------------- */
		
	bool  ReAP1::loadFromFile(char* filename) {
		unsigned int x, y;
		ifstream in(filename);

		if (!in) {
			cout << "Cannot open file.\n";
			return false;
		}

		for (y = 0; y < T; y++) {
			for (x = 0; x < phases.size(); x++) {

				in >> arrivalData[y][x];
			}
		}
		in.close();

		return true;
	};

	bool ReAP1::loadFromSeq(char* data, unsigned int size, int nPhases) {
		
		//ifstream in(filename);
		int ic = 0;
		int dataI;
		for (unsigned int ix = 0; ix < size; ++ix)
		{
			dataI = data[ix];
			dataI -= 48; //0 = 48, 1 = 49

			// TODO: works only for 1 digit data
			if(dataI >= 0) // skip spaces 
			{
				arrivalData[ic/nPhases][ic%nPhases] = dataI; 
				ic++;
			}
		}

		return true;
	}

	bool ReAP1::loadFromVector(std::vector<int> data, int nPhases) {
		
		for (unsigned int ix = 0; ix < data.size(); ++ix)
		{
			arrivalData[ix/nPhases][ix%nPhases] = data[ix];
		}
		return true;
	}

	void  ReAP1::printArrivals() {
		cout << "\n\n"; 
		for (unsigned int p=0; p < phases.size(); ++p)
		{
			cout << "\t" << phaseSeq[p];
		}
		cout << "\n\n";
		for (unsigned int i = 0; i < T; ++i) {
			cout << i+1 << "\t";
			for (unsigned int j = 0; j < phases.size(); ++j) {
				cout << arrivalData[i][j] << "\t";
			}
			cout << endl;
		}
		cout << endl;
	}

	void  ReAP1::printVector(vector<int> values) {

		cout << flush << "[  ";
		for (vector<int>::iterator i = values.begin(); i != values.end(); ++i) {
			if (i!= values.begin())
				cout << setfill (' ' ) << setw (3);

			int ix = *i;
			if (ix < 0)
				cout << "-";
			else
				cout <<ix;
		}
		cout << "  ]";
	}

	void  ReAP1::printMatrix(vector<vector<int> > values) {

		for (unsigned int i = 0; i < values.size(); ++i) {
			printVector(values[i]);
			cout << endl;
		}
	}

	vector<int>  ReAP1::printSequence(int arry[], int sz) {

		vector<int> seq;
		cout << "[ ";
		for (int i = 0; i < sz; i++) {
			seq.push_back(arry[i]);
			cout << phaseSeq[(i+initialPhase)%phases.size()]<<":"<< arry[i] << " ";
		}
		cout << "]";

		return seq;
	}

/* ---------------------------------------------------------------------
 * learning & logic
 * --------------------------------------------------------------------- */

//	void ReAP1::initQ(){

		//for (int i=0 ;i<nstate; i++)
		//{
		//	double actions[3] = {.0,.0,.0};
		//	Q[tState] = actions;
		//
		//}
//	}

	void ReAP1::updateState(ReAP1::REAP1STATE nState)
	{
		 tState = nState;
		 //TODO: overload
		// read values from environment and store in the agents mem
	}

	void ReAP1::updateReward(int newReward)
	{
		reward = newReward; // modify based on reward structure, eg, cummulative delay 

		//compute reward from action and store or 
	}

	void ReAP1::selectNextAction()
	{
		// based on the the reward and current state
		// extend current phase (how much?) or switch to any other phase (variable sequence)
	}

	vector<int> ReAP1::RunREAP() {

		std::vector<int> optControlSequence;

		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);
		optControlSequence.push_back(55);

		cout << "REAP started...\n";
		//selectNextAction();
		//updateState(NULL);
		//updateReward();

		cout << "\nOptimal Control Sequence: \n\n"; 
		cout << "\n\n...REAP ended\n\n";
		return optControlSequence;

	}; 

}