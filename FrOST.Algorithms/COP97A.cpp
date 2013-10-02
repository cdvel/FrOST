// This is the main DLL file.
//#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include "COP97A.h"
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

namespace COP97A{

	void Cop97A::setOutput(bool option){
		output = option;
	}

	void Cop97A::setInitialPhase(int ph){
		idxCurrentPh = initialPhase = ph;
	}
	
	void  Cop97A::setSaturationFlow(int phi, float satFlow) {
		satFlows[phi] = satFlow;
	}

	void  Cop97A::setLanePhases(int phi, int lanes) {
		lanePhases[phi] = lanes;
	}
	
	void Cop97A::setStartupLostTime(float time){
		startupLostTime = time;
	}
		
	void Cop97A::setMinGreenTime(int mgreen){
		mingreen = mgreen;
	}
	
	void Cop97A::setMaxGreenTime(int mxgreen){
		maxgreen = mxgreen;
	}

	void Cop97A::setRedTime(int rd){
		red= rd;
	}

	void Cop97A::setHorizon(int h){
		T= h;
		resizeArrivals();
	}
	
	void Cop97A::setMaxPhCompute(int mp){
		M= mp;
	}

	void Cop97A::setArrivals(std::vector<std::vector<int> > arrivals){
		arrivalData = arrivals;
	};

	void Cop97A::initParameters(){
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

	void Cop97A::resizeArrivals()
	{
		arrivalData.resize(T);
		for (unsigned int i = 0; i < T; ++i) {
			arrivalData[i].resize(phases.size());
		}
	}

	Cop97A::Cop97A(char* file, int iphase){
		initParameters();
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	Cop97A::Cop97A(char* file, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		if(!loadFromFile(file))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	Cop97A::Cop97A(char* data, int size, int nphases, int iphase){
		initParameters();
		if(!loadFromSeq(data, size, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	Cop97A::Cop97A(std::vector<int> data, int nphases, int iphase){
		initParameters();
		if(!loadFromVector(data, nphases))
			exit(0);
		idxCurrentPh = initialPhase = iphase;
	};

	Cop97A::Cop97A(std::vector<std::vector<int> > data, int iphase, int horizon){
		initParameters();
		setHorizon(horizon);
		arrivalData = data;
		idxCurrentPh = initialPhase = iphase;
	};

	Cop97A::Cop97A(int iphase, int horizon){
		initParameters();
		setInitialPhase(iphase);
		setHorizon(horizon);
		idxCurrentPh = initialPhase;
	
	};

	std::vector<int>  Cop97A::getFeasibleGreens(int sj, int j) {

		std::vector< int > set;

		//if (j == 1) //simplification removes the min green restriction for stage 1
		//{
		//	if (sj-red <= maxgreen)	// NEW
		//		set.push_back(sj - red);
		//	else
		//		set.push_back(maxgreen);	//NEW
		//}
		
		if (j == 1) //simplification removes the min green restriction for stage 1
		{			// NEW 1 stage case, usually applied, no zero, min green value or sj - red
			//set.push_back(0);
			int gg = sj - red;
			if (gg < mingreen)
				set.push_back(mingreen);
			else
			{
				if (gg <= maxgreen)	// NEW
					set.push_back(gg);
				else
					set.push_back(maxgreen);	//NEW
			}
		}
		
		else {
			set.push_back(0);		// allow phase skipping
			int c = mingreen;

			if (!(sj - red < mingreen)) {
				do {
					set.push_back(c);
					c++;
					if (c > maxgreen) //NEW
					{
						break;
					}
				} while (c < (sj - red));	
			}
		}

		return set;
	};

	int Cop97A::getArrivals(int a, int b, int phi) {

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

	int  Cop97A::getQ(int sj, int ph, int j) {

		if (sj == 0)
			return 0; //assuming initial queues are zero
		//index fix
		return Q[sj - 1][ph][j - 1]; //NEW: sj, starts at red, but this is a vector index fix
		//return Q[sj - red][ph][j - 1]; //was: this might be sj-1 instead
	}

	int  Cop97A::getB(int a, int b, int phi) {

		int dB = 0;
		int k = a; 
		int bb = b;

		while (k < b) { //a <= ak < b
			if (arrivalData[k][phi] != 0) //if vehicle k requests phase phi
				dB += b - k;
			k++;
		}
		return dB;
	}

	int  Cop97A::getArrivalEarliest(int si, int sj, int xj, int phi)
	{

		//arrival time of the earliest vehicle required to stop when phi(j);
		int timeArrival = 999;


		//NEW TODO: Work on improvements?
		//if(si > xj)
		//{
		//	for (int oph = 0; oph < phases.size(); oph++)
		//	{
		//		if (oph!=phi)	// check  other phases
		//		{
		//			for(int ix = sj - xj; ix < si + xj; ix++)	// while phase phi has r-o-w for xj
		//			{
		//				if (arrivalData[ix][oph] != 0 && timeArrival > ix){	// first arrival 
		//					timeArrival = ix;
		//					break;
		//				}
		//			}
		//		}
		//	}
		//}

		//// no stops, similar for A  calculations
		//if (timeArrival == 999) 
		timeArrival = si + xj;

		return timeArrival;
	}
	
	int  Cop97A::getM(int phi, int xj) { 
		// Maximum no. of vehicles that can be discharged in xj seconds for phase phi

		/*  TODO: simplicity assumption: M_phi(x) = INF for all x > 0;
		*   i.e. instantaneous queue clearance
		*
		*  M_phi (x) = saturation_flow_rate_(phi) * x
		* saturation_flow_rate_ in vphpl vehicles per hour per lane for phase phi
		*/

		/*
		if (xj == 0)
			return 0;

		return 100000;
		*/

		if (xj == 0)
			return 0;

		float satRate = getSaturationFlow(phi);
		float m = 100000.0;

		//return 100000;		// use simplicity assumption if no sat-flow is set, arbitrily large value
		if (satRate > 0)
			m = satRate * xj; //TODO: Check startupLT
		//float m = satRate * (xj - startupLostTime); //TODO: Check startupLT
		return (int)floor(m); // in vehicles
	
	}

	int  Cop97A::getT(int d, int phi) {	
		//function of no of vehicles discharged, total delay in discharging d vehicles

		// example assumption, instantaneous queue clearance = 0, for all d
		/*
		* T_phi (d) = d / saturation_flow_rate_(phi)
		* d is the number of vehicles to discharge
		*
		*/

		//return 0;


		if (d==0)
			return 0;

		float satRate = getSaturationFlow(phi);
		float t = 0;
	    if(satRate > 0)
			t  = d / satRate;	
		
		return (int)ceil(t + startupLostTime); // in seconds //TODO: Check startupLT ..  + startupLostTime
	}

	float  Cop97A::getSaturationFlow(int phi) { // sat-flow rate per phase, not per lane. in vehicles per sec
		// NOTE: Simplicity assumption return 0;
		float sf = satFlows[phi];
		if (sf <0) return 0;
		
		return (satFlows[phi]/3600)*lanePhases[phi]; // in vphpl, to vpspl
	}

	std::vector<int> Cop97A::getOptimalControl(){
		return optControlSequence;
	}; 

	int Cop97A::getInitialPhase(){
		return initialPhase;
	}

	int Cop97A::getRed(){
		return red;
	}

	void  Cop97A::printArrivals() {
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

	bool  Cop97A::loadFromFile(char* filename) {
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

	bool Cop97A::loadFromSeq(char* data, unsigned int size, int nPhases) {
		
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

	bool Cop97A::loadFromVector(std::vector<int> data, int nPhases) {
		
		for (unsigned int ix = 0; ix < data.size(); ++ix)
		{
			arrivalData[ix/nPhases][ix%nPhases] = data[ix];
		}
		return true;
	}

	void  Cop97A::initMatrices(int init) {
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

	vector<int>  Cop97A::printSequence(int arry[], int sz) {

		vector<int> seq;
		cout << "[ ";
		for (int i = 0; i < sz; i++) {
			seq.push_back(arry[i]);
			cout << phaseSeq[(i+initialPhase)%phases.size()]<<":"<< arry[i] << " ";
		}
		cout << "]";

		return seq;
	}

	void  Cop97A::printVector(vector<int> values) {

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

	void  Cop97A::printMatrix(vector<vector<int> > values) {

		for (unsigned int i = 0; i < values.size(); ++i) {
			printVector(values[i]);
			cout << endl;
		}
	}

	vector<int> Cop97A::RunCOP() {
		cout << "COP started...\n";

		cout << "\n\nInput Arrival Data: ";
		//printArrivals();

		std::vector< std::vector<int> > X;
		X.resize(T);

		v.resize(M);
		x_star.resize(M);

		for (unsigned int i = 0; i < M; ++i) {
			v[i].resize(T);
			x_star[i].resize(T);
		}

		initMatrices(-1);
		unsigned int j = 1;
		bool criterion_flag = 1;

		do {
			if(output){
				// <editor-fold defaultstate="collapsed" desc="header stage">
				cout << endl << "\n\t\t\tStage " << j << " Calculations [" << phaseSeq[idxCurrentPh] << "]" << endl;
				cout << "--------------------------------------------------------------------" << endl;
				cout << "s" << j << "\tx*(s" << j << ")\tv(s" << j << ")\tQA\tQB\tQC\tXj(s" << j << ")\n";
				cout << "--------------------------------------------------------------------"<< endl;
				// </editor-fold>
			}
			for (unsigned int sj = red; sj <= T; sj++) {
				
				if(output)
				cout << " " << sj;

				X[j] = getFeasibleGreens(sj, j);
				int xSz = X[j].size();

				L.resize(T);
				S.resize(T);
				for (unsigned int i = 0; i < T; ++i) {
					L[i].resize(xSz);
					S[i].resize(xSz);

					for (int j = 0; j < xSz; ++j) {
						L[i][j].resize(phases.size());
						S[i][j].resize(phases.size());
					}
				}

				Q.resize(T);
				for (unsigned int i = 0; i < T; ++i) {
					Q[i].resize(phases.size());

					for (unsigned int j = 0; j < phases.size(); ++j)
						Q[i][j].resize(M);
				}

				int index_xj = 0;
				int currentValueFn = -1;
				int minValueFn = 99999;
				int optimal_x = -1;
				int optimal_index_x = -1;

				for (vector<int>::iterator it = X[j].begin(); it != X[j].end(); ++it) {
					int xj = *it;

					int hj = (xj!=0) ? (xj+red) : 0; //transition value

					// at stage 0, no steps allocated
					int si = (j!=1) ? (sj-hj) : 0; // si equals s_{j-1}
					//  cout << "\n hj, si: " <<hj << ", " << si<< "\n";

					int tQueue = 0;
					int tStops = 0;
					int tDelay = 0;

					int index_sj = sj - red; //index fix
					int index_maxPh = -1;

					//performance index calculation Max Q Length
					// which phase has the longest temp queue?
					int pi_MaxQ = -1;
					int pi_NumStops = 0;
					int pi_Delay = 0;

					for (unsigned int index_p = 0; index_p < phases.size(); index_p++) {

						if (index_p != idxCurrentPh) // phase w/o right-of-way
						{

							int arrival = arrivalData[index_sj][index_p]; //

							// temporary queues
							tQueue = getQ(si, index_p, j - 1)
								+ getArrivals(si, sj, index_p);

							// temporary stops
							tStops = getArrivals(si, sj, index_p);

							//delay
							tDelay = getQ(si, index_p, j - 1)*(sj - si)
								+ getB(si, sj, index_p);

						} else { //phase with right-of-way

							// temporary queues
							int queueTerm = getQ(si, idxCurrentPh, j - 1)
								+ getArrivals(si, si + xj, idxCurrentPh)
								- getM(idxCurrentPh, xj);

							tQueue = max(0, queueTerm) 
								+ getArrivals(si + xj, sj, idxCurrentPh);

							// temporary stops
							int stopsTerm =
								getArrivals(si, si + xj, idxCurrentPh)
								- max(0, getM(idxCurrentPh, xj)
								- getQ(si, idxCurrentPh, j - 1));

							tStops = max(0, stopsTerm)
								+ getArrivals(si + xj, sj, idxCurrentPh);


							//NEW
							/*
							Calculate tp, function of sj and xj
							*/
							int tp = getArrivalEarliest(si, sj, xj, idxCurrentPh);

							//    int tp = si + xj; // equals s_{j} - red
							//    cout <<"(tp: "<< tp<<")";

							// delay
							int delayTerm = min(getQ(si, idxCurrentPh, j - 1),
								getM(idxCurrentPh, xj));

							tDelay = getT(delayTerm, idxCurrentPh)
								+ max(0, getQ(si, idxCurrentPh, j - 1) -
								getM(idxCurrentPh, xj))*(sj - si)
								+ getB(tp, sj, idxCurrentPh);
						}

						// record temporary queue lengths and stops
						L[index_sj][index_xj][index_p] = tQueue;
						S[index_sj][index_xj][index_p] = tStops;

						// PI Max Queue : use operator max
						if (tQueue > pi_MaxQ) {
							pi_MaxQ = tQueue;
							index_maxPh = index_p;
						}

						// PI Stops & Delay : use operator +
						pi_NumStops += tStops;
						pi_Delay += tDelay;

					} //end phaseSequence cycle

					// index fix TODO: implications
					if (j != 1 && si >= red){ // index fix to use si
						si -= red;
						//cout << "   si = " << si << " \n"; 
					}

					switch (PI) {
					case QUEUES:
						currentValueFn = max(pi_MaxQ, v[j - 1][si]);
						break;
					case STOPS:
						currentValueFn = pi_NumStops + v[j - 1][si];
						break;
					case DELAY:
						currentValueFn = pi_Delay + v[j - 1][si];
						break;
					}

					//minimisation v_j : keep minimum value
					if (minValueFn > currentValueFn) {
						minValueFn = currentValueFn;
						optimal_x = xj;
						optimal_index_x = index_xj;
					}

					index_xj++;
				} // end X[j] cycle

				// sj - red :  adjust value to column index
				v[j][sj - red] = minValueFn;
				x_star[j][sj - red] = optimal_x;

				// -red and -1 deal, reconcile indices

				int optIndeX = 0; // stage 1 simplification
				if (j != 1)
					optIndeX = optimal_index_x;

				// temporary to permanent queue lengths
				for (unsigned int pp = 0; pp < phases.size(); pp++)
					Q[sj - red][pp][j - 1] = L[sj - red][optIndeX][pp]; // -1 :index

				/**print*************************/
				if(output){
					cout << "\t" << optimal_x;
					cout << "\t" << v[j][sj - red];
				
					for (unsigned int pp = 0; pp < phases.size(); pp++)
						cout << "\t" << Q[sj - red][pp][j - 1];

					cout << setfill(' ') << setw(30 - 2 * L.size());
					cout.flush();
					printVector(X[j]);
					cout << endl;
				if (sj % 2 == 0)
					cout << endl;
				}

				// </editor-fold>
				/**print*************************/

			} //end sj cycle

			//************ STOPPING CRITERION ***********
			//if(criterion_flag)
			//{ 
			if (j >= phases.size()) {
				for (unsigned int k = 1; k <= phases.size() - 1; k++) {
					criterion_flag = criterion_flag && (v[j - k][T- red] == v[j][T - red]);
				}

				criterion_flag = !criterion_flag;
				idxCurrentPh = idxCurrentPh==2 ? 0:idxCurrentPh + 1;
				if (criterion_flag)
				{
					//Updates index of current phase in cycles
					j++;
				}
			}
			else
			{
				idxCurrentPh = idxCurrentPh==2 ? 0:idxCurrentPh + 1;
				j++;
			}
			//}
		} while (criterion_flag && j < M); // NEW: second condition

		if (output){
			cout << "\nStopping Criterion Triggered!\n";
			cout << "\n\nValue Functions for all Stages v(j,sj)\n\n";
			printMatrix(v);
			cout << "\nDecision Table for all Stages x*(j, sj)\n\n";
			printMatrix(x_star);
		}

		/*  Retrieval of Optimal Policy     */
		// cout << endl << "j :"<< j  <<endl;

		const int jsize = j - (phases.size() - 1);
		int s_star= T;

		//cout << endl << "jsize :"<< jsize  <<endl;
		//cout << endl << "s* :"<< s_star  <<endl;

		//new
		//int idxSeq = initialPhase;
		int idxSeq = idxCurrentPh;
		//string controlSeq = "[ ";
		//cout << "[ ";
		//int optimalControlSeq [jsize];
		int* optimalControlSeq = new int[jsize];

		for(int jj= jsize; jj>=1; jj--)
		{
			//cout <<"jsize "<< jsize;
			//cout <<"*jj, s_star-red = "<< jj <<", "<<s_star-red<<endl; 
			int xx = x_star[jj][s_star-red];
			//cout <<"jj, s_star-red = "<< jj <<", "<<s_star-red << " = " << xx<<endl;
			optimalControlSeq[jj-1] = xx;

			if (jj > 1) {
				int hj_star = (xx!=0) ? (xx+red) : 0; 
				s_star = s_star - hj_star;
				//s_star = (s_star <= red) ? red : s_star - hj_star;
				if (s_star <= red) s_star = red;
			}

			idxSeq = idxSeq==0 ? 2:idxSeq - 1;
		}
		//cout << "]\n";

		
		cout << "\nOptimal Control Sequence: \n\n"; 
		optControlSequence = printSequence(optimalControlSeq, jsize);
		delete optimalControlSeq;
		cout << "\n\n...COP ended\n\n";
		return optControlSequence;


	}; /**************** END MAIN*************/
}