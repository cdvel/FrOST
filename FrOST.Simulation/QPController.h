#ifndef FROST_SIMULATION_QP_CONTROLLER
#define FROST_SIMULATION_QP_CONTROLLER

namespace QPSim{


	int getPhase(int detectorIndex);
	void getNextControl();
	unsigned __stdcall COPThreadFunc( void* data );
	void printVectorToFile();

}

#endif