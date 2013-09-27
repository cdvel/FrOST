#ifndef FROST_SIMULATION_QP_PREDICTION
#define FROST_SIMULATION_QP_PREDICTION

#include "QPStates.h"

namespace QPSim{

	float getEstimatedArrivalTime (float time, float speed, int distance);
	float adjustToStep(float num);
	float getHorizonStep(ARRIVALDATA arrival, float simulationTime);
	void clearHorizon();
	void updateHorizon(float simulationTime);
	void estimateQueues(float currentTime);

}

#endif 