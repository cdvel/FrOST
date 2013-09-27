#ifndef FROST_SIMULATION_QP_STATES
#define FROST_SIMULATION_QP_STATES

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


#endif