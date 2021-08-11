/***************************************************************************

    file                 : can_server.cpp
    copyright            : (C) 2021 Enrico Pozzobon

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <ctime>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <tgf.h>
#include <tgfclient.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include "sensors.h"
#include "ObstacleSensors.h"


#define NBWHEELS 4
#define NBBUSES 3
#define NBBOTS 10
#define NBOBSTACLESENSORS 36
#define NBTRACKSENSORS 19

#define CAN_BUS_SENSORS 0
#define CAN_BUS_CONTROL 1
#define CAN_BUS_CTRLOUT 2

#define CAN_ID_FUEL_GEAR_RPM       0x001
#define CAN_ID_POS                 0x002
#define CAN_ID_DISTRACED           0x003
#define CAN_ID_LAPTIME             0x004
#define CAN_ID_ANGLE               0x005
#define CAN_ID_SPEED               0x006
#define CAN_ID_DAMMAGE             0x007
#define CAN_ID_WHEELSPINVEL        0x008
#define CAN_ID_OBSTACLE_SENSORS_A  0x010
#define CAN_ID_OBSTACLE_SENSORS_B  0x011
#define CAN_ID_OBSTACLE_SENSORS_C  0x012
#define CAN_ID_OBSTACLE_SENSORS_D  0x013
#define CAN_ID_OBSTACLE_SENSORS_E  0x014
#define CAN_ID_TRACKSENS_A         0x020
#define CAN_ID_TRACKSENS_B         0x021
#define CAN_ID_TRACKSENS_C         0x022

#define CAN_ID_CMD_ACCEL     0x401
#define CAN_ID_CMD_BRAKE     0x402
#define CAN_ID_CMD_GEAR      0x403
#define CAN_ID_CMD_STEER     0x404
#define CAN_ID_CMD_CLUTCH    0x405

#define TXMSGBOX_FUEL_GEAR_RPM        0
#define TXMSGBOX_POS                  1
#define TXMSGBOX_DISTRACED            2
#define TXMSGBOX_LAPTIME              3
#define TXMSGBOX_ANGLE                4
#define TXMSGBOX_SPEED                5
#define TXMSGBOX_DAMMAGE              6
#define TXMSGBOX_WHEELSPINVEL         7
#define TXMSGBOX_OBSTACLE_SENSORS_A   8
#define TXMSGBOX_OBSTACLE_SENSORS_B   9
#define TXMSGBOX_OBSTACLE_SENSORS_C  10
#define TXMSGBOX_OBSTACLE_SENSORS_D  11
#define TXMSGBOX_OBSTACLE_SENSORS_E  12
#define TXMSGBOX_TRACKSENS_A         13
#define TXMSGBOX_TRACKSENS_B         14
#define TXMSGBOX_TRACKSENS_C         15
#define NBCANTXMSGBOXES              16

#define RXMSGBOX_ACCEL   0
#define RXMSGBOX_BRAKE   1
#define RXMSGBOX_GEAR    2
#define RXMSGBOX_STEER   3
#define RXMSGBOX_CLUTCH  4
#define NBCANRXMSGBOXES  5

typedef int SOCKET;
typedef struct sockaddr_in tSockAddrIn;
#define CLOSE(x) close(x)
#define INVALID(x) x < 0

#define RACE_RESTART 1

double __SENSORS_RANGE__;

/*** Noise definitions ***/
#define __NOISE_STD__ 0.1
#define __OPP_NOISE_STD__ 0.02

static tTrack	*curTrack;
static int RESTARTING[NBBOTS];

static tdble prevDist[NBBOTS];
static tdble distRaced[NBBOTS];

static int currentKey[256];
static int currentSKey[256];

static int cansend(int socket, canid_t identifier, const void *data, size_t size);
static int canpoll(int socket, can_frame *output);
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void drive_inputs(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt);
static int can_update(int index);

static double normRand(double avg,double std);

/**** variables for CAN ***/
static int canSocket[NBBOTS][NBBUSES];
/************************************************/

static Sensors *trackSens[NBBOTS];
static ObstacleSensors *oppSens[NBBOTS];
static float trackSensAngle[NBBOTS][NBTRACKSENSORS];

static const char* botname[NBBOTS] = {"can_server 1", "can_server 2", "can_server 3", "can_server 4", "can_server 5", "can_server 6", "can_server 7", "can_server 8", "can_server 9", "can_server 10"};

static unsigned long total_tics[NBBOTS];

static uint64_t can_tx_msgboxes[NBBOTS][NBCANTXMSGBOXES];
static uint64_t can_rx_msgboxes[NBBOTS][NBCANRXMSGBOXES];
static const unsigned can_tx_msgboxes_dlc[NBCANTXMSGBOXES] = {
    7, 8, 8, 8, 4, 6, 4, 8,
    8, 8, 8, 8, 4,
    8, 8, 3
};
static const canid_t can_tx_msgboxes_ids[NBCANTXMSGBOXES] = {
    0x001, 0x002, 0x003, 0x004, 0x005, 0x006, 0x007, 0x008,
    0x010, 0x011, 0x012, 0x013, 0x014,
    0x020, 0x021, 0x022,
};

static float clampf(float x, float m, float M) {
    if (x < m) return m;
    if (x > M) return M;
    return x;
}

#define CLAMP_BITS(x, b, s) ((((1ULL << (b)) - 1) & ((uint64_t)(x))) << (s))

static void prepare_can_frame_fuel_gear_rpm(int index, uint32_t fuel, uint8_t gear, uint32_t rpm) {
    can_tx_msgboxes[index][TXMSGBOX_FUEL_GEAR_RPM] =
        CLAMP_BITS(fuel, 29, 0) | CLAMP_BITS(gear, 3, 29) | CLAMP_BITS(rpm, 24, 32);
}

static void prepare_can_frame_pos(int index, int32_t pos, uint32_t pos_z) {
    can_tx_msgboxes[index][TXMSGBOX_POS] =
        CLAMP_BITS(pos, 32, 0) | CLAMP_BITS(pos_z, 32, 32);
}

static void prepare_can_frame_dist_raced(int index, uint32_t curDistRaced, uint32_t distFromStartLine) {
    can_tx_msgboxes[index][TXMSGBOX_DISTRACED] =
        CLAMP_BITS(curDistRaced, 32, 0) | CLAMP_BITS(distFromStartLine, 32, 32);
}

static void prepare_can_frame_lap_time(int index, uint32_t curLapTime, uint32_t lastLapTime) {
    can_tx_msgboxes[index][TXMSGBOX_LAPTIME] =
        CLAMP_BITS(curLapTime, 32, 0) | CLAMP_BITS(lastLapTime, 32, 32);
}

static void prepare_can_frame_angle(int index, uint16_t uangle, int16_t distToMiddle) {
    can_tx_msgboxes[index][TXMSGBOX_ANGLE] =
        CLAMP_BITS(uangle, 16, 0) | CLAMP_BITS(distToMiddle, 16, 16);
}

static void prepare_can_frame_speed(int index, uint16_t speed_x, uint16_t speed_y, uint16_t speed_z) {
    can_tx_msgboxes[index][TXMSGBOX_SPEED] =
        CLAMP_BITS(speed_x, 16, 0) | CLAMP_BITS(speed_y, 16, 16) | CLAMP_BITS(speed_z, 16, 32);
}

static void prepare_can_frame_damage(int index, int dammage) {
    can_tx_msgboxes[index][TXMSGBOX_DAMMAGE] =
        CLAMP_BITS(dammage, 32, 0);
}

static void prepare_can_frame_wheels(int index, float wheels[NBWHEELS]) {
    can_tx_msgboxes[index][TXMSGBOX_WHEELSPINVEL] =
        CLAMP_BITS((uint16_t) (wheels[0] * 60.0f / (2 * PI)), 16, 0) |
        CLAMP_BITS((uint16_t) (wheels[1] * 60.0f / (2 * PI)), 16, 16) |
        CLAMP_BITS((uint16_t) (wheels[2] * 60.0f / (2 * PI)), 16, 32) |
        CLAMP_BITS((uint16_t) (wheels[3] * 60.0f / (2 * PI)), 16, 48);
}

static void prepare_can_frame_obstacles(int index, float obstacle[NBOBSTACLESENSORS]) {
    can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_A] =
        CLAMP_BITS(255.0f * clampf(obstacle[0], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(obstacle[1], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(obstacle[2], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(obstacle[3], 0.0f, 1.0f), 8, 24) |
        CLAMP_BITS(255.0f * clampf(obstacle[4], 0.0f, 1.0f), 8, 32) |
        CLAMP_BITS(255.0f * clampf(obstacle[5], 0.0f, 1.0f), 8, 40) |
        CLAMP_BITS(255.0f * clampf(obstacle[6], 0.0f, 1.0f), 8, 48) |
        CLAMP_BITS(255.0f * clampf(obstacle[7], 0.0f, 1.0f), 8, 56);
    can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_B] =
        CLAMP_BITS(255.0f * clampf(obstacle[8], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(obstacle[9], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(obstacle[10], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(obstacle[11], 0.0f, 1.0f), 8, 24) |
        CLAMP_BITS(255.0f * clampf(obstacle[12], 0.0f, 1.0f), 8, 32) |
        CLAMP_BITS(255.0f * clampf(obstacle[13], 0.0f, 1.0f), 8, 40) |
        CLAMP_BITS(255.0f * clampf(obstacle[14], 0.0f, 1.0f), 8, 48) |
        CLAMP_BITS(255.0f * clampf(obstacle[15], 0.0f, 1.0f), 8, 56);
    can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_C] =
        CLAMP_BITS(255.0f * clampf(obstacle[16], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(obstacle[17], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(obstacle[18], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(obstacle[19], 0.0f, 1.0f), 8, 24) |
        CLAMP_BITS(255.0f * clampf(obstacle[20], 0.0f, 1.0f), 8, 32) |
        CLAMP_BITS(255.0f * clampf(obstacle[21], 0.0f, 1.0f), 8, 40) |
        CLAMP_BITS(255.0f * clampf(obstacle[22], 0.0f, 1.0f), 8, 48) |
        CLAMP_BITS(255.0f * clampf(obstacle[23], 0.0f, 1.0f), 8, 56);
    can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_D] =
        CLAMP_BITS(255.0f * clampf(obstacle[24], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(obstacle[25], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(obstacle[26], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(obstacle[27], 0.0f, 1.0f), 8, 24) |
        CLAMP_BITS(255.0f * clampf(obstacle[28], 0.0f, 1.0f), 8, 32) |
        CLAMP_BITS(255.0f * clampf(obstacle[29], 0.0f, 1.0f), 8, 40) |
        CLAMP_BITS(255.0f * clampf(obstacle[30], 0.0f, 1.0f), 8, 48) |
        CLAMP_BITS(255.0f * clampf(obstacle[31], 0.0f, 1.0f), 8, 56);
    can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_E] =
        CLAMP_BITS(255.0f * clampf(obstacle[32], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(obstacle[33], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(obstacle[34], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(obstacle[35], 0.0f, 1.0f), 8, 24);
}

static void prepare_can_frame_track_sensors(int index, float sensor[NBTRACKSENSORS]) {
    can_tx_msgboxes[index][TXMSGBOX_TRACKSENS_A] =
        CLAMP_BITS(255.0f * clampf(sensor[0], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(sensor[1], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(sensor[2], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(sensor[3], 0.0f, 1.0f), 8, 24) |
        CLAMP_BITS(255.0f * clampf(sensor[4], 0.0f, 1.0f), 8, 32) |
        CLAMP_BITS(255.0f * clampf(sensor[5], 0.0f, 1.0f), 8, 40) |
        CLAMP_BITS(255.0f * clampf(sensor[6], 0.0f, 1.0f), 8, 48) |
        CLAMP_BITS(255.0f * clampf(sensor[7], 0.0f, 1.0f), 8, 56);
    can_tx_msgboxes[index][TXMSGBOX_TRACKSENS_B] =
        CLAMP_BITS(255.0f * clampf(sensor[8], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(sensor[9], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(sensor[10], 0.0f, 1.0f), 8, 16) |
        CLAMP_BITS(255.0f * clampf(sensor[11], 0.0f, 1.0f), 8, 24) |
        CLAMP_BITS(255.0f * clampf(sensor[12], 0.0f, 1.0f), 8, 32) |
        CLAMP_BITS(255.0f * clampf(sensor[13], 0.0f, 1.0f), 8, 40) |
        CLAMP_BITS(255.0f * clampf(sensor[14], 0.0f, 1.0f), 8, 48) |
        CLAMP_BITS(255.0f * clampf(sensor[15], 0.0f, 1.0f), 8, 56);
    can_tx_msgboxes[index][TXMSGBOX_TRACKSENS_C] =
        CLAMP_BITS(255.0f * clampf(sensor[16], 0.0f, 1.0f), 8, 0) |
        CLAMP_BITS(255.0f * clampf(sensor[17], 0.0f, 1.0f), 8, 8) |
        CLAMP_BITS(255.0f * clampf(sensor[18], 0.0f, 1.0f), 8, 16);
}

/*
 * Module entry point
 */
extern "C" int
    can_server(tModInfo *modInfo)
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

	for (int i = 0; i < NBBOTS; i++) {
		modInfo[i].name    = strdup(botname[i]);  // name of the module (short).
		modInfo[i].desc    = strdup(botname[i]);  // Description of the module (can be long).
		modInfo[i].fctInit = InitFuncPt;// Init function.
		modInfo[i].gfId    = ROB_IDENT;	// Supported framework version.
		modInfo[i].index   = i;		// Indices from 0 to 9.
	}
    return 0;
}

/* Module interface initialization. */
static int
InitFuncPt(int index, void *pt)
{
    tRobotItf *itf  = (tRobotItf *)pt;

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    /* for every track change or new race */
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */

    return 0;
}

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    curTrack = track;
    *carParmHandle = NULL;
#ifdef _PRINT_RACE_RESULTS__
    trackName = strrchr(track->filename, '/') + 1;
#endif
}

/* Start a new race. */
static void
newrace(int index, tCarElt* car, tSituation *s)
{

    total_tics[index]=0;

	memset(currentKey, 0, sizeof(currentKey));
	memset(currentSKey, 0, sizeof(currentSKey));

    //Set sensor range
    if (strcmp(getVersion(),"2009")==0)
    {
    	__SENSORS_RANGE__ = 100;
    	printf("*****2009*****\n");
    }
    else if (strcmp(getVersion(),"2010")==0 || strcmp(getVersion(),"2011")==0 || strcmp(getVersion(),"2012")==0 || strcmp(getVersion(),"2013")==0)
        __SENSORS_RANGE__ = 200;
    else
    {
    	printf("%s is not a recognized version",getVersion());
    	exit(0);
    }

    for (int busIndex = 0; busIndex < NBBUSES; busIndex++) {
        std::cout << "Opening bus " << busIndex << " on car " << index << "\n";
        canSocket[index][busIndex] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (canSocket[index][busIndex] < 0)
        {
            std::cerr << "Error: cannot create canSocket!";
            exit(1);
        }

        struct ifreq ifr;
        snprintf(ifr.ifr_name, IF_NAMESIZE, "vcan%d%c", index, busIndex + (int) 'a');
        ifr.ifr_name[IF_NAMESIZE-1] = 0;
        if (-1 == ioctl(canSocket[index][busIndex], SIOCGIFINDEX, &ifr)) {
            std::cerr << "Error: cannot ioctl canSocket!";
            exit(1);
        }

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (0 > bind(canSocket[index][busIndex], (struct sockaddr *)&addr, sizeof(addr))) {
            std::cerr << "Error: cannot bind canSocket!";
            exit(1);
        }
    }

    srand(time(NULL));

    // Initialization of track sensors
    trackSens[index] = new Sensors(car, NBTRACKSENSORS);
    for (int i = 0; i < NBTRACKSENSORS; ++i) {
    	trackSens[index]->setSensor(i,trackSensAngle[index][i],__SENSORS_RANGE__);
#ifdef __CAN_SERVER_VERBOSE__
    	std::cout << "Set Track Sensors " << i+1 << " at angle " << trackSensAngle[index][i] << std::endl;
#endif
	}
    // Initialization of opponents sensors
    oppSens[index] = new ObstacleSensors(NBOBSTACLESENSORS, curTrack, car, s, (int) __SENSORS_RANGE__);

    prevDist[index]=-1;
}

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s)
{
    drive_inputs(index, car, s);
    total_tics[index]++;

    // computing distance to middle
    float dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    // computing the car angle wrt the track axis
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI

    // update the value of track sensors only as long as the car is inside the track
    float trackSensorOut[NBTRACKSENSORS];
    if (dist_to_middle <= 1.0 && dist_to_middle >= -1.0)
    {
        trackSens[index]->sensors_update();
		for (int i = 0; i < NBTRACKSENSORS; ++i)
        {
            trackSensorOut[i] = trackSens[index]->getSensorOut(i);
            if (getNoisy())
            	trackSensorOut[i] *= normRand(1,__NOISE_STD__);
        }
    }
    else
    {
        for (int i = 0; i < NBTRACKSENSORS; ++i)
        {
            trackSensorOut[i] = -1;
        }
    }

    // update the value of opponent sensors
    float oppSensorOut[NBOBSTACLESENSORS];
    oppSens[index]->sensors_update(s);
    for (int i = 0; i < NBOBSTACLESENSORS; ++i)
    {
        oppSensorOut[i] = oppSens[index]->getObstacleSensorOut(i);
        if (getNoisy())
        	oppSensorOut[i] *= normRand(1,__OPP_NOISE_STD__);
    }

    float wheelSpinVel[NBWHEELS];
    for (int i = 0; i < NBWHEELS; ++i)
    {
        wheelSpinVel[i] = car->_wheelSpinVel(i);
    }

    if (prevDist[index] < 0)
    {
	    prevDist[index] = car->race.distFromStartLine;
    }
    float curDistRaced = car->race.distFromStartLine - prevDist[index];
    prevDist[index] = car->race.distFromStartLine;
    if (curDistRaced > 100)
    {
	    curDistRaced -= curTrack->length;
    }
    if (curDistRaced < -100)
    {
	    curDistRaced += curTrack->length;
    }

    distRaced[index] += curDistRaced;


    {
        uint32_t fuel = (uint32_t) (car->_fuel * 256 * 256);
        uint8_t gear = (uint8_t) car->_gear;
        uint32_t rpm = (uint32_t) car->_enginerpm * 256;
        prepare_can_frame_fuel_gear_rpm(index, fuel, gear, rpm);

        int dammage = getDamageLimit() ? car->_dammage : car->_fakeDammage;
        prepare_can_frame_damage(index, dammage);

        uint16_t speed_x = (uint16_t) (car->_speed_x * 256);
        uint16_t speed_y = (uint16_t) (car->_speed_y * 256);
        uint16_t speed_z = (uint16_t) (car->_speed_z * 256);
        prepare_can_frame_speed(index, speed_x, speed_y, speed_z);

        uint16_t uangle = (uint16_t) ((1 << 16) * angle / (PI * 2));
        int16_t distToMiddle = (int16_t) (INT16_MAX * clampf(dist_to_middle, -1.0f, 1.0f));
        prepare_can_frame_angle(index, uangle, distToMiddle);

        uint32_t curLapTime = (uint32_t) (car->_curLapTime * 256);
        uint32_t lastLapTime = (uint32_t) (car->_lastLapTime * 256);
        prepare_can_frame_lap_time(index, curLapTime, lastLapTime);

        uint32_t distFromStartLine = (uint32_t) (car->race.distFromStartLine * 256);
        uint32_t curDistRaced = (uint32_t) (distRaced[index] * 256);
        prepare_can_frame_dist_raced(index, curDistRaced, distFromStartLine);

        int32_t pos = car->race.pos;
        uint32_t pos_z = (uint32_t) (256 * (car->_pos_Z - RtTrackHeightL(&(car->_trkPos))));
        prepare_can_frame_pos(index, pos, pos_z);

        prepare_can_frame_wheels(index, wheelSpinVel);
        prepare_can_frame_obstacles(index, oppSensorOut);
        prepare_can_frame_track_sensors(index, trackSensorOut);
    }

    if (RESTARTING[index]==0)
    {
#ifdef __CAN_SERVER_VERBOSE__
        std::cout << "Sending: " << line << std::endl;
#endif

        car->_accelCmd  = ((uint8_t *) &can_rx_msgboxes[RXMSGBOX_ACCEL ])[0] / 255.0f;
        car->_brakeCmd  = ((uint8_t *) &can_rx_msgboxes[RXMSGBOX_BRAKE ])[0] / 255.0f;
        car->_gearCmd   = "\x00\x01\x02\x03\x04\x05\x06\xff"[((uint8_t *) &can_rx_msgboxes[RXMSGBOX_GEAR  ])[0] & 7];
        car->_steerCmd  = ((int8_t  *) &can_rx_msgboxes[RXMSGBOX_STEER ])[0] / 127.0f;
        car->_clutchCmd = ((uint8_t *) &can_rx_msgboxes[RXMSGBOX_CLUTCH])[0] / 255.0f;
    }

    if (can_update(index)) {
        std::cout <<
            car->_accelCmd << " " <<
            car->_brakeCmd << " " <<
            car->_gearCmd << " " <<
            car->_steerCmd << " " <<
            car->_clutchCmd << std::endl;
    }
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
    RESTARTING[index]=0;
    if (trackSens != NULL)
    {
        delete trackSens[index];
        trackSens[index] = NULL;
    }

    if (oppSens[index] != NULL)
    {
        delete oppSens[index];
        oppSens[index] = NULL;
    }
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    if (RESTARTING[index]!=1)
    {
        if (cansend(canSocket[index][CAN_BUS_SENSORS], 0x100, "shutdown", 8) < 0)
            std::cerr << "Error: cannot send shutdown message";
    }
    else
    {
        if (cansend(canSocket[index][CAN_BUS_SENSORS], 0x101, "restart", 7) < 0)
            std::cerr << "Error: cannot send restart message";
    }
    RESTARTING[index]=0;
    if (trackSens[index] != NULL)
    {
        delete trackSens[index];
        trackSens[index] = NULL;
    }
    if (oppSens[index] != NULL)
    {
        delete oppSens[index];
        oppSens[index] = NULL;
    }
    for (int busIndex = 0; busIndex < NBBUSES; busIndex++) {
        CLOSE(canSocket[index][busIndex]);
    }
    GfuiKeyEventRegisterCurrent(NULL);
    GfuiSKeyEventRegisterCurrent(NULL);
}

double normRand(double avg,double std)
{
	 double x1, x2, w, y1, y2;

	    do {
	            x1 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
	            x2 = 2.0 * rand()/(double(RAND_MAX)) - 1.0;
	            w = x1 * x1 + x2 * x2;
	    } while ( w >= 1.0 );

	    w = sqrt( (-2.0 * log( w ) ) / w );
	    y1 = x1 * w;
	    y2 = x2 * w;
	    return y1*std + avg;
}

static int cansend(int socket, canid_t identifier, const void *data, size_t size)
{
    if (size > 8) {
        size = 8;
    }
    can_frame frame;
    frame.can_id  = identifier;
	frame.can_dlc = size;
    memcpy(frame.data, data, size);

	ssize_t result = write(socket, &frame, sizeof(struct can_frame));

    if (result < 0) {
        std::cerr << "Error: cannot send can frame";
    }

    return (int) result;
}

static int canpoll(int socket, can_frame *output) {
    struct timeval timeVal;
    fd_set readSet;
    // Set timeout for client answer
    FD_ZERO(&readSet);
    FD_SET(socket, &readSet);
    timeVal.tv_sec = 0;
    timeVal.tv_usec = 0;

    if (select(socket+1, &readSet, NULL, NULL, &timeVal)) {
        // Read the client controller action
        int numRead = read(socket, output, sizeof(*output));
        if (numRead < 0)
        {
            std::cerr << "Error, cannot get any response from the client!";
            CLOSE(socket);
            exit(1);
        }
#ifdef __CAN_SERVER_VERBOSE__
            std::cout << "Received CAN frame with identifier " << output->can_id << std::endl;
#endif
        return numRead;
    } else {
        return 0;
    }
}

static int can_update(int index) {
    int total_messages = 0;
    int s = canSocket[index][CAN_BUS_SENSORS];
    for (int i = 0; i < NBCANTXMSGBOXES; i++) {
        cansend(s, can_tx_msgboxes_ids[i], &can_tx_msgboxes[index][i], can_tx_msgboxes_dlc[i]);
    }

    can_frame frame;
    while (0 < canpoll(canSocket[index][CAN_BUS_CONTROL], &frame))
    {
        total_messages++;
        // Set controls command and store them in variables
        switch (frame.can_id) {
            case CAN_ID_CMD_ACCEL : memcpy(can_rx_msgboxes[RXMSGBOX_ACCEL ], frame.data, 8); break;
            case CAN_ID_CMD_BRAKE : memcpy(can_rx_msgboxes[RXMSGBOX_BRAKE ], frame.data, 8); break;
            case CAN_ID_CMD_GEAR  : memcpy(can_rx_msgboxes[RXMSGBOX_GEAR  ], frame.data, 8); break;
            case CAN_ID_CMD_STEER : memcpy(can_rx_msgboxes[RXMSGBOX_STEER ], frame.data, 8); break;
            case CAN_ID_CMD_CLUTCH: memcpy(can_rx_msgboxes[RXMSGBOX_CLUTCH], frame.data, 8); break;
            default: break;
        }
    }
    return total_messages;
}

static int onKeyAction(unsigned char key, int modifier, int state) {
	currentKey[key] = state;
	return 0;
}

static int onSKeyAction(int key, int modifier, int state) {
	currentSKey[key] = state;
	return 0;
}

static void drive_inputs(int index, tCarElt* car, tSituation *s) {
    static int first_time = 1;
    static float intensity[4] = {0, 0, 0, 0};

    if (first_time) {
        first_time = 0;
		GfuiKeyEventRegisterCurrent(onKeyAction);
		GfuiSKeyEventRegisterCurrent(onSKeyAction);
    }

    float steerPressed = (currentKey['a'] == GFUI_KEY_DOWN ? -1.0f : 0.0f)
                       + (currentKey['d'] == GFUI_KEY_DOWN ? 1.0f : 0.0f);
    float upPressed    = currentKey['w'] == GFUI_KEY_DOWN ? 1.0f : 0.0f;
    float downPressed  = currentKey['s'] == GFUI_KEY_DOWN ? 1.0f : 0.0f;
    float spacePressed = currentKey[' '] == GFUI_KEY_DOWN ? 1.0f : 0.0f;
    intensity[0] = clampf(intensity[0] * 0.8f + steerPressed * 0.25f, -1.0f, 1.0f);
    intensity[1] = clampf(intensity[1] * 0.8f + upPressed    * 0.2f + (upPressed    - 0.5f) * 0.1f,  0.0f, 1.0f);
    intensity[2] = clampf(intensity[2] * 0.8f + downPressed  * 0.2f + (downPressed  - 0.5f) * 0.1f,  0.0f, 1.0f);
    intensity[3] = clampf(intensity[3] * 0.8f + spacePressed * 0.2f + (spacePressed - 0.5f) * 0.1f,  0.0f, 1.0f);
    int8_t gear = 0x7f;
    if (currentKey['0'] == GFUI_KEY_DOWN) { gear = 0; };
    if (currentKey['1'] == GFUI_KEY_DOWN) { gear = 1; };
    if (currentKey['2'] == GFUI_KEY_DOWN) { gear = 2; };
    if (currentKey['3'] == GFUI_KEY_DOWN) { gear = 3; };
    if (currentKey['4'] == GFUI_KEY_DOWN) { gear = 4; };
    if (currentKey['5'] == GFUI_KEY_DOWN) { gear = 5; };
    if (currentKey['6'] == GFUI_KEY_DOWN) { gear = 6; };
    if (currentKey['7'] == GFUI_KEY_DOWN) { gear = -1; };
    if (gear != 0x7f) {
        cansend(canSocket[index][CAN_BUS_CTRLOUT], CAN_ID_CMD_GEAR, &gear, 1);
    }
    int8_t steerCtrl = (int8_t) (intensity[0] * 127.0f);
    uint8_t accelCtrl = (uint8_t) (intensity[1] * 255.0f);
    uint8_t brakeCtrl = (uint8_t) (intensity[2] * 255.0f);
    uint8_t clutchCtrl = (uint8_t) (intensity[3] * 255.0f);
    cansend(canSocket[index][CAN_BUS_CTRLOUT], CAN_ID_CMD_STEER, &steerCtrl, 1);
    cansend(canSocket[index][CAN_BUS_CTRLOUT], CAN_ID_CMD_ACCEL, &accelCtrl, 1);
    cansend(canSocket[index][CAN_BUS_CTRLOUT], CAN_ID_CMD_BRAKE, &brakeCtrl, 1);
    cansend(canSocket[index][CAN_BUS_CTRLOUT], CAN_ID_CMD_CLUTCH, &clutchCtrl, 1);
}
