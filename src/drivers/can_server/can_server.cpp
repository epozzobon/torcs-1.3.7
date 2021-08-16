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
#include "dbc_funcs.h"


#define NBWHEELS 4
#define NBBUSES 3
#define NBBOTS 10
#define NBOBSTACLESENSORS 36
#define NBTRACKSENSORS 19

#define CAN_BUS_SENSORS 0
#define CAN_BUS_CONTROL 1
#define CAN_BUS_CTRLOUT 2

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
static can_obj_dbc_funcs_h_t can_obj_dbc_funcs;

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
static unsigned can_tx_msgboxes_dlc[NBCANTXMSGBOXES] = {};
static canid_t can_tx_msgboxes_ids[NBCANTXMSGBOXES] = {};

static float clampf(float x, float m, float M) {
    if (x < m) return m;
    if (x > M) return M;
    return x;
}

/*
 * Module entry point
 */
extern "C" int
    can_server(tModInfo *modInfo)
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    can_tx_msgboxes_ids[TXMSGBOX_FUEL_GEAR_RPM] = 0x400;
    can_tx_msgboxes_ids[TXMSGBOX_DAMMAGE] = 0x401;
    can_tx_msgboxes_ids[TXMSGBOX_WHEELSPINVEL] = 0x402;
    can_tx_msgboxes_ids[TXMSGBOX_SPEED] = 0x403;
    can_tx_msgboxes_ids[TXMSGBOX_POS] = 0x404;
    can_tx_msgboxes_ids[TXMSGBOX_ANGLE] = 0x405;
    can_tx_msgboxes_ids[TXMSGBOX_LAPTIME] = 0x406;
    can_tx_msgboxes_ids[TXMSGBOX_DISTRACED] = 0x407;
    can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_A] = 0x300;
    can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_B] = 0x301;
    can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_C] = 0x302;
    can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_D] = 0x303;
    can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_E] = 0x304;
    can_tx_msgboxes_ids[TXMSGBOX_TRACKSENS_A] = 0x500;
    can_tx_msgboxes_ids[TXMSGBOX_TRACKSENS_B] = 0x501;
    can_tx_msgboxes_ids[TXMSGBOX_TRACKSENS_C] = 0x502;

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


    can_obj_dbc_funcs_h_t *o = &can_obj_dbc_funcs;
    {

        encode_can_0x401_Damage(o, getDamageLimit() ? car->_dammage : car->_fakeDammage);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_DAMMAGE], &can_tx_msgboxes[index][TXMSGBOX_DAMMAGE]);

        encode_can_0x405_DistanceFromMiddle(o, clampf(dist_to_middle, -1.0f, 1.0f));
        encode_can_0x405_Angle(o, angle);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_ANGLE], &can_tx_msgboxes[index][TXMSGBOX_ANGLE]);

        encode_can_0x400_FuelLevel(o, car->_fuel);
        encode_can_0x400_EngineSpeed(o, car->_enginerpm);
        encode_can_0x400_Gear(o, car->_gear);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_FUEL_GEAR_RPM], &can_tx_msgboxes[index][TXMSGBOX_FUEL_GEAR_RPM]);

        encode_can_0x403_SpeedX(o, car->_speed_x);
        encode_can_0x403_SpeedY(o, car->_speed_y);
        encode_can_0x403_SpeedZ(o, car->_speed_z);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_SPEED], &can_tx_msgboxes[index][TXMSGBOX_SPEED]);

        encode_can_0x406_CurrentLapTime(o, car->_curLapTime);
        encode_can_0x406_LastLapTime(o, car->_lastLapTime);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_LAPTIME], &can_tx_msgboxes[index][TXMSGBOX_LAPTIME]);

        encode_can_0x407_DistanceFromStartLine(o, car->race.distFromStartLine);
        encode_can_0x407_CurrentDistanceRaced(o, distRaced[index]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_DISTRACED], &can_tx_msgboxes[index][TXMSGBOX_DISTRACED]);

        encode_can_0x404_Position(o, car->race.pos);
        encode_can_0x404_PositionZ(o, car->_pos_Z - RtTrackHeightL(&(car->_trkPos)));
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_POS], &can_tx_msgboxes[index][TXMSGBOX_POS]);

        encode_can_0x402_WheelSpin0(o, wheelSpinVel[0] * 60.0f / (2 * PI));
        encode_can_0x402_WheelSpin1(o, wheelSpinVel[1] * 60.0f / (2 * PI));
        encode_can_0x402_WheelSpin2(o, wheelSpinVel[2] * 60.0f / (2 * PI));
        encode_can_0x402_WheelSpin3(o, wheelSpinVel[3] * 60.0f / (2 * PI));
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_WHEELSPINVEL], &can_tx_msgboxes[index][TXMSGBOX_WHEELSPINVEL]);

        encode_can_0x300_ObstacleSensor0 (o, oppSensorOut[0 ]);
        encode_can_0x300_ObstacleSensor1 (o, oppSensorOut[1 ]);
        encode_can_0x300_ObstacleSensor2 (o, oppSensorOut[2 ]);
        encode_can_0x300_ObstacleSensor3 (o, oppSensorOut[3 ]);
        encode_can_0x300_ObstacleSensor4 (o, oppSensorOut[4 ]);
        encode_can_0x300_ObstacleSensor5 (o, oppSensorOut[5 ]);
        encode_can_0x300_ObstacleSensor6 (o, oppSensorOut[6 ]);
        encode_can_0x300_ObstacleSensor7 (o, oppSensorOut[7 ]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_A], &can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_A]);
        encode_can_0x301_ObstacleSensor8 (o, oppSensorOut[8 ]);
        encode_can_0x301_ObstacleSensor9 (o, oppSensorOut[9 ]);
        encode_can_0x301_ObstacleSensor10(o, oppSensorOut[10]);
        encode_can_0x301_ObstacleSensor11(o, oppSensorOut[11]);
        encode_can_0x301_ObstacleSensor12(o, oppSensorOut[12]);
        encode_can_0x301_ObstacleSensor13(o, oppSensorOut[13]);
        encode_can_0x301_ObstacleSensor14(o, oppSensorOut[14]);
        encode_can_0x301_ObstacleSensor15(o, oppSensorOut[15]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_B], &can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_B]);
        encode_can_0x302_ObstacleSensor16(o, oppSensorOut[16]);
        encode_can_0x302_ObstacleSensor17(o, oppSensorOut[17]);
        encode_can_0x302_ObstacleSensor18(o, oppSensorOut[18]);
        encode_can_0x302_ObstacleSensor19(o, oppSensorOut[19]);
        encode_can_0x302_ObstacleSensor20(o, oppSensorOut[20]);
        encode_can_0x302_ObstacleSensor21(o, oppSensorOut[21]);
        encode_can_0x302_ObstacleSensor22(o, oppSensorOut[22]);
        encode_can_0x302_ObstacleSensor23(o, oppSensorOut[23]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_C], &can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_C]);
        encode_can_0x303_ObstacleSensor24(o, oppSensorOut[24]);
        encode_can_0x303_ObstacleSensor25(o, oppSensorOut[25]);
        encode_can_0x303_ObstacleSensor26(o, oppSensorOut[26]);
        encode_can_0x303_ObstacleSensor27(o, oppSensorOut[27]);
        encode_can_0x303_ObstacleSensor28(o, oppSensorOut[28]);
        encode_can_0x303_ObstacleSensor29(o, oppSensorOut[29]);
        encode_can_0x303_ObstacleSensor30(o, oppSensorOut[30]);
        encode_can_0x303_ObstacleSensor31(o, oppSensorOut[31]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_D], &can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_D]);
        encode_can_0x304_ObstacleSensor32(o, oppSensorOut[32]);
        encode_can_0x304_ObstacleSensor33(o, oppSensorOut[33]);
        encode_can_0x304_ObstacleSensor34(o, oppSensorOut[34]);
        encode_can_0x304_ObstacleSensor35(o, oppSensorOut[35]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_OBSTACLE_SENSORS_E], &can_tx_msgboxes[index][TXMSGBOX_OBSTACLE_SENSORS_E]);

        encode_can_0x500_TrackSensor0 (o, trackSensorOut[0 ]);
        encode_can_0x500_TrackSensor1 (o, trackSensorOut[1 ]);
        encode_can_0x500_TrackSensor2 (o, trackSensorOut[2 ]);
        encode_can_0x500_TrackSensor3 (o, trackSensorOut[3 ]);
        encode_can_0x500_TrackSensor4 (o, trackSensorOut[4 ]);
        encode_can_0x500_TrackSensor5 (o, trackSensorOut[5 ]);
        encode_can_0x500_TrackSensor6 (o, trackSensorOut[6 ]);
        encode_can_0x500_TrackSensor7 (o, trackSensorOut[7 ]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_TRACKSENS_A], &can_tx_msgboxes[index][TXMSGBOX_TRACKSENS_A]);
        encode_can_0x501_TrackSensor8 (o, trackSensorOut[8 ]);
        encode_can_0x501_TrackSensor9 (o, trackSensorOut[9 ]);
        encode_can_0x501_TrackSensor10(o, trackSensorOut[10]);
        encode_can_0x501_TrackSensor11(o, trackSensorOut[11]);
        encode_can_0x501_TrackSensor12(o, trackSensorOut[12]);
        encode_can_0x501_TrackSensor13(o, trackSensorOut[13]);
        encode_can_0x501_TrackSensor14(o, trackSensorOut[14]);
        encode_can_0x501_TrackSensor15(o, trackSensorOut[15]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_TRACKSENS_B], &can_tx_msgboxes[index][TXMSGBOX_TRACKSENS_B]);
        encode_can_0x502_TrackSensor16(o, trackSensorOut[16]);
        encode_can_0x502_TrackSensor17(o, trackSensorOut[17]);
        encode_can_0x502_TrackSensor18(o, trackSensorOut[18]);
        pack_message(o, can_tx_msgboxes_ids[TXMSGBOX_TRACKSENS_C], &can_tx_msgboxes[index][TXMSGBOX_TRACKSENS_C]);
    }

    if (RESTARTING[index]==0)
    {
#ifdef __CAN_SERVER_VERBOSE__
        std::cout << "Sending: " << line << std::endl;
#endif

        double accel, brake, clutch, steer;
        uint8_t gear;

        decode_can_0x080_Accel(o, &accel);
        decode_can_0x080_Brake(o, &brake);
        decode_can_0x080_Clutch(o, &clutch);
        decode_can_0x080_Steer(o, &steer);
        decode_can_0x080_Gear(o, &gear);

        car->_brakeCmd = brake;
        car->_accelCmd = accel;
        car->_clutchCmd = clutch;
        car->_steerCmd = steer;
        car->_gearCmd = "\x00\x01\x02\x03\x04\x05\x06\xff"[gear & 7];
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

static double normRand(double avg,double std)
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
        can_obj_dbc_funcs_h_t *o = &can_obj_dbc_funcs;
        unpack_message(o, frame.can_id, *(uint64_t *) &frame.data, frame.len, 0);
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
    static uint8_t currentGear = 0;
    static bool gearWasSwitching = false;

    if (first_time) {
        first_time = 0;
		GfuiKeyEventRegisterCurrent(onKeyAction);
		GfuiSKeyEventRegisterCurrent(onSKeyAction);
    }

    float steerPressed = (currentKey['d'] == GFUI_KEY_DOWN ? -1.0f : 0.0f)
                       + (currentKey['a'] == GFUI_KEY_DOWN ? 1.0f : 0.0f);
    float upPressed    = currentKey['w'] == GFUI_KEY_DOWN ? 1.0f : 0.0f;
    float downPressed  = currentKey['s'] == GFUI_KEY_DOWN ? 1.0f : 0.0f;
    float spacePressed = currentKey[' '] == GFUI_KEY_DOWN ? 1.0f : 0.0f;
    intensity[0] = clampf(intensity[0] * 0.8f + steerPressed * 0.25f, -1.0f, 1.0f);
    intensity[1] = clampf(intensity[1] * 0.8f + upPressed    * 0.2f + (upPressed    - 0.5f) * 0.1f,  0.0f, 1.0f);
    intensity[2] = clampf(intensity[2] * 0.8f + downPressed  * 0.2f + (downPressed  - 0.5f) * 0.1f,  0.0f, 1.0f);
    intensity[3] = clampf(intensity[3] * 0.8f + spacePressed * 0.2f + (spacePressed - 0.5f) * 0.1f,  0.0f, 1.0f);

    bool gearUpPressed    = currentKey['e'] == GFUI_KEY_DOWN;
    bool gearDownPressed  = currentKey['q'] == GFUI_KEY_DOWN;
    if (!gearWasSwitching) {
        if (gearUpPressed) {
            currentGear = (currentGear + 1) & 0x7;
        } else if (gearDownPressed) {
            currentGear = (currentGear - 1) & 0x7;
        }
    }
    gearWasSwitching = gearUpPressed || gearDownPressed;

    can_obj_dbc_funcs_h_t *o = &can_obj_dbc_funcs;
    encode_can_0x080_Steer(o, intensity[0]);
    encode_can_0x080_Accel(o, intensity[1]);
    encode_can_0x080_Brake(o, intensity[2]);
    encode_can_0x080_Clutch(o, intensity[3]);
    encode_can_0x080_Gear(o, currentGear);
    uint64_t d = 0;
    if (-1 != pack_message(o, 0x80, &d)) {
        cansend(canSocket[index][CAN_BUS_CTRLOUT], 0x80, &d, 5);
    }
}
