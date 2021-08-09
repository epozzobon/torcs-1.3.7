/***************************************************************************

    file                 : can_server.cpp
    copyright            : (C) 2007 Daniele Loiacono

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
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include "sensors.h"
#include "ObstacleSensors.h"

typedef int SOCKET;
typedef struct sockaddr_in tSockAddrIn;
#define CLOSE(x) close(x)
#define INVALID(x) x < 0

#define NBBUSES 3
#define SENSORS_BUS 0
#define CONTROL_BUS 1
#define NBBOTS 10

#define CAN_ID_ANGLE         0x001
#define CAN_ID_CURLAPTIME    0x002
#define CAN_ID_DAMMAGE       0x003
#define CAN_ID_DISTFROMSTART 0x004
#define CAN_ID_DISTRACED     0x005
#define CAN_ID_FUEL          0x006
#define CAN_ID_GEAR          0x007
#define CAN_ID_LASTLAPTIME   0x008
#define CAN_ID_OPPONENTS     0x009
#define CAN_ID_RACEPOS       0x010
#define CAN_ID_RPM           0x011
#define CAN_ID_SPEEDX        0x012
#define CAN_ID_SPEEDY        0x013
#define CAN_ID_SPEEDZ        0x014
#define CAN_ID_TRACK         0x015
#define CAN_ID_TRACKPOS      0x016
#define CAN_ID_WHEELSPINVEL  0x017
#define CAN_ID_Z             0x018
#define CAN_ID_FOCUS         0x019

#define CAN_ID_CMD_ACCEL     0x401
#define CAN_ID_CMD_BRAKE     0x402
#define CAN_ID_CMD_GEAR      0x403
#define CAN_ID_CMD_STEER     0x404
#define CAN_ID_CMD_CLUTCH    0x405
#define CAN_ID_CMD_FOCUS     0x406

#define RACE_RESTART 1

double __SENSORS_RANGE__;
#define __FOCUS_RANGE__ 200

/*** Noise definitions ***/
#define __NOISE_STD__ 0.1
#define __OPP_NOISE_STD__ 0.02
#define __FOCUS_NOISE_STD__ 0.01

static tTrack	*curTrack;
static int RESTARTING[NBBOTS];

static int cansend(int socket, canid_t identifier, const void *data, size_t size);
static int canpoll(int socket, can_frame *output);
static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int indSENSORS_BUSex);
static int  InitFuncPt(int index, void *pt);

static double normRand(double avg,double std);

/**** variables for CAN ***/
static int canSocket[NBBOTS][NBBUSES];
/************************************************/

static tdble oldAccel[NBBOTS];
static tdble oldBrake[NBBOTS];
static tdble oldSteer[NBBOTS];
static tdble oldClutch[NBBOTS];
static tdble prevDist[NBBOTS];
static tdble distRaced[NBBOTS];

static int oldFocus[NBBOTS];//ML
static int oldGear[NBBOTS];

static Sensors *trackSens[NBBOTS];
static ObstacleSensors *oppSens[NBBOTS];
static Sensors *focusSens[NBBOTS];//ML
static float trackSensAngle[NBBOTS][19];

static const char* botname[NBBOTS] = {"can_server 1", "can_server 2", "can_server 3", "can_server 4", "can_server 5", "can_server 6", "can_server 7", "can_server 8", "can_server 9", "can_server 10"};

static unsigned long total_tics[NBBOTS];

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

	focusSens[index] = new Sensors(car, 5);//ML
	for (int i = 0; i < 5; ++i) {//ML
		focusSens[index]->setSensor(i,(car->_focusCmd)+i-2.0,200);//ML
	}//ML

    // Initialization of track sensors
    trackSens[index] = new Sensors(car, 19);
    for (int i = 0; i < 19; ++i) {
    	trackSens[index]->setSensor(i,trackSensAngle[index][i],__SENSORS_RANGE__);
#ifdef __CAN_SERVER_VERBOSE__
    	std::cout << "Set Track Sensors " << i+1 << " at angle " << trackSensAngle[index][i] << std::endl;
#endif
	}
    // Initialization of opponents sensors
    oppSens[index] = new ObstacleSensors(36, curTrack, car, s, (int) __SENSORS_RANGE__);

    prevDist[index]=-1;
}

/* Drive during race. */
static void
drive(int index, tCarElt* car, tSituation *s)
{

    total_tics[index]++;

    // computing distance to middle
    float dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    // computing the car angle wrt the track axis
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI

	//Update focus sensors' angle
	for (int i = 0; i < 5; ++i) {
		focusSens[index]->setSensor(i,(car->_focusCmd)+i-2.0,200);
	}

    // update the value of track sensors only as long as the car is inside the track
    float trackSensorOut[19];
	float focusSensorOut[5];//ML
    if (dist_to_middle<=1.0 && dist_to_middle >=-1.0 )
    {
        trackSens[index]->sensors_update();
		for (int i = 0; i < 19; ++i)
        {
            trackSensorOut[i] = trackSens[index]->getSensorOut(i);
            if (getNoisy())
            	trackSensorOut[i] *= normRand(1,__NOISE_STD__);
        }
		focusSens[index]->sensors_update();//ML
		if ((car->_focusCD <= car->_curLapTime + car->_curTime)//ML Only send focus sensor reading if cooldown is over
			&& (car->_focusCmd != 360))//ML Only send focus reading if requested by client
		{//ML
			for (int i = 0; i < 5; ++i)
			{
				focusSensorOut[i] = focusSens[index]->getSensorOut(i);
				if (getNoisy())
					focusSensorOut[i] *= normRand(1,__FOCUS_NOISE_STD__);
			}
			car->_focusCD = car->_curLapTime + car->_curTime + 1.0;//ML Add cooldown [seconds]
		}//ML
		else//ML
		{//ML
			for (int i = 0; i < 5; ++i)//ML
			    focusSensorOut[i] = -1;//ML During cooldown send invalid focus reading
		}//ML
    }
    else
    {
        for (int i = 0; i < 19; ++i)
        {
            trackSensorOut[i] = -1;
        }
		for (int i = 0; i < 5; ++i)
		{
			focusSensorOut[i] = -1;
		}
    }

    // update the value of opponent sensors
    float oppSensorOut[36];
    oppSens[index]->sensors_update(s);
    for (int i = 0; i < 36; ++i)
    {
        oppSensorOut[i] = oppSens[index]->getObstacleSensorOut(i);
        if (getNoisy())
        	oppSensorOut[i] *= normRand(1,__OPP_NOISE_STD__);
    }

    float wheelSpinVel[4];
    for (int i=0; i<4; ++i)
    {
        wheelSpinVel[i] = car->_wheelSpinVel(i);
    }

    if (prevDist[index]<0)
    {
	    prevDist[index] = car->race.distFromStartLine;
    }
    float curDistRaced = car->race.distFromStartLine - prevDist[index];
    prevDist[index] = car->race.distFromStartLine;
    if (curDistRaced>100)
    {
	    curDistRaced -= curTrack->length;
    }
    if (curDistRaced<-100)
    {
	    curDistRaced += curTrack->length;
    }

    distRaced[index] += curDistRaced;

    /**********************************************************************
     ****************** Building state string *****************************
     **********************************************************************/

    float speed_x = float(car->_speed_x  * 3.6);
    float speed_y = float(car->_speed_y  * 3.6);
    float speed_z = float(car->_speed_z  * 3.6);
    float curLapTime = float(car->_curLapTime);
    float pos_z = car->_pos_Z  - RtTrackHeightL(&(car->_trkPos));
    float lastLapTime = float(car->_lastLapTime);
    int dammage = getDamageLimit() ? car->_dammage : car->_fakeDammage;

    cansend(canSocket[index][SENSORS_BUS], CAN_ID_ANGLE, &angle, sizeof(angle));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_CURLAPTIME, &curLapTime, sizeof(curLapTime));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_DAMMAGE, &dammage, sizeof(dammage));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_DISTFROMSTART, &car->race.distFromStartLine, sizeof(car->race.distFromStartLine));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_DISTRACED, &distRaced[index], sizeof(distRaced[index]));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_FUEL, &car->_fuel, sizeof(car->_fuel));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_GEAR, &car->_gear, sizeof(car->_gear));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_LASTLAPTIME, &lastLapTime, sizeof(lastLapTime));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_OPPONENTS, &oppSensorOut, sizeof(oppSensorOut));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_RACEPOS, &car->race.pos, sizeof(car->race.pos));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_RPM, &car->_enginerpm, sizeof(car->_enginerpm));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_SPEEDX, &speed_x, sizeof(speed_x));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_SPEEDY, &speed_y, sizeof(speed_y));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_SPEEDZ, &speed_z, sizeof(speed_z));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_TRACK, &trackSensorOut, sizeof(trackSensorOut));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_TRACKPOS, &dist_to_middle, sizeof(dist_to_middle));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_WHEELSPINVEL, &wheelSpinVel, sizeof(wheelSpinVel));
    cansend(canSocket[index][SENSORS_BUS], CAN_ID_Z, &pos_z, sizeof(pos_z));
	cansend(canSocket[index][SENSORS_BUS], CAN_ID_FOCUS, &focusSensorOut, sizeof(focusSensorOut));//ML

    if (RESTARTING[index]==0)
    {
#ifdef __CAN_SERVER_VERBOSE__
        std::cout << "Sending: " << line << std::endl;
#endif
        can_frame frame;
        while (0 < canpoll(canSocket[index][CONTROL_BUS], &frame))
        {
            // Set controls command and store them in variables
            switch (frame.can_id) {
                case CAN_ID_CMD_ACCEL:  memcpy(&car->_accelCmd,  frame.data, sizeof(car->_accelCmd)  ); break;
                case CAN_ID_CMD_BRAKE:  memcpy(&car->_brakeCmd,  frame.data, sizeof(car->_brakeCmd)  ); break;
                case CAN_ID_CMD_GEAR:   memcpy(&car->_gearCmd,   frame.data, sizeof(car->_gearCmd)   ); break;
                case CAN_ID_CMD_STEER:  memcpy(&car->_steerCmd,  frame.data, sizeof(car->_steerCmd)  ); break;
                case CAN_ID_CMD_CLUTCH: memcpy(&car->_clutchCmd, frame.data, sizeof(car->_clutchCmd) ); break;
                case CAN_ID_CMD_FOCUS:  memcpy(&car->_focusCmd,  frame.data, sizeof(car->_focusCmd)  ); break;
                default: break;
            }

            oldAccel[index] = car->_accelCmd;
            oldBrake[index] = car->_brakeCmd;
            oldGear[index] = car->_gearCmd;
            oldSteer[index] = car->_steerCmd;
            oldClutch[index] = car->_clutchCmd;
            oldFocus[index] = car->_focusCmd;

            std::cout <<
                car->_accelCmd << " " <<
                car->_brakeCmd << " " <<
                car->_gearCmd << " " <<
                car->_steerCmd << " " <<
                car->_clutchCmd << " " <<
                car->_focusCmd << std::endl;
        }
    }

    car->_accelCmd = oldAccel[index];
    car->_brakeCmd = oldBrake[index];
    car->_gearCmd  = oldGear[index];
    car->_steerCmd = oldSteer[index];
    car->_clutchCmd = oldClutch[index];
    car->_focusCmd = oldFocus[index];
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
	if (focusSens[index] != NULL)//ML
    {
        delete focusSens[index];
        focusSens[index] = NULL;
    }
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
    if (RESTARTING[index]!=1)
    {
        if (cansend(canSocket[index][SENSORS_BUS], 0x100, "shutdown", 8) < 0)
            std::cerr << "Error: cannot send shutdown message";
    }
    else
    {
        if (cansend(canSocket[index][SENSORS_BUS], 0x101, "restart", 7) < 0)
            std::cerr << "Error: cannot send restart message";
    }
    RESTARTING[index]=0;
    if (trackSens[index] != NULL)
    {
        delete trackSens[index];
        trackSens[index] = NULL;
    }
	if (focusSens[index] != NULL)//ML
    {
        delete focusSens[index];
        focusSens[index] = NULL;
    }
    if (oppSens[index] != NULL)
    {
        delete oppSens[index];
        oppSens[index] = NULL;
    }
    for (int busIndex = 0; busIndex < NBBUSES; busIndex++) {
        CLOSE(canSocket[index][busIndex]);
    }
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
