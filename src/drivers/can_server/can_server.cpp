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
#include "CanServerBot.h"
#include "InputManager.h"

#define NBBOTS 10

const char* botname[NBBOTS] = {"can_server 1", "can_server 2", "can_server 3", "can_server 4", "can_server 5", "can_server 6", "can_server 7", "can_server 8", "can_server 9", "can_server 10"};

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
static void newrace(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s);
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int InitFuncPt(int index, void *pt);

CanServerBot bots[NBBOTS];
InputManager inputs;

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

    return bots[index].init(itf);
}

/* Called for every track change or new race. */
static void
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s)
{
    *carParmHandle = NULL;
    bots[index].initTrack(track, carHandle, carParmHandle, s);
}

/* Start a new race. */
static void
newrace(int index, tCarElt* car, tSituation *s)
{
    bots[index].newrace(car, s);
}

static void
drive(int index, tCarElt* car, tSituation *s)
{
    inputs.attach();
    bots[index].drive(car, s);
}

static void
endrace(int index, tCarElt *car, tSituation *s)
{
    bots[index].endrace(car, s);
}

static void
shutdown(int index)
{
    bots[index].shutdown();
    inputs.detach();
}
