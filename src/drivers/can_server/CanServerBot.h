#include <robot.h>
#include "dbc_funcs.h"
#include "sensors.h"
#include "ObstacleSensors.h"


#define NBWHEELS 4
#define NBBUSES 3
#define NBTRACKSENSORS 19
#define NBOBSTACLESENSORS 36


class CanServerBot {
private:
    int RESTARTING;

    tTrack *curTrack;
    tRobotItf *robot;
    tdble prevDist;
    tdble distRaced;
    can_obj_dbc_funcs_h_t can_obj_dbc_funcs;
    int canSocket[NBBUSES];
    Sensors *trackSens;
    ObstacleSensors *oppSens;
    float trackSensAngle[NBTRACKSENSORS];
    unsigned long total_tics;
    int can_update();

public:
    CanServerBot();
    int init(tRobotItf *itf);
    void initTrack(tTrack* track, void *carHandle, void **carParmHandle, tSituation *s);
    void newrace(tCarElt* car, tSituation *s);
    void drive(tCarElt* car, tSituation *s);
    void endrace(tCarElt *car, tSituation *s);
    void shutdown();
};
