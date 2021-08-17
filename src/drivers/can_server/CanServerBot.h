#include <robot.h>
#include "dbc_funcs.h"
#include "sensors.h"
#include "ObstacleSensors.h"


#define NBBUSES 3


class CanServerBot {
private:
    tTrack *curTrack;
    tRobotItf *robot;
    tdble prevDist;
    tdble distRaced;
    can_obj_dbc_funcs_h_t can_obj_dbc_funcs;
    int canSocket[NBBUSES];
    Sensors *trackSens;
    ObstacleSensors *oppSens;
    unsigned long total_tics;

    // Input variables
    float steerInput=0, accelInput=0, brakeInput=0, clutchInput=0;
    uint8_t gearInput = 0;
    bool gearWasSwitching = false;

    // Private methods
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
