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

#include "CanServerBot.h"
#include "InputManager.h"


#define CAN_BUS_SENSORS 0
#define CAN_BUS_CONTROL 1
#define CAN_BUS_CTRLOUT 2

#define NB_CAN_ID_SENSOR 16
#define NB_CAN_ID_CONTROL 1

double __SENSORS_RANGE__;

static const canid_t can_tx_msgboxes_ids_sensor[NB_CAN_ID_SENSOR] = {
    0x400, 0x401, 0x402, 0x403, 0x404, 0x405, 0x406, 0x407,
    0x300, 0x301, 0x302, 0x303, 0x304, 0x500, 0x501, 0x502
};

static const canid_t can_tx_msgboxes_ids_control[NB_CAN_ID_CONTROL] = {
    0x80,
};

int CanServerBot::init(tRobotItf *itf) {
    this->robot = itf;
    return 0;
}

static int cansend(int socket, canid_t identifier, const void *data, size_t size);
static int canpoll(int socket, can_frame *output);
static float clampf(float x, float m, float M);


CanServerBot::CanServerBot() {
    
}

void CanServerBot::initTrack(tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) {
    this->curTrack = track;
}

void CanServerBot::newrace(tCarElt* car, tSituation *s) {
    this->total_tics=0;

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
        std::cout << "Opening bus " << busIndex << " on car " << this->robot->index << "\n";
        this->canSocket[busIndex] = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (this->canSocket[busIndex] < 0)
        {
            std::cerr << "Error: cannot create canSocket!";
            exit(1);
        }

        struct ifreq ifr;
        snprintf(ifr.ifr_name, IF_NAMESIZE, "vcan%d%c", this->robot->index, busIndex + (int) 'a');
        ifr.ifr_name[IF_NAMESIZE-1] = 0;
        if (-1 == ioctl(this->canSocket[busIndex], SIOCGIFINDEX, &ifr)) {
            std::cerr << "Error: cannot ioctl canSocket!";
            exit(1);
        }

        struct sockaddr_can addr;
        memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (0 > bind(this->canSocket[busIndex], (struct sockaddr *)&addr, sizeof(addr))) {
            std::cerr << "Error: cannot bind canSocket!";
            exit(1);
        }
    }

    // Initialization of track sensors
    this->trackSens = new Sensors(car, NBTRACKSENSORS);
    for (int i = 0; i < NBTRACKSENSORS; ++i) {
    	this->trackSens->setSensor(i,this->trackSensAngle[i],__SENSORS_RANGE__);
#ifdef __CAN_SERVER_VERBOSE__
    	std::cout << "Set Track Sensors " << i+1 << " at angle " << this->trackSensAngle[i] << std::endl;
#endif
	}
    // Initialization of opponents sensors
    this->oppSens = new ObstacleSensors(NBOBSTACLESENSORS, this->curTrack, car, s, (int) __SENSORS_RANGE__);

    this->prevDist=-1;
}

void CanServerBot::drive(tCarElt* car, tSituation *s) {
    this->total_tics++;

    static float analogInputs[4] = {0, 0, 0, 0};
    static uint8_t currentGear = 0;
    static bool gearWasSwitching = false;
    InputManager *inp = InputManager::getInstance();

    float steerPressed = (inp->isKeyDown('d') ? -1.0f : 0.0f)
                       + (inp->isKeyDown('a') ? 1.0f : 0.0f);
    float upPressed    = inp->isKeyDown('w') ? 1.0f : 0.0f;
    float downPressed  = inp->isKeyDown('s') ? 1.0f : 0.0f;
    float spacePressed = inp->isKeyDown(' ') ? 1.0f : 0.0f;
    analogInputs[0] = clampf(analogInputs[0] * 0.8f + steerPressed * 0.25f, -1.0f, 1.0f);
    analogInputs[1] = clampf(analogInputs[1] * 0.8f + upPressed    * 0.2f + (upPressed    - 0.5f) * 0.1f,  0.0f, 1.0f);
    analogInputs[2] = clampf(analogInputs[2] * 0.8f + downPressed  * 0.2f + (downPressed  - 0.5f) * 0.1f,  0.0f, 1.0f);
    analogInputs[3] = clampf(analogInputs[3] * 0.8f + spacePressed * 0.2f + (spacePressed - 0.5f) * 0.1f,  0.0f, 1.0f);

    bool gearUpPressed    = inp->isKeyDown('e');
    bool gearDownPressed  = inp->isKeyDown('q');
    if (!gearWasSwitching) {
        if (gearUpPressed) {
            currentGear = (currentGear + 1) & 0x7;
        } else if (gearDownPressed) {
            currentGear = (currentGear - 1) & 0x7;
        }
    }
    gearWasSwitching = gearUpPressed || gearDownPressed;

    // computing distance to middle
    float dist_to_middle = 2*car->_trkPos.toMiddle/(car->_trkPos.seg->width);
    // computing the car angle wrt the track axis
    float angle =  RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
    NORM_PI_PI(angle); // normalize the angle between -PI and + PI

    // update the value of track sensors only as long as the car is inside the track
    float trackSensorOut[NBTRACKSENSORS];
    if (dist_to_middle <= 1.0 && dist_to_middle >= -1.0)
    {
        this->trackSens->sensors_update();
		for (int i = 0; i < NBTRACKSENSORS; ++i)
        {
            trackSensorOut[i] = this->trackSens->getSensorOut(i);
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
    this->oppSens->sensors_update(s);
    for (int i = 0; i < NBOBSTACLESENSORS; ++i)
    {
        oppSensorOut[i] = this->oppSens->getObstacleSensorOut(i);
    }

    float wheelSpinVel[NBWHEELS];
    for (int i = 0; i < NBWHEELS; ++i)
    {
        wheelSpinVel[i] = car->_wheelSpinVel(i);
    }

    if (this->prevDist < 0)
    {
	    this->prevDist = car->race.distFromStartLine;
    }
    float curDistRaced = car->race.distFromStartLine - this->prevDist;
    this->prevDist = car->race.distFromStartLine;
    if (curDistRaced > 100)
    {
	    curDistRaced -= curTrack->length;
    }
    if (curDistRaced < -100)
    {
	    curDistRaced += curTrack->length;
    }

    this->distRaced += curDistRaced;


    can_obj_dbc_funcs_h_t *o = &this->can_obj_dbc_funcs;
    {
        encode_can_0x401_Damage(o, getDamageLimit() ? car->_dammage : car->_fakeDammage);
        encode_can_0x405_DistanceFromMiddle(o, clampf(dist_to_middle, -1.0f, 1.0f));
        encode_can_0x405_Angle(o, angle);
        encode_can_0x400_FuelLevel(o, car->_fuel);
        encode_can_0x400_EngineSpeed(o, car->_enginerpm);
        encode_can_0x400_Gear(o, car->_gear);
        encode_can_0x403_SpeedX(o, car->_speed_x);
        encode_can_0x403_SpeedY(o, car->_speed_y);
        encode_can_0x403_SpeedZ(o, car->_speed_z);
        encode_can_0x406_CurrentLapTime(o, car->_curLapTime);
        encode_can_0x406_LastLapTime(o, car->_lastLapTime);
        encode_can_0x407_DistanceFromStartLine(o, car->race.distFromStartLine);
        encode_can_0x407_CurrentDistanceRaced(o, this->distRaced);
        encode_can_0x404_Position(o, car->race.pos);
        encode_can_0x404_PositionZ(o, car->_pos_Z - RtTrackHeightL(&(car->_trkPos)));
        encode_can_0x402_WheelSpin0(o, wheelSpinVel[0] * 60.0f / (2 * PI));
        encode_can_0x402_WheelSpin1(o, wheelSpinVel[1] * 60.0f / (2 * PI));
        encode_can_0x402_WheelSpin2(o, wheelSpinVel[2] * 60.0f / (2 * PI));
        encode_can_0x402_WheelSpin3(o, wheelSpinVel[3] * 60.0f / (2 * PI));
        encode_can_0x300_ObstacleSensor0 (o, oppSensorOut[0 ]);
        encode_can_0x300_ObstacleSensor1 (o, oppSensorOut[1 ]);
        encode_can_0x300_ObstacleSensor2 (o, oppSensorOut[2 ]);
        encode_can_0x300_ObstacleSensor3 (o, oppSensorOut[3 ]);
        encode_can_0x300_ObstacleSensor4 (o, oppSensorOut[4 ]);
        encode_can_0x300_ObstacleSensor5 (o, oppSensorOut[5 ]);
        encode_can_0x300_ObstacleSensor6 (o, oppSensorOut[6 ]);
        encode_can_0x300_ObstacleSensor7 (o, oppSensorOut[7 ]);
        encode_can_0x301_ObstacleSensor8 (o, oppSensorOut[8 ]);
        encode_can_0x301_ObstacleSensor9 (o, oppSensorOut[9 ]);
        encode_can_0x301_ObstacleSensor10(o, oppSensorOut[10]);
        encode_can_0x301_ObstacleSensor11(o, oppSensorOut[11]);
        encode_can_0x301_ObstacleSensor12(o, oppSensorOut[12]);
        encode_can_0x301_ObstacleSensor13(o, oppSensorOut[13]);
        encode_can_0x301_ObstacleSensor14(o, oppSensorOut[14]);
        encode_can_0x301_ObstacleSensor15(o, oppSensorOut[15]);
        encode_can_0x302_ObstacleSensor16(o, oppSensorOut[16]);
        encode_can_0x302_ObstacleSensor17(o, oppSensorOut[17]);
        encode_can_0x302_ObstacleSensor18(o, oppSensorOut[18]);
        encode_can_0x302_ObstacleSensor19(o, oppSensorOut[19]);
        encode_can_0x302_ObstacleSensor20(o, oppSensorOut[20]);
        encode_can_0x302_ObstacleSensor21(o, oppSensorOut[21]);
        encode_can_0x302_ObstacleSensor22(o, oppSensorOut[22]);
        encode_can_0x302_ObstacleSensor23(o, oppSensorOut[23]);
        encode_can_0x303_ObstacleSensor24(o, oppSensorOut[24]);
        encode_can_0x303_ObstacleSensor25(o, oppSensorOut[25]);
        encode_can_0x303_ObstacleSensor26(o, oppSensorOut[26]);
        encode_can_0x303_ObstacleSensor27(o, oppSensorOut[27]);
        encode_can_0x303_ObstacleSensor28(o, oppSensorOut[28]);
        encode_can_0x303_ObstacleSensor29(o, oppSensorOut[29]);
        encode_can_0x303_ObstacleSensor30(o, oppSensorOut[30]);
        encode_can_0x303_ObstacleSensor31(o, oppSensorOut[31]);
        encode_can_0x304_ObstacleSensor32(o, oppSensorOut[32]);
        encode_can_0x304_ObstacleSensor33(o, oppSensorOut[33]);
        encode_can_0x304_ObstacleSensor34(o, oppSensorOut[34]);
        encode_can_0x304_ObstacleSensor35(o, oppSensorOut[35]);
        encode_can_0x500_TrackSensor0 (o, trackSensorOut[0 ]);
        encode_can_0x500_TrackSensor1 (o, trackSensorOut[1 ]);
        encode_can_0x500_TrackSensor2 (o, trackSensorOut[2 ]);
        encode_can_0x500_TrackSensor3 (o, trackSensorOut[3 ]);
        encode_can_0x500_TrackSensor4 (o, trackSensorOut[4 ]);
        encode_can_0x500_TrackSensor5 (o, trackSensorOut[5 ]);
        encode_can_0x500_TrackSensor6 (o, trackSensorOut[6 ]);
        encode_can_0x500_TrackSensor7 (o, trackSensorOut[7 ]);
        encode_can_0x501_TrackSensor8 (o, trackSensorOut[8 ]);
        encode_can_0x501_TrackSensor9 (o, trackSensorOut[9 ]);
        encode_can_0x501_TrackSensor10(o, trackSensorOut[10]);
        encode_can_0x501_TrackSensor11(o, trackSensorOut[11]);
        encode_can_0x501_TrackSensor12(o, trackSensorOut[12]);
        encode_can_0x501_TrackSensor13(o, trackSensorOut[13]);
        encode_can_0x501_TrackSensor14(o, trackSensorOut[14]);
        encode_can_0x501_TrackSensor15(o, trackSensorOut[15]);
        encode_can_0x502_TrackSensor16(o, trackSensorOut[16]);
        encode_can_0x502_TrackSensor17(o, trackSensorOut[17]);
        encode_can_0x502_TrackSensor18(o, trackSensorOut[18]);
        encode_can_0x080_Steer(o, analogInputs[0]);
        encode_can_0x080_Accel(o, analogInputs[1]);
        encode_can_0x080_Brake(o, analogInputs[2]);
        encode_can_0x080_Clutch(o, analogInputs[3]);
        encode_can_0x080_Gear(o, currentGear);
    }

    if (this->RESTARTING==0)
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

    if (this->can_update()) {
        std::cout <<
            car->_accelCmd << " " <<
            car->_brakeCmd << " " <<
            car->_gearCmd << " " <<
            car->_steerCmd << " " <<
            car->_clutchCmd << std::endl;
    }
}

void CanServerBot::endrace(tCarElt *car, tSituation *s) {
    this->RESTARTING=0;
    if (trackSens != NULL)
    {
        delete this->trackSens;
        this->trackSens = NULL;
    }

    if (this->oppSens != NULL)
    {
        delete this->oppSens;
        this->oppSens = NULL;
    }
}

void CanServerBot::shutdown() {
    if (this->RESTARTING!=1)
    {
        if (cansend(this->canSocket[CAN_BUS_SENSORS], 0x100, "shutdown", 8) < 0)
            std::cerr << "Error: cannot send shutdown message";
    }
    else
    {
        if (cansend(this->canSocket[CAN_BUS_SENSORS], 0x101, "restart", 7) < 0)
            std::cerr << "Error: cannot send restart message";
    }
    this->RESTARTING=0;
    if (this->trackSens != NULL)
    {
        delete this->trackSens;
        this->trackSens = NULL;
    }
    if (this->oppSens != NULL)
    {
        delete this->oppSens;
        this->oppSens = NULL;
    }
    for (int busIndex = 0; busIndex < NBBUSES; busIndex++) {
        close(this->canSocket[busIndex]);
    }
}

int CanServerBot::can_update() {
    int total_messages = 0;
    int s;
    can_obj_dbc_funcs_h_t *o = &this->can_obj_dbc_funcs;

    s = this->canSocket[CAN_BUS_SENSORS];
    for (int i = 0; i < NB_CAN_ID_SENSOR; i++) {
        uint64_t x;
        pack_message(o, can_tx_msgboxes_ids_sensor[i], &x);
        cansend(s, can_tx_msgboxes_ids_sensor[i], &x, 8);
    }

    s = this->canSocket[CAN_BUS_CTRLOUT];
    for (int i = 0; i < NB_CAN_ID_CONTROL; i++) {
        uint64_t x;
        pack_message(o, can_tx_msgboxes_ids_control[i], &x);
        cansend(s, can_tx_msgboxes_ids_control[i], &x, 8);
    }

    can_frame frame;
    while (0 < canpoll(this->canSocket[CAN_BUS_CONTROL], &frame))
    {
        total_messages++;
        // Set controls command and store them in variables
        unpack_message(o, frame.can_id, *(uint64_t *) &frame.data, frame.len, 0);
    }
    return total_messages;
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
            close(socket);
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

static float clampf(float x, float m, float M) {
    if (x < m) return m;
    if (x > M) return M;
    return x;
}
