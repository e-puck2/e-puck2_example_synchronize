
/*
    Copyright 2007 Alexandre Campo, Alvaro Gutierrez, Valentin Longchamp.

    This file is part of libIrcom.

    libIrcom is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    libIrcom is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libIrcom.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "usbcfg.h"
#include "ircom/ircom.h"
#include "ircom/ircomTools.h"
#include "ircom/ircomReceive.h"
#include "ircom/ircomMessages.h"
#include "ircom/ircomSend.h"
#include "epuck1x/motor_led/advance_one_timer/e_led.h"
#include "selector.h"
#include "motors.h"
#include "leds.h"

#define NB_NEIGHBOURS 8

//number of motor steps to make a complete turn
#define NUMBER_STEPS_FULL_TURN 1292.682914927 
#define ATTRACTION_DIST 25.0

#define NEIGHBOUR_TTL 30000
#define COM_CYCLE_SPEED 6000
#define TURNSPEED 100.0
#define SPEED 0.0
#define MOVE_THRESOLD 0.0
#define ORIENTATION_DELTA 5.0 * M_PI / 180.0

typedef struct
{
    int id;
    float angle;
    int sensor;
    float distance;
    long lastSeen;
} Neighbour;

Neighbour neighbours[NB_NEIGHBOURS];

float sensorDir[8] = {0.2967, 0.8727, 1.5708, 2.6180, 3.6652, 4.7124, 5.4105, 5.9865};

int id;
//float direction = 0.0;

float direction = 0.0;
double turnspeed = TURNSPEED;
double speed = SPEED;

float attractionX;
float attractionY;
float orientationX;
float orientationY;

int nbNeighboursClose;
int nbNeighboursFar;

long int lastClock;

void initRandomNumberGenerator(void);
int getRand(int a, int b);
void AddNeighbour(int id, float angle, int dir, float distance);
void processNewMessage(IrcomMessage* msg);
void sendData(int msg);
void sendAngleToNeighbours(void);
void sendId(void);
void sendAngleToId(int value);

void move(void);
void perception(void);
void updateDirections(float angle);

static THD_WORKING_AREA(move_thd_wa, 128);
static THD_FUNCTION(move_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);
    systime_t time;
    while(1) {
    	time = chVTGetSystemTime();
    	move();
    	chThdSleepUntilWindowed(time, time + MS2ST(250)); //4 Hz
    }
}

static THD_WORKING_AREA(sync_thd_wa, 128);
static THD_FUNCTION(sync_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

    while(1)
    {
		int messageReceived = 0;
		while(((ircomGetTime() - lastClock < COM_CYCLE_SPEED) || (ircomIsReceiving() == 1)))
		{
		    IrcomMessage msg;
		    ircomPopMessage(&msg);

		    if (msg.error == 0)
		    {
				processNewMessage(&msg);
				messageReceived = 1;
		    }
		}
		e_set_body_led(messageReceived);

		sendId();
		sendAngleToNeighbours();
		lastClock = ircomGetTime();

		e_set_body_led(0);
    }

    ircomStop();
}

void startSynchronize(void) {

    // initialise buffer 
    int k;
    for (k = 0; k < NB_NEIGHBOURS; k++) {
    	neighbours[k].id = -1;
    }
    
    
    if (SDU1.config->usbp->state == USB_ACTIVE) { // Skip printing if port not opened.
    	chprintf((BaseSequentialStream *)&SDU1, "-OK-\r\n");
    }

    // initialize ircom, then rng and start listening    
    ircomStart();
    ircomEnableContinuousListening();
    initRandomNumberGenerator();
    // after rng init we can disable prox sensors
    ircomDisableProximity();
    ircomEnableContinuousListening();
    ircomListen();
    
    id = get_selector();

    ircomResetTime();
    lastClock = ircomGetTime();

    // advertise current direction
    e_led_clear();
    e_set_led(0,1);
    e_set_led(4,1);
    
    chThdCreateStatic(sync_thd_wa, sizeof(sync_thd_wa), NORMALPRIO, sync_thd, NULL);

    // activate movement
    //e_activate_agenda(move, 2500); //millisec/10 => 250 ms
    chThdCreateStatic(move_thd_wa, sizeof(move_thd_wa), NORMALPRIO, move_thd, NULL);

}

void sendId ()
{
    int msg;
    //MESSAGE Broadcast: msg(0:3) = 1111, msg(4:7) = ID
    msg = (id << 4) + 0xF;
    
    //Send frame
    sendData(msg);
}

void sendAngleToNeighbours ()
{
    int i;
    for(i = 0; i < NB_NEIGHBOURS; i++)
    {	
		if(neighbours[i].id >= 0)
		{
		    // received a message no more than 2 seconds ago 
		    if (ircomGetTime() - neighbours[i].lastSeen < NEIGHBOUR_TTL)
		    {
				sendAngleToId(i);
		    }
		}
    }
}

void sendAngleToId(int i)
{
    int msg = neighbours[i].id;

    // Compute angle between angle direction detected and goal direction
    float angle = direction - neighbours[i].angle;

    // put it in -pi:pi 
    while (angle < -M_PI) angle += 2.0 * M_PI;
    while (angle >= M_PI) angle -= 2.0 * M_PI;

    // put it in 0:2pi
    if (angle < 0.0)
	angle += 2.0 * M_PI;

    // Merge the message: msg(0:3) = neighbour id , msg(4:7) = angle in 15 slices
    msg = (msg << 4) + (int) (angle * 2.387324146); // 15.0 / M_PI / 2.0); // => (angle/2*M_PI)*15
   
    // Send frame
    sendData(msg);
}

void sendData(int msg)
{    
    // send the msg
    ircomSend((long int) msg);

    //wait for sending done    
    while (ircomSendDone() == 0);

}

void processNewMessage (IrcomMessage* msg)
{	
    int n = (int) msg->value;
    int aux = n & 0xF;
    int nid = n >> 4;

    if (aux == 0xF)
    {
		if (nid != id)
		{
		    AddNeighbour(nid, msg->direction, msg->receivingSensor, msg->distance); 	
		}
    }
    else
    {
		if (nid == id)
		{
		    // convert angle put it in -pi:pi
		    float angle = (float) aux * 0.41887902047; // 2.0 * M_PI / 15.0
		    
		    if (angle >= M_PI)
			angle = angle - 2.0 * M_PI;
		    
		    // move to local mark
		    float ndir = msg->direction;
		    angle = ndir - M_PI + angle;		
		    while (angle < -M_PI) angle += 2.0 * M_PI;
		    while (angle >= M_PI) angle -= 2.0 * M_PI;
		    
		    // update current direction
		    float delta = angle - direction;
		    while (delta < -M_PI) delta += 2.0 * M_PI;
		    while (delta >= M_PI) delta -= 2.0 * M_PI;
		    
		    if (delta > 0)
			direction += ORIENTATION_DELTA;
		    else if (delta < 0)
			direction -= ORIENTATION_DELTA;
		    
		    while (direction < -M_PI) direction += 2.0 * M_PI;
		    while (direction >= M_PI) direction -= 2.0 * M_PI;
		}
    }
}

void AddNeighbour(int id, float angle, int sensor, float distance)
{
    int i;
    int oldest = -1;
    int free = -1;
    long oldestClock = ircomGetTime();

    for (i = 0; i < NB_NEIGHBOURS; i++)
    {
		if(id == neighbours[i].id)
		{
		    neighbours[i].angle = angle;	    
		    neighbours[i].sensor = sensor;
		    neighbours[i].distance = distance;	    
		    neighbours[i].lastSeen = ircomGetTime();
		    return;
		}
		else if(neighbours[i].id == -1)
		{
		    free = i;
		}
		else if(neighbours[i].lastSeen < oldestClock)
		{
		    oldestClock = neighbours[i].lastSeen;
		    oldest = i;
		}
    }

    if (free >= 0) 
	i = free;
    else
	i = oldest;

    neighbours[i].id = id;
    neighbours[i].angle = angle;
    neighbours[i].sensor = sensor;
    neighbours[i].distance = distance;	    
    neighbours[i].lastSeen = ircomGetTime();
}

void initRandomNumberGenerator ()
{
    unsigned int seed = 0;

    long int i;
    for (i = 0; i < 10000; i++)
    {
	if (i % 500)
	{
	    seed += ircom_get_prox(0) ^ ircom_get_prox(1);
	    seed += ircom_get_prox(2) ^ ircom_get_prox(3);
	    seed += ircom_get_prox(4) ^ ircom_get_prox(5);
	    seed += ircom_get_prox(6) ^ ircom_get_prox(7);
	}
    }
    srand(seed);
}

int getRand(int a, int b)
{
    int r;
    r = (double)a + ((double)(rand()) * (double)(b-a) / ((double)RAND_MAX));
    return (int) r;
}


void perception ()
{
    attractionX = 0.0;
    attractionY = 0.0;
    nbNeighboursClose = 0;
    nbNeighboursFar = 0;

    // go through table of neighbours
    int i;
    for (i = 0; i < NB_NEIGHBOURS; i++)
    {
	// entry allocated ?
	if (neighbours[i].id >= 0)
	{
	    // received a message no more than 2 seconds ago 
	    if (ircomGetTime() - neighbours[i].lastSeen < NEIGHBOUR_TTL)
	    {
		// the neighbour is far away and we should drive towards him
		if (neighbours[i].distance >= ATTRACTION_DIST)
		{
		    // add contribution of a neighbour : 
		    attractionX += cos(neighbours[i].angle);
		    attractionY += sin(neighbours[i].angle);		    
		    nbNeighboursFar++;
		}
		// take care of orientation
		else
		{
		    // orientation vector is updated during process of messages received
		    nbNeighboursClose++;
		}
	    }
	}
    }
}

void move()
{
    perception();

    // compute final direction and motor speed  
    speed = 0.0;   
    float desiredAngle = direction;
    
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    // if angle is not big ... don't turn
    if (fabs(desiredAngle) < MOVE_THRESOLD * M_PI / 180.0)
    {
		// direction does not change...
		leftSpeed = speed;
		rightSpeed = speed;
    }    
	    // turn left
	    else if (desiredAngle >= 0.0)
	    {
		leftSpeed = -turnspeed;
		rightSpeed = turnspeed;
	
		// update internal direction according to movement
		float timestep = 0.25;
		float angle = 2.0 * M_PI * turnspeed * timestep / NUMBER_STEPS_FULL_TURN;
		updateDirections(-angle);
    }
    // turn right
    else
    {
		leftSpeed = turnspeed;
		rightSpeed = -turnspeed;
	
		// update internal direction according to movement
		float timestep = 0.25;
		float angle = 2.0 * M_PI * turnspeed * timestep / NUMBER_STEPS_FULL_TURN;
		updateDirections(angle);
    }
    
    // change movement direction
    left_motor_set_speed(leftSpeed);
    right_motor_set_speed(rightSpeed);
}

void updateDirections (float angle)
{
    // update own's global desired direction according to movement
    direction += angle;
    while (direction < -M_PI) direction += 2.0 * M_PI;
    while (direction >= M_PI) direction -= 2.0 * M_PI;

    // update also estimated direction of friends (or neighbours)
    int  i;
    for (i = 0; i < NB_NEIGHBOURS; i++)
    {
	if (neighbours[i].id >= 0)
	{
	    float a = neighbours[i].angle;
	    
	    a += angle;
	    while (a < -M_PI) a += 2.0 * M_PI;
	    while (a >= M_PI) a -= 2.0 * M_PI;

	    int s = (int)((a + M_PI) / (2.0 * M_PI / 8.0));

	    neighbours[i].angle = a;
	    neighbours[i].sensor = s;
	}
    }
}

