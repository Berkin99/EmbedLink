/*
 *       ______          __             ____    _       __
 *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
 *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
 *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
 *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
 *
 *  EmbedLink Firmware
 *  Copyright (c) 2024 Yeniay RD, All rights reserved.
 *  _________________________________________________________
 *
 *  EmbedLink Firmware is free software: you can redistribute
 *  it and/or  modify it under  the  terms of the  GNU Lesser
 *  General Public License as  published by the Free Software
 *  Foundation,  either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  EmbedLink  Firmware is  distributed  in the  hope that it
 *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with EmbedLink Firmware. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "xmath3d.h"
#include "xmath.h"
#include "pid.h"
#include "control_height.h"
#include "kinematics.h"
#include "nrx.h"

#ifdef CONTROL_HEIGHT_VELOCITY
/* Velocity PID */
#define PID_VELOCITY {0.46f, 0.00015f, 0.003f}	/* Velocity  PID Values    */

static pidHandle_t hpidVz;				        /* Velocityz PID Handle    */
static pid_t pid_vz = PID_VELOCITY;
#else

/* Height PID */
#define PID_HEIGHT    {540, 1, 900}	    /* height PID Values    */
#define HOWER_POWER   500 				/* Howering Power 12.0V */

static  pidHandle_t hpidHeight;     	/* height PID Handle    */
static  pid_t pidHeight = PID_HEIGHT;
#endif

void controlInitHEIGHT (void){

#ifdef CONTROL_HEIGHT_VELOCITY
	/* Velocity Z */
	pidInit(&hpidVz);
	hpidVz.coefficient = pid_vz;
	hpidVz.iLimit = 0.08;
	hpidVz.dt = 0.004;
#else
	/* HeightZ PID Initialize */
	pidInit(&hpidHeight);
	hpidHeight.coefficient = pidHeight;
	hpidHeight.iLimit = 50;
	hpidHeight.dt = 0.004;
#endif
}

/* IN  : Target Z in meters
 * OUT : Power [0 ,1]  (constrained to [0.2, 0.8])
 * [!] Run this Function with fixed rate
 * [!] Need to add Hovering Power
 * */
float  controlTaskHEIGHT  (float height){

#ifdef CONTROL_HEIGHT_VELOCITY
	/* VelocityZ Based Control */
	float targetvz = (height - kinematicsState()->position.z) / 2;
	targetvz = clampf32(targetvz, -1.25f, 1.25f);
	float p = pidUpdate(&hpidVz, kinematicsState()->velocity.z, targetvz);
	p += 0.480f; /* HOWER POWER 11.1V */
	return clampf32(p, 0.2f, 0.8f);

#else
	/* PositionZ Based Control */
	float power = 0.0f;
	float error = height - xkinematicsState()->position.z;

	power += error * pidHeight.kp;
	hpidHeight.integral += error * pidHeight.ki * hpidHeight.dt;
	hpidHeight.integral = clampf32(hpidHeight.integral, -hpidHeight.iLimit, hpidHeight.iLimit);

	power += hpidHeight.integral;
	float derivative = xkinematicsState()->velocity.z * pidHeight.kd;
	power -= derivative;
	power += HOWER_POWER; /* HOWER POWER 11.1V  [0, 1000] */

	vec_t up = vnew(0.0, 0.0, 1.0);
	up = kinematicsRotateFrame(up, xkinematicsState()->rotation.v);
	power = power / up.z;

	power  = clampf32(power, 300, 800);
	power /= 1000.0f;

	return power;

#endif
}

void   controlResetHEIGHT (void){
#ifdef CONTROL_HEIGHT_VELOCITY
	pidReset(&hpidVz);
#else
	pidReset(&hpidHeight);
#endif
}

// NRX_GROUP_START(pidheight)
// NRX_ADD(NRX_FLOAT, kp, &pidHeight.kp)
// NRX_ADD(NRX_FLOAT, ki, &pidHeight.ki)
// NRX_ADD(NRX_FLOAT, kd, &pidHeight.kd)
// NRX_GROUP_STOP(pidheight)
