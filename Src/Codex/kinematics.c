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

#include "systime.h"
#include "kinematics.h"
#include "nrx.h"
#include "matrix.h"

static state_t  transform;
static uint32_t stateTimestamps[STATE_TYPECOUNT];

state_t* kinematicsState(void){
	return &transform;
}

void kinematicsAppend(state_e index, kinv_t data){
	if(index >= STATE_TYPECOUNT) return;
	for (uint8_t i = 0; i < 3; i++) {
		if(data.stdDev.axis[i] != 0){
			transform.kinv[index].axis[i] = data.axis[i];
			transform.kinv[index].stdDev.axis[i] = data.stdDev.axis[i];
		}
	}
	stateTimestamps[index] = millis();
}

void kinematicsStateUpdate(state_t* pState, uint8_t* checkList){
	for (uint8_t i = 0; i < STATE_TYPECOUNT; i++) {
		if(checkList[i] != 0) kinematicsAppend(i, pState->kinv[i]);
	}
}

int8_t kinematicsValidity(state_e index, uint32_t timeout){
	if(index >= STATE_TYPECOUNT) return E_OVERFLOW;
	if(millis() - stateTimestamps[index] > timeout) return E_TIMEOUT;

	int8_t validity  = 0;
	validity = (transform.kinv[index].stdDev.x > 0) << 0 |
			   (transform.kinv[index].stdDev.y > 0) << 1 |
			   (transform.kinv[index].stdDev.z > 0) << 2 ;

	return validity;
}

int8_t kinematicsVector(state_e index, kinv_t* pBuffer){
	*pBuffer = transform.kinv[index];
	return kinematicsValidity(index, KINEMATICS_TIMEOUT_MS);
}

vec_t kinematicsRotateFrame(vec_t v, vec_t frame){
	matrix_t R =  mnew(3, 3);
	matrix_t mv = mnew(3, 1);

	mv.mx[0][0] = v.x;
	mv.mx[1][0] = v.y;
	mv.mx[2][0] = v.z;

	float sx = sinf(frame.x * DEG2RAD);
	float cx = cosf(frame.x * DEG2RAD);
	float sy = sinf(frame.y * DEG2RAD);
	float cy = cosf(frame.y * DEG2RAD);
	float sz = sinf(frame.z * DEG2RAD);
	float cz = cosf(frame.z * DEG2RAD);

	R.mx[0][0] = cy * cz;
	R.mx[0][1] = sx * sy * cz - cx * sz;
	R.mx[0][2] = cx * sy * cz + sx * sz;
	R.mx[1][0] = cy * sz;
	R.mx[1][1] = sx * sy * sz + cx * cz;
	R.mx[1][2] = cx * sy * sz - sx * cz;
	R.mx[2][0] = -sy;
	R.mx[2][1] = sx * cy;
	R.mx[2][2] = cx * cy;

	mv = mdot(R, mv);
	return mkvec(mv.mx[0][0], mv.mx[1][0], mv.mx[2][0]);
}

NRX_GROUP_START(position)
NRX_ADD(NRX_FLOAT, x, &transform.position.x)
NRX_ADD(NRX_FLOAT, y, &transform.position.y)
NRX_ADD(NRX_FLOAT, z, &transform.position.z)
NRX_GROUP_STOP(position)

NRX_GROUP_START(rotation)
NRX_ADD(NRX_FLOAT, x, &transform.rotation.x)
NRX_ADD(NRX_FLOAT, y, &transform.rotation.y)
NRX_ADD(NRX_FLOAT, z, &transform.rotation.z)
NRX_GROUP_STOP(rotation)

NRX_GROUP_START(velocity)
NRX_ADD(NRX_FLOAT, x, &transform.velocity.x)
NRX_ADD(NRX_FLOAT, y, &transform.velocity.y)
NRX_ADD(NRX_FLOAT, z, &transform.velocity.z)
NRX_GROUP_STOP(velocity)

NRX_GROUP_START(acceleration)
NRX_ADD(NRX_FLOAT, x, &transform.acceleration.x)
NRX_ADD(NRX_FLOAT, y, &transform.acceleration.y)
NRX_ADD(NRX_FLOAT, z, &transform.acceleration.z)
NRX_GROUP_STOP(acceleration)

NRX_GROUP_START(attitude)
NRX_ADD(NRX_FLOAT, x, &transform.attitude.x)
NRX_ADD(NRX_FLOAT, y, &transform.attitude.y)
NRX_ADD(NRX_FLOAT, z, &transform.attitude.z)
NRX_GROUP_STOP(attitude)

NRX_GROUP_START(ivelocity)
NRX_ADD(NRX_FLOAT, x, &transform.ivelocity.x)
NRX_ADD(NRX_FLOAT, y, &transform.ivelocity.y)
NRX_ADD(NRX_FLOAT, z, &transform.ivelocity.z)
NRX_GROUP_STOP(ivelocity)

NRX_GROUP_START(iacceleration)
NRX_ADD(NRX_FLOAT, x, &transform.iacceleration.x)
NRX_ADD(NRX_FLOAT, y, &transform.iacceleration.y)
NRX_ADD(NRX_FLOAT, z, &transform.iacceleration.z)
NRX_GROUP_STOP(iacceleration)

NRX_GROUP_START(iattitude)
NRX_ADD(NRX_FLOAT, x, &transform.iattitude.x)
NRX_ADD(NRX_FLOAT, y, &transform.iattitude.y)
NRX_ADD(NRX_FLOAT, z, &transform.iattitude.z)
NRX_GROUP_STOP(iattitude)

NRX_GROUP_START(magnetization)
NRX_ADD(NRX_FLOAT, x, &transform.magnetization.x)
NRX_ADD(NRX_FLOAT, y, &transform.magnetization.y)
NRX_ADD(NRX_FLOAT, z, &transform.magnetization.z)
NRX_GROUP_STOP(magnetization)

NRX_GROUP_START(pressure)
NRX_ADD(NRX_FLOAT, value, &transform.pressure.value)
NRX_GROUP_STOP(pressure)

NRX_GROUP_START(temperature)
NRX_ADD(NRX_FLOAT, value, &transform.temperature.value)
NRX_GROUP_STOP(temperature)
