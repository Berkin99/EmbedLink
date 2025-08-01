/**
 *    __  __ ____ _  __ ____ ___ __  __
 *    \ \/ // __// |/ //  _// _ |\ \/ /
 *     \  // _/ /    /_/ / / __ | \  /
 *     /_//___//_/|_//___//_/ |_| /_/
 *
 *         Yeniay System Firmware
 *
 *       Copyright (C) 2024 Yeniay
 *
 * This  program  is  free software:   you
 * can  redistribute it  and/or  modify it
 * under  the  terms of  the  GNU  General
 * Public  License as  published  by   the
 * Free Software Foundation, in version 3.
 *
 * You  should  have  received  a  copy of
 * the  GNU  General  Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "uavcom.h"
#include "control.h"
#include "rc_interface.h"
#include "nrx.h"
#include "kinematics.h"
#include "navigation.h"
#include "serial.h"

#define UAV_IDLE     0
#define UAV_MANUAL   1
#define UAV_HEIGHT   2
#define UAV_AUTO     3
#define UAV_TAKEOFF  4
#define UAV_LAND     5

static control_t     empty;
static uint8_t*      input;
static RC_Handle_t   rc;
static uint8_t       refset;
static vec_t 		 ref;

void uavcomInit(void){
	rc = RC_NewHandle();
	rc.state = RC_ARMED;
	ref = vzero();
	refset = 1;
	empty.type = CONTROL_ATTITUDE;
}

void uavcomUpdate(uint8_t *pBuffer){
//	serialPrint("[>] UAV %d:%d:%d:%d:%d \n", pBuffer[0], pBuffer[1], pBuffer[2], pBuffer[3], pBuffer[4]); /* Debug */

	input = &pBuffer[1];
	if(refset == 1){navigationPositionOrigin(ref); refset = 0;}

	switch(pBuffer[0]){
		case UAV_IDLE: 	  uavIDLE();    break;
		case UAV_MANUAL:  uavMANUAL();  break;
		case UAV_HEIGHT:  uavHEIGHT();  break;
		case UAV_AUTO: 	  uavAUTO();    break;
		case UAV_TAKEOFF: uavTAKEOFF(); break;
		case UAV_LAND:    uavLAND();    break;
		default:          uavIDLE();    break;
	}
}

void uavIDLE(void){
	controllerTimeUpdate();
	controllerModeSet(CONTROL_MODE_IDLE);
}

void uavMANUAL(void){
	RC_Update(&rc, input);
	controllerModeSet(CONTROL_MODE_MANUAL);
}

void uavHEIGHT(void){
	RC_Update(&rc, input);
	controllerModeSet(CONTROL_MODE_0);
}

void uavAUTO(void){
	control_t ctrl;
	ctrl.type = CONTROL_POSITION;
	ctrl.cpos.x = *((float*)&input[0]);
	ctrl.cpos.y = *((float*)&input[4]);
	ctrl.cpos.z = *((float*)&input[8]);
	controllerUpdate(ctrl);
	controllerModeSet(CONTROL_MODE_1);
}

void uavTAKEOFF(void){
	controllerTimeUpdate();
	controllerModeSet(CONTROL_MODE_2);
}

void uavLAND(void){
	controllerTimeUpdate();
	controllerModeSet(CONTROL_MODE_3);
}

NRX_GROUP_START(uavcom)
NRX_ADD(NRX_UINT8, refset, &refset)
NRX_ADD(NRX_FLOAT, ref.x,  &ref.x)
NRX_ADD(NRX_FLOAT, ref.y,  &ref.y)
NRX_ADD(NRX_FLOAT, ref.z,  &ref.z)
NRX_GROUP_STOP(uavcom)
