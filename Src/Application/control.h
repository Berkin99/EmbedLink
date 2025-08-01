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

#ifndef CONTROL_H_
#define CONTROL_H_

#include "xmath3d.h"

typedef vec_t controlAttitude_t;	/* degrees/s */
typedef vec_t controlVelocity_t;	/* m/s */
typedef vec_t controlPosition_t;	/* meters */
typedef vec_t controlRange_t;		/* [-1, 1] */
typedef float controlPower_t;		/* [ 0, 1] */

typedef enum{
	CONTROL_ATTITUDE,
	CONTROL_VELOCITY,
	CONTROL_POSITION,
	CONTROL_RANGE,
	CONTROL_POWER,
}control_e;

typedef struct{
	control_e type;
	union{
		controlAttitude_t catt;
		controlVelocity_t cvel;
		controlPosition_t cpos;
		controlPower_t    cpow;
		vec_t			  ctrl;
	};
}control_t;

typedef struct{
	controlPower_t	  cpow;
	controlRange_t    crange;
	controlPosition_t cpos;
}controller_t;

#endif /* CONTROL_H_ */
