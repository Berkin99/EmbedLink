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

#ifndef GEOCONFIG_H_
#define GEOCONFIG_H_

#define GRAVITY 			 	9.81f
#define CONST_SEA_PRESSURE 		1013.4f          /* Location based */
#define CONST_PF 				0.1902630958f    /* (1/5.25588f) Pressure factor */
#define CONST_PF2 				44330.0f
#define FIX_TEMP 				25.0f            /* Fixed Temperature */
#define MAGNETIC_DECLINATION 	(6.18f)          /* Magnetic north and Geographic north difference, Location based */
#define EARTH_EQX_R 			6371000          /* Equator radius in meters */
#define EARTH_POLAR_R 			6357000          /* Polar radius in meters*/

#endif /* GEOCONFIG_H_ */
