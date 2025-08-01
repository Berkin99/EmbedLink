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

#ifndef UAVCOM_H_
#define UAVCOM_H_

#include <stdint.h>

void uavcomInit(void);
void uavcomUpdate(uint8_t *pBuffer);

void uavIDLE(void);
void uavMANUAL(void);
void uavHEIGHT(void);
void uavAUTO(void);
void uavTAKEOFF(void);
void uavLAND(void);

#endif /* UAVCOM_H_ */
