/*
	diy-VT100 firmware

	Source code of diy-VT100 firmware.
	Run on STM32F767VxTx.

	URL: https://www.madresistor.com/diy-vt100
	Hardware design: https://gitlab.com/madresistor/diy-vt100-hardware
*/

/*
 * bit-array.h
 *
 *  Created on: 7.05.2020 г.
 *      Author: Simo
 */

#ifndef INC_BIT_ARRAY_H_
#define INC_BIT_ARRAY_H_

#include <stdint.h>
#include <stdbool.h>

#define mask( index )   ( 1<<( 7 & index ) )
#define offset( index ) ( index>>3 )

void bitarr_write( uint8_t *data, uint16_t index, bool value );
bool bitarr_read( const uint8_t *data, uint16_t index );
void bitarr_flip( uint8_t *data, uint16_t index );
void bitarr_high( uint8_t *data, uint16_t index );
void bitarr_low( uint8_t *data, uint16_t index );

#endif /* INC_BIT_ARRAY_H_ */
