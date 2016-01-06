/*
 * cph.h
 *
 *  Created on: Dec 11, 2015
 *      Author: jcobb
 */

#ifndef SRC_CPH_CPH_H_
#define SRC_CPH_CPH_H_


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#include <asf.h>

//#define MAIN_TEST

#include <conf_board.h>
#include <cph_clock.h>


#define SOFTWARE_VER_STRING  	"Version 2.01    "




#endif /* SRC_CPH_CPH_H_ */
