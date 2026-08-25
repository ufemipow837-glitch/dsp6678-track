/*
 * track_die.h
 *
 *  Created on: 2023骞�??0鏈�??2鏃�??
 *      Author: mxq
 */

#ifndef TRACK_DIE_H_
#define TRACK_DIE_H_
#include "struct.h"

#pragma pack(4)
void track_die(struct reliable (*reliable_point), 
				struct TARGETTRACE_END (*track_end),
				int (*reliable_track_num),
				struct SHOTPOINT (*shotpoint));



#endif /* TRACK_DIE_H_ */
