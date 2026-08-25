/*
 * track.h
 *
 *  Created on: 2023骞�??0鏈�??4鏃�??
 *      Author: 22431
 */

#ifndef TRACK_H_
#define TRACK_H_
#include "struct.h"

#pragma pack(4)


void track(
		struct debugger_track (*track_debugger),
        struct MISSIONPARA (*radar),
		struct TARGETPIONT_1 (*TARGETPIONT_track),
		struct temp_track (*temp_track),
		struct temp_track (*track_asso_data),
        struct reliable (*reliable_track),
		struct TARGETTRACE_RENEW (*track_renew),
		struct TARGETTRACE_END (*track_end),
		struct MANAGEPARA (*par),
		int (*reliable_track_num),
		struct SHOTPOINT (*shotpoint),
		int (*open_USEFLAG));


#endif /* TRACK_H_ */
