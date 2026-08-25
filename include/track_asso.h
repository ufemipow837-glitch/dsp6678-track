#ifndef _TRACK_ASSO_H_
#define _TRACK_ASSO_H_
#include "struct.h"

#pragma pack(4)
void track_asso(struct target_track (*target_data),
                struct reliable (*reliable_track),
                int (*reliable_track_num));


#endif 
