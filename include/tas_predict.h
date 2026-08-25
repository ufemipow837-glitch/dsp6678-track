/*
 * tas_predict.h
 *
 *  Created on: 2023??0??0??
 *      Author: 22431
 */

#ifndef TAS_PREDICT_H_
#define TAS_PREDICT_H_
#include "track.h"
#pragma pack(4)

void tas_predict(struct MISSIONPARA (*radar),
				struct reliable (*reliable_point),
				int (*reliable_track_num),
				struct MANAGEPARA (*par));
//struct MANAGEPARA* tas_predict(struct MISSIONPARA *radar,
                                //struct reliable *reliable_point,
                                //int (*reliable_track_num),
                                // MANAGEPARA *par);


#endif /* TAS_PREDICT_H_ */
