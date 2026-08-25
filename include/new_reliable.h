/*
 * renew_reliable.h
 *
 *  Created on: 2023??1??3??
 *      Author: 22431
 */

#ifndef NEW_RELIABLE_H_
#define NEW_RELIABLE_H_

#pragma pack(4)

void new_reliable(struct reliable (*reliable_track),
                    struct temp_track (*temp_track),
                    struct temp_track (*track_asso_data),
                    int (*reliable_track_num),
					int (*reliable_num_all),
					int (*temp_track_num),
				    int (*track_asso_num));



#endif /* RENEW_RELIABLE_H_ */
