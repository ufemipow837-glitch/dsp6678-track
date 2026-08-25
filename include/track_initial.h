#ifndef _INITIALVOID_H_
#define _INITIALVOID_H_
#include "struct.h"
#include "track.h"

#pragma pack(4)
void get_asso_info1(struct target_track (*target_data),
					struct temp_track (*temp_track),
					int (*loop_of_track),
					int (*track_asso_num),
					float (*d));

void get_asso_info2(struct target_track (*target_data),
					struct temp_track (*temp_track),
					int (*loop_of_track),
					int (*track_asso_num),
					float (*alpha1));

void get_track_asso1(struct target_track (*target_data),
					struct temp_track (*temp_track),
					int (*loop_of_track),
					int (*track_asso_num),
					struct temp_track (*track_asso_data));

void get_track_asso2(struct target_track (*target_data),
					struct temp_track (*temp_track),
					int (*loop_of_track),
					int (*track_asso_num),
					struct temp_track (*track_asso_data));

void get_track_begin(struct target_track *current_dot,
					int (*track_asso_num),
					struct temp_track (*track_asso_data));

void track_initial(struct target_track (*target_data), //
				struct temp_track (*temp_track), //
				int (*temp_track_num),
				struct temp_track (*track_asso_data),
				int (*track_asso_num));


#endif
