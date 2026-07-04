#ifndef __UNICOMM_H
#define __UNICOMM_H

#include <stdint.h>

#define UNICOMM_ACTION_CODE_MAX_LEN 20U

typedef enum
{
    UNICOMM_STAGE_NONE = 0,
    UNICOMM_STAGE_MEASURING,
    UNICOMM_STAGE_ACTIONING,
} UniCommStage_e;

extern float current_x;
extern float current_y;
extern float target_x;
extern float target_y;
extern float upstream_current_yaw;
extern float upstream_goal_yaw;
extern uint8_t nav_current_valid;
extern uint8_t nav_goal_valid;

extern UniCommStage_e unicomm_latest_stage;
extern uint32_t unicomm_frame_seq;
extern uint8_t unicomm_current_tag_id;
extern uint8_t unicomm_has_current_tag;
extern float unicomm_x_m;
extern float unicomm_y_m;
extern float unicomm_bearing_deg;
extern float unicomm_yaw_deg;
extern float unicomm_margin;
extern char unicomm_action_code[UNICOMM_ACTION_CODE_MAX_LEN + 1U];

void UniComm_UART7_Init(void);

#endif
