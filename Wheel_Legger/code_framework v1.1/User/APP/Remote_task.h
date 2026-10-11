#ifndef __REMOTE_TASK_H
#define __REMOTE_TASK_H

#include "main.h"
#include "INS_task.h"


#define REMOTE_TASK_PERIOD_MS 1U

extern volatile uint8_t remote_gimbal_rearm_required;
void Remote_ControlStep(void);
extern void remote_task(void);

#endif