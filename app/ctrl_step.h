/**
 * *****************************************************************************
 * @file         ctrl_step.c
 * @brief        stepping motor control
 * @author       NGU
 * @date         20210502
 * @version      1
 * @copyright    Copyright (C) 2021 NGU
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CTRL_STEP_H__
#define __CTRL_STEP_H__

#include "ctrl.h"

#define PICK_INDEX_CLI -18000

#define PICK_FLAG_REVERSE (1 << 0)
#define PICK_FLAG_RUN     (1 << 1)
#define PICK_FLAG_AUTO    (1 << 2)
#define PICK_FLAG_ZERO    (1 << 3)

#define PICK_PWM_DIVIDE 320
#define PICK_PWM_DELTA  400

typedef struct
{
    uint32_t fr;
    uint32_t cnt;
    uint32_t idx;
    uint32_t set;
    int8_t flag;
} ctrl_step_t;

extern ctrl_step_t step;

__BEGIN_DECLS

extern void pick_init(void);
extern void pick_set(int32_t hz);
extern uint32_t pick_set_dir(int32_t offset);
extern void pick_set_freq(uint32_t hz);
extern void pick_zero_cli(int32_t idx);
extern void pick_index(uint32_t idx);
extern void pick_start(int32_t offset);
extern void pick_stop(void);
extern void pick_update(uint32_t inc, uint32_t cnt);

__END_DECLS

/* Enddef to prevent recursive inclusion -------------------------------------*/
#endif /* __CTRL_STEP_H__ */

/************************ (C) COPYRIGHT NGU ********************END OF FILE****/
