/**
 * *****************************************************************************
 * @file         cc_lpf.c/h
 * @brief        Low Pass Filter
 * @author       tqfx
 * @date         20210101
 * @version      1
 * @copyright    Copyright (c) 2021
 * @code         utf-8                                                  @endcode
 * *****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CC_LPF_H__
#define __CC_LPF_H__

/* Includes ------------------------------------------------------------------*/
/* Private includes ----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#undef __BEGIN_DECLS
#undef __END_DECLS

#if defined(__cplusplus)
#define __BEGIN_DECLS \
    extern "C"        \
    {
#define __END_DECLS }
#else
#define __BEGIN_DECLS
#define __END_DECLS
#endif /* __cplusplus */

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    float in;  /*!< input */
    float out; /*!< output */
    float k;   /*!< The parameter of filtering */
    float t;   /*!< The time interval of filtering, unit /s */
} cc_lpf_t;

/* Exported functions prototypes ---------------------------------------------*/

__BEGIN_DECLS

/**
 * @brief        LPF init
 * @param[out]   lpf:   The pointer of LPF structure
 * @param[in]    param: The parameter of filtering
 * @param[in]    time:  The time interval of filtering, unit /s
*/
void cc_lpf_init(cc_lpf_t *lpf,
                 float     param,
                 float     time);

/**
 * @brief        LPF calculate
 * @param[in]    lpf:   The pointer of LPF structure
 * @param[in]    input: The input
*/
void cc_lpf(cc_lpf_t *lpf,
            float     input);

/**
 * @brief        LPF clear
 * @param[in]    lpf: The pointer of LPF structure
*/
void cc_lpf_clear(cc_lpf_t *lpf);

__END_DECLS

/* Private defines -----------------------------------------------------------*/

/* __CC_LPF_H__ --------------------------------------------------------------*/
#endif /* __CC_LPF_H__ */

/************************ (C) COPYRIGHT tqfx *******************END OF FILE****/
