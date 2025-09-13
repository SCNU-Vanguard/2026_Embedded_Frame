/**
* @file INS.h
 * @author guatai (2508588132@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __INS_H__
#define __INS_H__

#include <stdint.h>

typedef struct 
{
    /* data */
}__attribute__((packed))INS_behaviour_t;

typedef struct
{
    /* data */
}__attribute__((packed))INS_cmd_t;

extern void INS_Data_Update(void);

#endif /* __INS_H__ */