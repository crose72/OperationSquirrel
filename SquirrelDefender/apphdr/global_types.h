#pragma once

/********************************************************************************
 * @file    global_types.h
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Global typedefs to be shared throughout the program
 ********************************************************************************/
#ifndef GLOBAL_TYPES_H
#define GLOBAL_TYPES_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Global type declarations
 ********************************************************************************/

enum SYSTEM_STATE
{
    DEFAULT,
    INIT,
    PRE_ARM_GOOD,
    STANDBY,
    IN_FLIGHT_GOOD,
    IN_FLIGHT_ERROR
};

enum CONTROL_DIM
{
    X,
    Y,
    Z,
    N_DIMS
};

#endif // GLOBAL_TYPES_H
