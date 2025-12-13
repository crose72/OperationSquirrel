/********************************************************************************
 * @file    global_types.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Global types shared across the software system.
 ********************************************************************************/
#ifndef GLOBAL_TYPES_H
#define GLOBAL_TYPES_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Global type declarations
 ********************************************************************************/
enum SystemState
{
    DEFAULT,
    INIT,
    PRE_ARM_GOOD,
    ARMED,
    STANDBY,
    IN_FLIGHT_GOOD,
    IN_FLIGHT_ERROR
};

enum ControlDim
{
    X,
    Y,
    Z,
    ROLL,
    PITCH,
    YAW,
    THRUST,
    N_DIMS
};

#endif // GLOBAL_TYPES_H
