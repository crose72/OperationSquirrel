/********************************************************************************
 * @file    vehicle_controller.h
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   High-level interface for sending control commands to the vehicle.
 *          Performs takeoff, landing, motion control, and mode transitions.
 ********************************************************************************/
#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

/********************************************************************************
 * Includes
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Defi./s    nitions
 ********************************************************************************/
class VehicleController
{
    VehicleController();
    ~VehicleController();

public:
    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // VEHICLE_CONTROLLER_H
