/********************************************************************************
 * @file    path_planner
 * @author  Cameron Rose
 * @date    3/12/2025
 ********************************************************************************/
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ocp_nlp_interface.h"
#include "external_function_interface.h"
#include <ocp_nlp/ocp_nlp_common.h>
#include <iostream>
#include <vector>
#include <cmath>

//#include <ocp_nlp/ocp_nlp_common.h>
//#include <ocp_nlp_interface.h>
//#include <ocp_nlp/ocp_nlp_sqp.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
class Planner
{
public:
    Planner();
    ~Planner();

    static bool init(void);
    static void loop(void);
    static void shutdown(void);

private:
};

#endif // PATH_PLANNER_H
