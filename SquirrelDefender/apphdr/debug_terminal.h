#pragma once

/********************************************************************************
 * @file    debug_terminal.h
 * @author  Cameron Rose
 * @date    12/27/2023
 ********************************************************************************/
#ifndef DEBUG_TERMINAL_H
#define DEBUG_TERMINAL_H

/************************************
 * Includes
 ************************************/
#include <iostream>
#include <fstream>
#include <string>

/************************************
 * Imported objects
 ************************************/

/************************************
 * Exported objects
 ************************************/

/************************************
 * Function prototypes
 ************************************/
class Debugger 
{
    public:
        Debugger(const std::string& terminal);
        ~Debugger(void);

        void Print(const std::string& message);

    private:
        std::ofstream debug_terminal;
};

#endif // DEBUG_TERMINAL_H