#pragma once

/********************************************************************************
 * @file    debugger.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef DEBUGGER_H
#define DEBUGGER_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class DebugTerm 
{
    public:
        DebugTerm(void);
        DebugTerm(const std::string& terminal);
        ~DebugTerm(void);

        void cpp_cout(const std::string& message);
        void cpp_cout_oneline(const std::string& message);
        void cpp_cerr(const std::string& message);
        void cpp_cerr_oneline(const std::string& message);

    private:
        bool use_default_term;
        std::ofstream debug_terminal;
};

class PrintPass
{
    public:
        PrintPass(void);
        ~PrintPass(void);

        static int c_printf(const char *format, ...);
        static int c_fprintf(const char *format, ...);
        static void cpp_cout(const std::string& message);
        static void cpp_cout_oneline(const std::string& message); 
        static void cpp_cerr(const std::string& message);
        static void cpp_cerr_oneline(const std::string& message);

};

#endif // DEBUGGER_H