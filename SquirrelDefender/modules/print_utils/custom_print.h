#pragma once

/********************************************************************************
 * @file    custom_print.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef CUSTOM_PRINT_H
#define CUSTOM_PRINT_H

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
class PrintTerm 
{
    public:
        PrintTerm(void);
        PrintTerm(const std::string& terminal);
        ~PrintTerm(void);

        void cpp_cout(const std::string& message);
        void cpp_cout_oneline(const std::string& message);
        void cpp_cerr(const std::string& message);
        void cpp_cerr_oneline(const std::string& message);

    private:
        bool use_default_term;
        std::ofstream debug_terminal;
};

class Print
{
    public:
        Print(void);
        ~Print(void);

        static int c_printf(const char *format, ...);
        static int c_fprintf(const char *format, ...);
        static void cpp_cout(const std::string& message);
        static void cpp_cout_oneline(const std::string& message); 
        static void cpp_cerr(const std::string& message);
        static void cpp_cerr_oneline(const std::string& message);

};

#endif // CUSTOM_PRINT_H
