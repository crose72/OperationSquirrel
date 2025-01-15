/********************************************************************************
 * @file    debugger.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Provide interfaces to
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "custom_print.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: PrintTerm
 * Description: Default constructor that defaults to the standard output.
 ********************************************************************************/
PrintTerm::PrintTerm() : use_default_term(true)
{
#ifdef DEBUG_BUILD
    // No need to do anything as use_default_terminal is already set to true
#endif
}

/********************************************************************************
 * Function: PrintTerm
 * Description: Class constructor.
 ********************************************************************************/
PrintTerm::PrintTerm(const std::string &terminal) : debug_terminal(terminal), use_default_term(false)
{
    if (!debug_terminal.is_open())
    {
        PrintPass::cpp_cerr("Failed to open terminal: " + terminal);
        use_default_term = true;
    }
}

/********************************************************************************
 * Function: ~PrintTerm
 * Description: Class destructor.
 ********************************************************************************/
PrintTerm::~PrintTerm(void)
{
    if (debug_terminal.is_open())
    {
        debug_terminal.close();
    }
}

/********************************************************************************
 * Function: cpp_cout
 * Description: Print statements to the additional terminal.
 ********************************************************************************/
void PrintTerm::cpp_cout(const std::string &message)
{
    if (debug_terminal.is_open())
    {
        debug_terminal << message << std::endl;
    }

    if (use_default_term)
    {
        PrintPass::cpp_cout(message);
    }
}

/********************************************************************************
 * Function: cpp_cout_oneline
 * Description: Print statements to the additional terminal with no newline char.
 ********************************************************************************/
void PrintTerm::cpp_cout_oneline(const std::string &message)
{
    if (debug_terminal.is_open())
    {
        debug_terminal << message;
    }

    if (use_default_term)
    {
        PrintPass::cpp_cout_oneline(message);
    }
}

/********************************************************************************
 * Function: cpp_cerr
 * Description: Print statements to the additional terminal.
 ********************************************************************************/
void PrintTerm::cpp_cerr(const std::string &message)
{
    if (debug_terminal.is_open())
    {
        debug_terminal << message << std::endl;
    }

    if (use_default_term)
    {
        PrintPass::cpp_cerr(message);
    }
}

/********************************************************************************
 * Function: cpp_cerr_oneline
 * Description: Print statements to the additional terminal with no newline char.
 ********************************************************************************/
void PrintTerm::cpp_cerr_oneline(const std::string &message)
{
    if (debug_terminal.is_open())
    {
        debug_terminal << message;
    }

    if (use_default_term)
    {
        PrintPass::cpp_cerr_oneline(message);
    }
}

/********************************************************************************
 * Function: PrintPass
 * Description: Class constructor.
 ********************************************************************************/
PrintPass::PrintPass(void) {}

/********************************************************************************
 * Function: ~PrintPass
 * Description: Class destructor.
 ********************************************************************************/
PrintPass::~PrintPass(void) {}

/********************************************************************************
 * Function: c_printf
 * Description: Use c printf to print formatted strings to the default terminal.
 ********************************************************************************/
int PrintPass::c_printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = vprintf(format, args);
    va_end(args);
    return result;
}

/********************************************************************************
 * Function: c_fprintf
 * Description: Use c fprintf to print formatted error strings to the default
 *              terminal.
 ********************************************************************************/
int PrintPass::c_fprintf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = vfprintf(stderr, format, args);
    va_end(args);
    return result;
}

/********************************************************************************
 * Function: cpp_cout
 * Description: Use c++ function cout to print to terminal.
 ********************************************************************************/
void PrintPass::cpp_cout(const std::string &message)
{
    std::cout << message << std::endl;
}

/********************************************************************************
 * Function: cpp_cout_oneline
 * Description: Use c++ function cout to print to terminal with no newline char.
 ********************************************************************************/
void PrintPass::cpp_cout_oneline(const std::string &message)
{
    std::cout << message;
}

/********************************************************************************
 * Function: cpp_cerr
 * Description: Use c++ function cerr to print errors to terminal.
 ********************************************************************************/
void PrintPass::cpp_cerr(const std::string &message)
{
    std::cerr << message << std::endl;
}

/********************************************************************************
 * Function: cpp_cerr
 * Description: Use c++ function cerr to print errors to terminal with no
 *              newline char.
 ********************************************************************************/
void PrintPass::cpp_cerr_oneline(const std::string &message)
{
    std::cerr << message;
}
