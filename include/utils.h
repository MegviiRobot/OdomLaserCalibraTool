#ifndef CALIB_UTILS_H
#define CALIB_UTILS_H

#include <iostream>

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string YELLOW = "3m";
const std::string BLUE = "4m";
const std::string WHITE = "7m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

/*!
 * \brief colouredString. Print coloured string
 * in terminal.
 * \param str. Input string.
 * \param colour. Colour option: BLACK, RED,
 * GREEN, YELLOW, BLUE, WHITE.
 * \param option. Char type option: BOLD, REGULAR,
 * UNDERLINE.
 * \return
 */
std::string colouredString(std::string str, std::string colour, std::string option);

#endif
