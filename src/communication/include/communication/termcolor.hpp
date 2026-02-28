#ifndef TERMCOLOR_HPP_
#define TERMCOLOR_HPP_

#include <iostream>
#include <sstream>

namespace termcolor {
  constexpr const char* reset   = "\033[0m";

  constexpr const char* red     = "\033[31m";
  constexpr const char* yellow  = "\033[33m";
  constexpr const char* brown   = "\033[33m";
  constexpr const char* green   = "\033[32m";
  constexpr const char* blue    = "\033[34m";
  constexpr const char* white   = "\033[37m";
  constexpr const char* orange256 = "\033[38;5;208m";
}

#endif // TERMCOLOR_HPP_
