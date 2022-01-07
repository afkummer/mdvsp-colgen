#pragma once

#include <unistd.h>
#include <fstream>

inline long getMemoryUsageKb() {
   std::ifstream fid("/proc/self/statm");
   long value;
   fid >> value >> value;
   return (value * getpagesize())/1024;
}