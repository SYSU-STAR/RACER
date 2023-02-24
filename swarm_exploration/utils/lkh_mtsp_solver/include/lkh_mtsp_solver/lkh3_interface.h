#ifndef _LKH3_INTERFACE_H
#define _LKH3_INTERFACE_H

extern "C" {
  #include "LKH.h"
  #include "Genetic.h"
  #include "BIT.h"
}

int solveMTSPWithLKH3(const char* input_file);

#endif