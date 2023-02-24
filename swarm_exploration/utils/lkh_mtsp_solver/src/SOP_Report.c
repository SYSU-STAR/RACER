#include "LKH.h"

void SOP_Report(GainType Cost)
{
    printff("  Cost = " GainFormat "_" GainFormat "\n",
            CurrentPenalty, Cost);
}
