#include "LKH.h"
#include "Segment.h"

GainType TSPTW_MakespanCost()
{
    Node *N = Depot, *NextN;
    GainType Sum = 0;
    int Forward = SUCC(N)->Id != N->Id + DimensionSaved;

    if (ProblemType != TSPTW)
        return 0;
    do {
        if (N->Id <= DimensionSaved)
            if (Sum < N->Earliest)
                Sum = N->Earliest;
        NextN = Forward ? SUCC(N) : PREDD(N);
        Sum += (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
    } while ((N = NextN) != Depot);
    return Sum;
}
