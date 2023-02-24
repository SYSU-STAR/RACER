#include "LKH.h"
#include "Segment.h"

GainType Penalty_TSPTW()
{
    Node *N = Depot, *NextN;
    GainType Sum = 0, P = 0;
    int Forward = SUCC(N)->Id != N->Id + DimensionSaved;

    do {
        if (N->Id <= DimensionSaved) {
            if (Sum < N->Earliest)
                Sum = N->Earliest;
            else if (Sum > N->Latest &&
                     (P += Sum - N->Latest) > CurrentPenalty)
                return CurrentPenalty + 1;
            NextN = Forward ? SUCC(N) : PREDD(N);
        }
        NextN = Forward ? SUCC(N) : PREDD(N);
        Sum += (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
    } while ((N = NextN) != Depot);
    if (Sum > Depot->Latest &&
        ((P += Sum - Depot->Latest) > CurrentPenalty ||
         (P == CurrentPenalty && CurrentGain <= 0)))
        return CurrentPenalty + (CurrentGain > 0);
    return P;
}
