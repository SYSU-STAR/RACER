#include "LKH.h"
#include "Segment.h"

GainType Penalty_ACVRP()
{
    static Node *StartRoute = 0;
    Node *N, *CurrentRoute;
    GainType DemandSum, P = 0;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved;

    if (!StartRoute)
        StartRoute = Depot;
    if (StartRoute->Id > DimensionSaved)
        StartRoute -= DimensionSaved;
    N = StartRoute;
    do {
        CurrentRoute = N;
        DemandSum = 0;
        do {
            if (N->Id <= Dim && N != Depot) {
                if ((DemandSum += N->Demand) > Capacity)
                    P += DemandSum - Capacity;
                if (P > CurrentPenalty ||
                    (P == CurrentPenalty && CurrentGain <= 0)) {
                    StartRoute = CurrentRoute;
                    return CurrentPenalty + (CurrentGain > 0);
                }
            }
            N = Forward ? SUCC(N) : PREDD(N);
        } while (N->DepotId == 0);
    } while (N != StartRoute);
    return P;
}
