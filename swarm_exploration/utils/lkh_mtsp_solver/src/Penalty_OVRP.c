#include "LKH.h"
#include "Segment.h"

GainType Penalty_OVRP()
{
    static Node *StartRoute = 0;
    Node *N, *NextN, *CurrentRoute;
    GainType DemandSum, DistanceSum, P = 0;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved;

    if (!StartRoute)
        StartRoute = Depot;
    if (StartRoute->Id > DimensionSaved)
        StartRoute -= DimensionSaved;
    N = StartRoute;
    do {
        CurrentRoute = N;
        DistanceSum = DemandSum = 0;
        do {
            if (N->Id <= Dim && N != Depot) {
                if ((DemandSum += N->Demand) > Capacity)
                    P += DemandSum - Capacity;
                if (DistanceSum < N->Earliest)
                    DistanceSum = N->Earliest;
                if (DistanceSum > N->Latest)
                    P += DistanceSum - N->Latest;
                if (P > CurrentPenalty ||
                    (P == CurrentPenalty && CurrentGain <= 0)) {
                    StartRoute = CurrentRoute;
                    return CurrentPenalty + (CurrentGain > 0);
                }
                DistanceSum += N->ServiceTime;
            }
            NextN = Forward ? SUCC(N) : PREDD(N);
            if (DistanceLimit != DBL_MAX)
                DistanceSum += (C(N, NextN) - N->Pi - NextN->Pi) /
                    Precision;
            N = Forward ? SUCC(NextN) : PREDD(NextN);
        } while (N->DepotId == 0);
        if (DistanceSum > DistanceLimit &&
            ((P += DistanceSum - DistanceLimit) > CurrentPenalty ||
             (P == CurrentPenalty && CurrentGain <= 0))) {
            StartRoute = CurrentRoute;
            return CurrentPenalty + (CurrentGain > 0);
        }
    } while (N != StartRoute);
    return P;
}
