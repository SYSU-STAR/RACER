#include "LKH.h"
#include "Segment.h"

GainType Penalty_RCTVRP()
{
    static Node *StartRoute = 0;
    Node *N, *NextN, *CurrentRoute;
    GainType RiskSum, CostSum, DemandSum, P = 0;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved, d;

    if (!StartRoute)
        StartRoute = Depot;
    if (StartRoute->Id > DimensionSaved)
        StartRoute -= DimensionSaved;
    N = StartRoute;
    do {
        CurrentRoute = N;
        RiskSum = CostSum = DemandSum = 0;
        do {
            if (N->Id <= Dim && N != Depot) {
                if (CostSum < N->Earliest)
                    CostSum = N->Earliest;
                if (CostSum > N->Latest &&
                    ((P += CostSum - N->Latest) > CurrentPenalty ||
                     (P == CurrentPenalty && CurrentGain <= 0))) {
                    StartRoute = CurrentRoute;
                    return CurrentPenalty + (CurrentGain > 0);
                }
                DemandSum += N->Demand;
                CostSum += N->ServiceTime;
            }
            NextN = Forward ? SUCC(N) : PREDD(N);
            d = (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
            RiskSum += DemandSum * d;
            if (RiskSum > RiskThreshold &&
                ((P += RiskSum - RiskThreshold) > CurrentPenalty ||
                 (P == CurrentPenalty && CurrentGain <= 0))) {
                StartRoute = CurrentRoute;
                return CurrentPenalty + (CurrentGain > 0);
            }
            CostSum += d;
            if (CostSum > DistanceLimit &&
                ((P += CostSum - DistanceLimit) > CurrentPenalty ||
                 (P == CurrentPenalty && CurrentGain <= 0))) {
                StartRoute = CurrentRoute;
                return CurrentPenalty + (CurrentGain > 0);
            }
            N = Forward ? SUCC(NextN) : PREDD(NextN);
        } while (N->DepotId == 0);
        if (CostSum > Depot->Latest &&
            ((P += CostSum - Depot->Latest) > CurrentPenalty ||
             (P == CurrentPenalty && CurrentGain <= 0))) {
            StartRoute = CurrentRoute;
            return CurrentPenalty + (CurrentGain > 0);
        }
    } while (N != StartRoute);
    return P;
}
