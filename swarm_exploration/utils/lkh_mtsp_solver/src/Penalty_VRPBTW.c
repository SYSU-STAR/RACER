#include "LKH.h"
#include "Segment.h"

GainType Penalty_VRPBTW()
{
    static Node *StartRoute = 0;
    Node *N, *NextN, *CurrentRoute;
    GainType CostSum, DemandSum[2], P = 0;
    int Linehauls, Backhauls;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved;

    if (!StartRoute)
        StartRoute = Depot;
    if (StartRoute->Id > DimensionSaved)
        StartRoute -= DimensionSaved;
    N = StartRoute;
    do {
        CurrentRoute = N;
        DemandSum[0] = DemandSum[1] = 0;
        Linehauls = Backhauls = 0;
        CostSum = 0;
        do {
            if (N->Id <= Dim && N != Depot) {
                if (CostSum < N->Earliest)
                    CostSum = N->Earliest;
                if (CostSum > N->Latest)
                    P += CostSum - N->Latest;
                if (N->Backhaul)
                    Backhauls++;
                else {
                    Linehauls++;
                    if (Backhauls > 0)
                        P += 1000000;
                }
                if ((DemandSum[N->Backhaul] += N->Demand) > Capacity)
                    P += DemandSum[N->Backhaul] - Capacity;
                if (P > CurrentPenalty ||
                    (P == CurrentPenalty && CurrentGain <= 0)) {
                    StartRoute = CurrentRoute;
                    return CurrentPenalty + (CurrentGain > 0);
                }
                CostSum += ServiceTime;
            }
            NextN = Forward ? SUCC(N) : PREDD(N);
            CostSum += (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
            CostSum = (CostSum + 50) / 100 * 100;
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
