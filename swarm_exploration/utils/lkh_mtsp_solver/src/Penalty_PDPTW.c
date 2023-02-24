#include "LKH.h"
#include "Segment.h"

GainType Penalty_PDPTW()
{
    static Node *StartRoute = 0;
    Node *N, *NextN, *CurrentRoute, *M, *NextM;
    GainType CostSum, DemandSum, P = 0;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved;
    int Pickups, Deliveries;

    if (!StartRoute)
        StartRoute = Depot;
    if (StartRoute->Id > DimensionSaved)
        StartRoute -= DimensionSaved;
    N = StartRoute;
    do {
        CurrentRoute = N;
        Serial++;
        CostSum = DemandSum = 0;
        Pickups = Deliveries = 0;
        do {
            if (N->Id <= Dim && N != Depot) {
                DemandSum += N->Demand;
                if (DemandSum > Capacity)
                    P += DemandSum - Capacity;
                if (CostSum < N->Earliest)
                    CostSum = N->Earliest;
                if (CostSum > N->Latest)
                    P += CostSum - N->Latest;
                if (P > CurrentPenalty) {
                    StartRoute = CurrentRoute;
                    return P;
                }
                if (N->Pickup) {
                    if (++Deliveries > Pickups ||
                        NodeSet[N->Pickup].Serial != Serial)
                        P += 1;
                } else if (N->Delivery) {
                    N->Serial = Serial;
                    if (++Pickups == 1) {
                        M = N;
                        while (M->DepotId == 0) {
                            if (M->Pickup)
                                M->Serial = Serial;
                            NextM = Forward ? SUCC(M) : PREDD(M);
                            M = Forward ? SUCC(NextM) : PREDD(NextM);
                        }
                    }
                }
                if (P > CurrentPenalty) {
                    StartRoute = CurrentRoute;
                    return P;
                }
                CostSum += N->ServiceTime;
            }
            NextN = Forward ? SUCC(N) : PREDD(N);
            CostSum += (C(N, NextN) - N->Pi - NextN->Pi) / Precision;
            N = Forward ? SUCC(NextN) : PREDD(NextN);
        } while (N->DepotId == 0);
        if (CostSum > Depot->Latest) {
            P += CostSum - Depot->Latest;
            if (P > CurrentPenalty) {
                StartRoute = CurrentRoute;
                return P;
            }
        }
    } while (N != StartRoute);
    return P;
}
