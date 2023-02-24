#include "LKH.h"
#include "Segment.h"

GainType Penalty_VRPB()
{
    static Node *StartRoute = 0;
    Node *N, *NextN, *CurrentRoute;
    GainType DemandSum[2], P = 0;
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
        NextN = Forward ? SUCC(N) : PREDD(N);
        do {
            if (N->Id <= Dim && N != Depot) {
                if (N->Backhaul) {
                    Backhauls++;
                    if (Linehauls == 0)
                        P += 1000000;
                } else {
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
            }
            N = Forward ? SUCC(N) : PREDD(N);
        } while (N->DepotId == 0);
    } while (N != StartRoute);
    return P;
}

