#include "LKH.h"
#include "Segment.h"

GainType Penalty_M_PDTSP()
{
    Node *N;
    GainType P = 0;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved;
    int Load = 0, MaxLoad = INT_MIN, MinLoad = INT_MAX, k;
    static int *ComLoad = 0, *Min = 0;

    if (!ComLoad) {
        ComLoad = (int *) malloc(DemandDimension * sizeof(int));
        Min = (int *) malloc(DemandDimension * sizeof(int));
    }
    for (k = 0; k < DemandDimension; k++) {
        ComLoad[k] = 0;
        Min[k] = INT_MAX;
    }
    N = Depot;
    do {
        if (N->Id <= Dim) {
            Load += N->Demand;
            MinLoad = 0;
            for (k = 0; k < DemandDimension; k++) {
                ComLoad[k] += N->M_Demand[k];
                Load += N->M_Demand[k];
                if (ComLoad[k] < Min[k])
                    Min[k] = ComLoad[k];
                MinLoad += Min[k];
                if (ProblemType == M1_PDTSP && ComLoad[k] < 0)
                    P -= ComLoad[k];
            }
            if (Load > MaxLoad)
                MaxLoad = Load;
            if (MaxLoad - MinLoad > Capacity)
                P += MaxLoad - MinLoad - Capacity;
            if (P > CurrentPenalty)
                return CurrentPenalty + 1;
        }
        N = Forward ? SUCC(N) : PREDD(N);
    } while (N != Depot);
    return P;
}
