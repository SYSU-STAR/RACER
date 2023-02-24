#include "LKH.h"

#undef max
static int max(int a, int b)
{
    return a > b ? a : b;
}

void PDPTW_Reduce()
{
    const int M = INT_MAX / 2 / Precision;
    int i, j;
    Node *N, *V;

    if (Salesmen > 1)
        return;
    for (i = 1; i <= Dim; i++) {
        N = &NodeSet[i];
        for (j = 1; j <= Dim; j++) {
            if (j != i &&
                max(N->Earliest, N == Depot ? 0 : Depot->C[i]) +
                N->ServiceTime + N->C[j] > NodeSet[j].Latest)
                N->C[j] = M;
        }
        if (N->Delivery) {
            for (j = 1; j < i; j++) {
                V = &NodeSet[j];
                if (V->Delivery && N->Demand + V->Demand > Capacity)
                    N->C[j] = V->C[i] = M;
            }
        } else if (N->Pickup) {
            for (j = 1; j < i; j++) {
                V = &NodeSet[j];
                if (V->Pickup && -(N->Demand + V->Demand) > Capacity)
                    N->C[j] = V->C[i] = M;
            }
        }
        if ((j = N->Pickup))
            NodeSet[i].C[j] = Depot->C[i] = M;
        else if ((j = N->Delivery))
            NodeSet[j].C[i] = N->C[Depot->Id] = M;
    }
}
