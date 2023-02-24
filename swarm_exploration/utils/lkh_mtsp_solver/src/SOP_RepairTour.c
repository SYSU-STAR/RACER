#include "LKH.h"
#include "Heap.h"

#define InDegree V

GainType SOP_RepairTour()
{
    Node **Fringe, *First = 0, *Last, *N;
    int FringeNodes = 0, Forward, Min, i, j;
    GainType Cost;
    Constraint *Con;

    First = Last = &NodeSet[1];
    N = First;
    do
        N->InDegree = 0;
    while ((N = N->Suc) != First);
    NodeSet[DimensionSaved].InDegree = DimensionSaved - 1;
    Forward = First->Suc->Id != 1 + DimensionSaved;
    i = 0;
    do {
        if (N->Id <= DimensionSaved) {
            for (Con = N->FirstConstraint; Con; Con = Con->Next)
                Con->t2->InDegree++;
            N->Rank = Forward ? ++i : --i;
        }
    } while ((N = N->Suc) != First);
    Fringe = (Node **) malloc(DimensionSaved * sizeof(Node *));
    First->Prev = First->Next = First;
    FringeNodes = 0;
    do {
        if (N->Id <= DimensionSaved && N != First && N->InDegree == 0)
            Fringe[FringeNodes++] = N;
    } while ((N = N->Suc) != First);
    while (FringeNodes > 0) {
        Min = INT_MAX;
        for (j = FringeNodes - 1; j >= 0; j--) {
            N = Fringe[j];
            if (N == (Forward ? Last->Suc : Last->Pred) - DimensionSaved) {
                i = j;
                break;
            }
            if (N->Rank < Min) {
                Min = N->Rank;
                i = j;
            }
        }
        N = Fringe[i];
        Fringe[i] = Fringe[--FringeNodes];
        N->Prev = Last;
        N->Next = First;
        First->Prev = Last->Next = N;
        Last = N;
        for (Con = N->FirstConstraint; Con; Con = Con->Next)
            if (--Con->t2->InDegree == 0)
                Fringe[FringeNodes++] = Con->t2;
            else if (Con->t2->InDegree < 0)
                eprintf("SOP_RepairTour: Precedence cycle detected");
    }
    free(Fringe);
    N = &NodeSet[DimensionSaved];
    N->Prev = Last;
    N->Next = First;
    First->Prev = Last->Next = N;
    N = First;
    Follow(N, N);
    do {
        Follow(N->Next, N);
    } while ((N = N->Next) != First);
    do {
        Precede(&NodeSet[N->Id + DimensionSaved], N);
    } while ((N = N->Next) != First);
    Cost = 0;
    do
        Cost += C(N, N->Suc) - N->Pi - N->Suc->Pi;
    while ((N = N->Suc) != First);
    CurrentPenalty = 0;
    return Cost / Precision;
}
