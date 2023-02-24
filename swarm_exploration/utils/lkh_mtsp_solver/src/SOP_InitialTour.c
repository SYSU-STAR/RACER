#include "LKH.h"
#include "Heap.h"

/* The SOP_InitialTour function generates a tour using topological
 * sorting
 */

#define InDegree V

GainType SOP_InitialTour()
{
    Node **Fringe, *First = 0, *Last, *N;
    int FringeNodes = 0, *Subset, Count, i;
    GainType Cost;
    Constraint *Con;

    assert(Asymmetric);
    N = FirstNode;
    do
        N->InDegree = 0;
    while ((N = N->Suc) != FirstNode);
    do {
        if (N->Id <= DimensionSaved)
            for (Con = N->FirstConstraint; Con; Con = Con->Next)
                Con->t2->InDegree++;
    } while ((N = N->Suc) != FirstNode);
    if (ProblemType == SOP)
        NodeSet[DimensionSaved].InDegree = DimensionSaved - 1;
    Fringe = (Node **) malloc(DimensionSaved * sizeof(Node *));
    Subset = (int *) malloc(DimensionSaved * sizeof(int));
    First = Last = &NodeSet[1];
    First->Prev = First->Next = First;
    FringeNodes = 0;
    do {
        if (N->Id <= DimensionSaved && N != First && N->InDegree == 0)
            Fringe[FringeNodes++] = N;
    } while ((N = N->Suc) != FirstNode);
    while (FringeNodes > 0) {
        Count = 0;
        for (i = 0; i < FringeNodes; i++)
            if (IsCandidate(Last, Fringe[i] + DimensionSaved))
                Subset[Count++] = i;
        i = Count > 0 ? Subset[Random() % Count] : Random() % FringeNodes;
        N = Fringe[i];
        Fringe[i] = Fringe[--FringeNodes];
        N->Prev = Last;
        N->Next = First;
        First->Prev = Last->Next = N;
        Last = N;
        for (Con = N->FirstConstraint; Con; Con = Con->Next) {
            if (--Con->t2->InDegree == 0)
                Fringe[FringeNodes++] = Con->t2;
            else if (Con->t2->InDegree < 0)
                eprintf("SOP_InitialTour: Precedence cycle detected");
        }
    }
    free(Fringe);
    free(Subset);
    if (ProblemType == SOP) {
        N = &NodeSet[DimensionSaved];
        N->Prev = Last;
        N->Next = First;
        First->Prev = Last->Next = N;
    }
    N = First;
    Follow(N, N);
    do {
        Follow(N->Next, N);
    } while ((N = N->Next) != First);
    do {
        Precede(N + DimensionSaved, N);
    } while ((N = N->Next) != First);
    for (i = 2; i <= Salesmen; i++) {
        Last = &NodeSet[Dimension - Salesmen + i];
        while (Forbidden(N, Last))
            N = N->Suc;
        Follow(Last, Last);
        Follow(Last, N);
        N = Last;
        Last = &NodeSet[Last->Id - DimensionSaved];
        Follow(Last, Last);
        Follow(Last, N);
    }
    Cost = 0;
    N = First;
    do
        Cost += C(N, N->Suc) - N->Pi - N->Suc->Pi;
    while ((N = N->Suc) != First);
    CurrentPenalty = PLUS_INFINITY;
    CurrentPenalty = Penalty ? Penalty() : 0;
    return Cost / Precision;
}
