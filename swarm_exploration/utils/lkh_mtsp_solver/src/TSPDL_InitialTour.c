#include "LKH.h"

/* The TSPL_InitialTour function generates a tour using Proposition 1.1 in
 * the paper
 *
 *       Rakke J. G., Christiansen M., Fagerholt, K., Laporte, G.:
 *       The Traveling SalesmanProblem with Draft Limits.
 *       Comput. Oper. Res., 39:2161-2167 (2012)
 */

static int compare(const void *Na, const void *Nb);

GainType TSPDL_InitialTour()
{
    Node **T, *N, *Last, *Temp;
    int i, j;
    GainType Cost = 0;
    double EntryTime = GetTime();

    if (TraceLevel >= 1)
        printff("TSPDL = ");
    assert(Asymmetric);
    assert(Salesmen == 1);
    T = (Node **) malloc(DimensionSaved * sizeof(Node *));
    for (i = 0; i < DimensionSaved; i++)
        T[i] = &NodeSet[i + 1];
    for (i = 1; i < DimensionSaved; i++) {
        j = rand() % (i + 1);
        Temp = T[i];
        T[i] = T[j];
        T[j] = Temp;
    }
    qsort(T, DimensionSaved, sizeof(Node *), compare);
    for (i = 0; i < DimensionSaved; i++) {
        if (T[i] == Depot) {
            Temp = T[0];
            T[0] = Depot;
            T[i] = Temp;
            break;
        }
    }
    Last = FirstNode = T[0];
    Follow(Last, Last);
    for (i = 1; i < DimensionSaved; Last = T[i], i++)
        Follow(T[i], Last);
    free(T);
    for (i = 1; i <= DimensionSaved; i++) {
        N = &NodeSet[i];
        Precede(N + DimensionSaved, N);
    }
    Cost = 0;
    N = Last;
    do
        Cost += C(N, N->Suc) - N->Pi - N->Suc->Pi;
    while ((N = N->Suc) != Last);
    Cost /= Precision;
    CurrentPenalty = PLUS_INFINITY;
    CurrentPenalty = Penalty ? Penalty() : 0;
    if (TraceLevel >= 1) {
        printff(GainFormat "_" GainFormat, CurrentPenalty, Cost);
        if (Optimum != MINUS_INFINITY && Optimum != 0)
            printff(", Gap = %0.2f%%", 100.0 * (Cost - Optimum) / Optimum);
        printff(", Time = %0.2f sec.\n", fabs(GetTime() - EntryTime));
    }
    return Cost;
}

static int compare(const void *Na, const void *Nb)
{
    return (*(Node **) Nb)->DraftLimit - (*(Node **) Na)->DraftLimit;
}
