#include "LKH.h"
#include "Heap.h"

/* The MTSP_InitialTour function computes an mTSP tour using the algorithm
 * described on page 392 in
 *
 *     B. Soylu,
 *     A general variable neighborhood search heuristic for multiple
 *     traveling salesmen problem.
 *     Computers & Industrial Engineering 90, pp. 390–401, 2015.  
 */

GainType MTSP_InitialTour()
{
    Node *N, *Last, *Winner, *Route;
    GainType Cost = 0;
    int Min, d, i, Dim = DimensionSaved - Salesmen + 1;
    double EntryTime = GetTime();

    if (TraceLevel >= 1)
        printff("MTSP = ");
    for (i = 0; i < Salesmen; i++) {
        Route = i == 0 ? Depot : &NodeSet[Dim + i];
        if (Route == FirstNode)
            FirstNode = Route->Suc;
        Follow(Route, Route);
        Route->Cost = 0;
        Route->V = 0;
    }
    /* Insert the special nodes in their corresponding routes */
    for (i = 1; i <= Dim; i++) {
        N = &NodeSet[i];
        if (N->Special) {
            Route = N->Special == 1 ? Depot :
                &NodeSet[Dim + N->Special - 1];
            Follow(N, Route);
            assert(!Forbidden(N, Route));
            N->Cost = (C(N, Route) - N->Pi - Route->Pi) / Precision;
            Route->Cost = 2 * N->Cost;
            Route->V++;
            Cost += Route->Cost;
        }
    }
    /* Use a heap for sorting the unassigned nodes */
    for (i = 1; i <= Dim; i++) {
        N = &NodeSet[i];
        if (N == Depot || N->Special)
            continue;
        N->Rank = (C(N, Depot) - N->Pi - Depot->Pi) / Precision;
        HeapLazyInsert(N);
    }
    Heapify();
    while ((N = HeapDeleteMin())) {
        N->Cost = N->Rank;
        Winner = 0;
        Min = INT_MAX;
        /* Find a winner route */
        for (i = 0; i < Salesmen; i++) {
            Route = i == 0 ? Depot : &NodeSet[Dim + i];
            if (Route->V == MTSPMaxSize)
                continue;
            Last = Route->Pred;
            d = (C(Last, N) - Last->Pi - N->Pi) / Precision +
                N->Cost - Last->Cost;
            if (MTSPObjective == MINMAX || MTSPObjective == MINMAX_SIZE)
                d *= Route->Cost / (Cost + 0.5);
            else
                d += N->Cost - Last->Cost;
            if (d < Min) {
                Min = d;
                Winner = Route;
            }
        }
        assert(Winner);
        Cost -= Winner->Cost;
        Last = Winner->Pred;
        if (MTSPObjective == MINMAX || MTSPObjective == MINMAX_SIZE)
            Winner->Cost += (C(Last, N) - Last->Pi - N->Pi) / Precision +
                N->Cost - Last->Cost;
        else
            Winner->Cost += Min;
        Cost += Winner->Cost;
        Winner->V++;
        Follow(N, Last);
    }
    for (i = 0; i < Salesmen; i++) {
        Route = i == 0 ? Depot : &NodeSet[Dim + i];
        N = (i == 0 ? Depot : &NodeSet[Dim + i])->Pred;
        N->Suc = i == 1 ? Depot :
            i == 0 ? &NodeSet[Dim + Salesmen - 1] : &NodeSet[Dim + i - 1];
    }
    N = FirstNode;
    do
        N->Suc->Pred = N;
    while ((N = N->Suc) != FirstNode);

    CurrentPenalty = PLUS_INFINITY;
    CurrentPenalty = Penalty();
    if (TraceLevel >= 1) {
        if (Salesmen > 1 || ProblemType == SOP)
            printff(GainFormat "_" GainFormat, CurrentPenalty, Cost);
        else
            printff(GainFormat, Cost);
        if (Optimum != MINUS_INFINITY && Optimum != 0) {
            if (MTSPObjective == MINMAX || MTSPObjective == MINMAX_SIZE)
                printff(", Gap = %0.4f%%",
                        100.0 * (CurrentPenalty - Optimum) / Optimum);
            else
                printff(", Gap = %0.4f%%",
                        100.0 * (Cost - Optimum) / Optimum);
        }
        printff(", Time = %0.2f sec.\n", fabs(GetTime() - EntryTime));
    }
    return Cost;
}
