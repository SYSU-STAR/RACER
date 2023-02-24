#include "LKH.h"

/* The CVRP_InitialTour function computes an initial tour using the 
 * Clarke and Wright savings algorithm.
 */

#define ITERATIONS 10

#define Degree V
#define Size LastV
#define Load Loc

static Node *Find(Node * v);
static void Union(Node * x, Node * y);
static void MakeSets();
static void Distribute(int Constrained, double R);
static int compareValue(const void *s1, const void *s2);
static void CreateS();

typedef struct Saving {
    int Value, i, j;
} Saving;

static Saving *S;
static int Sets, SSize;

GainType CVRP_InitialTour()
{
    Node *N, *Last, *Next, *Tour;
    int s, Dim = Dimension - Salesmen + 1, it;
    GainType Cost, BestCost = PLUS_INFINITY, BestPenalty = PLUS_INFINITY;
    double EntryTime = GetTime();

    if (TraceLevel >= 1)
        printff("CVRP = ");
    assert(!Asymmetric);
    if (!S)
        CreateS();
    for (it = 0; it < ITERATIONS; it++) {
        for (s = 0; s < Salesmen; s++) {
            Tour = s == 0 ? Depot : &NodeSet[Dim + s];
            if (Tour == FirstNode)
                FirstNode = Tour->Suc;
            Follow(Tour, Depot);
            Tour->Dad = Tour;
            Tour->Prev = Tour->Next = 0;
            Tour->Degree = 0;
            Tour->Size = 1;
        }
        MakeSets();
        if (it > 0)
            Distribute(1, 0.01);
        if (Sets > Salesmen)
            Distribute(1, 0);
        if (Sets > Salesmen) {
            if (BestPenalty == 0)
                continue;
            Distribute(0, 0);
        }
        Sets += Salesmen;
        N = FirstNode;
        do {
            if (N->Degree <= 1 && (s = N->Special) > 0) {
                Tour = s == 1 ? Depot : &NodeSet[Dim + s - 1];
                Union(N, Tour);
                s = s == Salesmen ? 1 : s + 1;
                if (N->Degree <= 1) {
                    Tour = s == 1 ? Depot : &NodeSet[Dim + s - 1];
                    Union(N, Tour);
                }
            }
        } while ((N = N->Suc) != FirstNode);
        while (Sets > 0) {
            if (N->Degree <= 1) {
                for (s = 1; s <= Salesmen; s++) {
                    Tour = s == 1 ? Depot : &NodeSet[Dim + s - 1];
                    if (Tour->Degree <= 1 &&
                        (Sets == 1 || Find(N) != Find(Tour))) {
                        Union(N, Tour);
                        if (N->Degree <= 1)
                            N = N->Pred;
                        break;
                    }
                }
            }
            N = N->Suc;
        }
        Last = N = FirstNode = Depot;
        do {
            Next = N->Next != Last ? N->Next : N->Prev;
            Follow(N, Last);
            Last = N;
        } while ((N = Next) != FirstNode);
        N = FirstNode = Depot;
        Cost = 0;
        do
            Cost += C(N, N->Suc) - N->Pi - N->Suc->Pi;
        while ((N = N->Suc) != FirstNode);
        Cost /= Precision;
        Cost += ServiceTime * (Dim - 1);
        CurrentPenalty = PLUS_INFINITY;
        CurrentPenalty = Penalty();
        if (CurrentPenalty < BestPenalty ||
            (CurrentPenalty == BestPenalty && Cost < BestCost)) {
            N = FirstNode;
            while ((N = N->OldSuc = N->Suc) != FirstNode);
            BestCost = Cost;
            BestPenalty = CurrentPenalty;
        }
    }
    N = FirstNode;
    do
        (N->Suc = N->OldSuc)->Pred = N;
    while ((N = N->Suc) != FirstNode);
    Cost = BestCost;
    CurrentPenalty = BestPenalty;
    if (TraceLevel >= 1) {
        if (Salesmen > 1 || ProblemType == SOP)
            printff(GainFormat "_" GainFormat, CurrentPenalty, Cost);
        else
            printff(GainFormat, Cost);
        if (Optimum != MINUS_INFINITY && Optimum != 0)
            printff(", Gap = %0.2f%%", 100.0 * (Cost - Optimum) / Optimum);
        printff(", Time = %0.2f sec.\n", fabs(GetTime() - EntryTime));
    }
    if (Run == Runs) {
        free(S);
        S = 0;
    }
    return Cost;
}

void CreateS()
{
    int Dim = Dimension - Salesmen + 1, i, j;
    Node *Ni, *Nj;
    SSize = 0;
    S = (Saving *) malloc((Dim - 2) * (Dim - 1) / 2 * sizeof(Saving));
    /* Compute savings */
    for (i = 1; i < Dim; i++) {
        Ni = &NodeSet[i];
        if (Ni == Depot)
            continue;
        for (j = i + 1; j <= Dim; j++) {
            Nj = &NodeSet[j];
            if (Nj == Depot || Forbidden(Ni, Nj))
                continue;
            S[SSize].Value = FixedOrCommon(Ni, Nj) ? INT_MAX :
                OldDistance(Ni, Depot) +
                OldDistance(Depot, Nj) - OldDistance(Ni, Nj);
            S[SSize].i = i;
            S[SSize].j = j;
            SSize++;
        }
    }
    /* Rank the savings in descending order */
    qsort(S, SSize, sizeof(Saving), compareValue);
}

static Node *Find(Node * v)
{
    if (v != v->Dad)
        v->Dad = Find(v->Dad);
    return v->Dad;
}

static void Union(Node * x, Node * y)
{
    Node *u = Find(x), *v = Find(y);
    if (u->Size < v->Size) {
        u->Dad = v;
        v->Size += u->Size;
        v->Load += u->Load;
        v->Cost += u->Cost + OldDistance(x, y) -
            OldDistance(x, Depot) - OldDistance(y, Depot);
    } else {
        v->Dad = u;
        u->Size += v->Size;
        u->Load += v->Load;
        u->Cost += v->Cost + OldDistance(x, y) -
            OldDistance(x, Depot) - OldDistance(y, Depot);
    }
    if (x->Degree++ == 0)
        x->Prev = y;
    else
        x->Next = y;
    if (y->Degree++ == 0)
        y->Prev = x;
    else
        y->Next = x;
    Sets--;
}

static void MakeSets()
{
    Node *N = FirstNode;
    Sets = 0;
    do {
        N->Dad = N;
        N->Prev = N->Next = 0;
        N->Degree = 0;
        N->Size = 1;
        N->Load = N->Demand;
        N->Cost = 2 * OldDistance(N, Depot);
        Sets++;
    } while ((N = N->Suc) != FirstNode);
}

static void Distribute(int Constrained, double R)
{
    Node *Ni, *Nj, *u, *v;
    int i;

    for (i = 0; i < SSize && Sets > Salesmen; i++) {
        if (R > 0 && Random() % 1000 <= 1000 * R)
            continue;
        Ni = &NodeSet[S[i].i];
        Nj = &NodeSet[S[i].j];
        if (Ni->Degree < 2 && Nj->Degree < 2) {
            u = Find(Ni);
            v = Find(Nj);
            if (u == v)
                continue;
            if (!Constrained ||
                (u->Load + v->Load <= Capacity &&
                 (DistanceLimit == DBL_MAX ||
                  u->Cost + v->Cost + OldDistance(Ni, Nj) -
                  OldDistance(Ni, Depot) - OldDistance(Nj, Depot) +
                  (u->Size + v->Size) * ServiceTime <= DistanceLimit)))
                Union(Ni, Nj);
        }
    }
}

static int compareValue(const void *s1, const void *s2)
{
    int v1 = ((Saving *) s1)->Value;
    int v2 = ((Saving *) s2)->Value;
    return v1 > v2 ? -1 : v1 == v2 ? 0 : 1;
}
