#include "LKH.h"
#include "Heap.h"

static void Dijkstra(Node * Source);

void STTSP2TSP()
{
    int NewDimension = 0, i, j, k;
    int **Matrix;
    Node *N1 = FirstNode, *N2;

    i = 0;
    do {
        if (N1->Required) {
            NewDimension++;
            N1->Serial = i++;
        }
    } while ((N1 = N1->Suc) != FirstNode);
    Matrix = (int **) malloc(NewDimension * sizeof(int **));
    for (i = 0; i < NewDimension; i++)
        Matrix[i] = (int *) malloc(NewDimension * sizeof(int));
    do {
        if (N1->Required) {
            Dijkstra(N1);
            N1->PathLength = (int *) calloc(NewDimension + 1, sizeof(int));
            N1->Path = (int **) calloc(NewDimension + 1, sizeof(int *));
            i = N1->Serial;
            N2 = FirstNode;
            do {
                if (N2 != N1 && N2->Required) {
                    j = N2->Serial;
                    Matrix[i][j] = N2->Cost / Precision;
                    Node *N = N2;
                    while ((N = N->Dad) != N1)
                        N1->PathLength[j + 1]++;
                    if (N1->PathLength[j + 1] > 0) {
                        N1->Path[j + 1] = 
                           (int *) malloc(N1->PathLength[j + 1] * sizeof(int));
                        k = N1->PathLength[j + 1];
                        N = N2;
                        while ((N = N->Dad) != N1)
                            N1->Path[j + 1][--k] = N->OriginalId;
                    }
                }
            } while ((N2 = N2->Suc) != FirstNode);
        }
    } while ((N1 = N1->Suc) != FirstNode);
    j = 0;
    for (i = 1; i <= Dimension; i++) {
        N1 = &NodeSet[i];
        if (N1->Required) {
            N1->Id = N1->Serial + 1;
            N1->C = Matrix[N1->Serial] - 1;
            N1->CandidateSet = 0;
            NodeSet[++j] = *N1;
        }
    }
    for (i = 1; i <= NewDimension; i++, N1 = N2) {
        N2 = &NodeSet[i];
        if (i == 1)
            FirstNode = N2;
        else
            Link(N1, N2);
    }
    Link(N1, FirstNode);
    Dimension = DimensionSaved = NewDimension;
    WeightType = EXPLICIT;
    Distance = Distance_EXPLICIT;
}

static void Dijkstra(Node * Source)
{
    Node *Blue;          
    Node *N;
    Candidate *NBlue;
    int d;

    Blue = N = Source;
    Blue->Dad = 0;
    Blue->Loc = 0;
    Blue->Cost = 0;
    HeapClear();
    while ((N = N->Suc) != Source) {
        N->Dad = Blue;
        N->Cost = N->Rank = INT_MAX / 2 / Precision;
        HeapLazyInsert(N);
    }
    for (NBlue = Blue->CandidateSet; (N = NBlue->To); NBlue++) {
        N->Dad = Blue;
        N->Cost = N->Rank = NBlue->Cost;
        HeapSiftUp(N);
    }
    while ((Blue = HeapDeleteMin())) {
        for (NBlue = Blue->CandidateSet; (N = NBlue->To); NBlue++) {
            if (N->Loc && (d = Blue->Cost + NBlue->Cost) < N->Cost) {
                N->Dad = Blue;
                N->Cost = N->Rank = d;
                HeapSiftUp(N);
            }
        }
    }
}
