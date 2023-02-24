#include "LKH.h"

/*
 * The TrimCandidateSet function takes care that each node has 
 * associated at most MaxCandidates candidate edges.                         
 */

void TrimCandidateSet(int MaxCandidates)
{
    Node *From;
    Candidate *NFrom;
    int Count, MaxDepotCandidates, MaxCand;

    MaxDepotCandidates = Dimension == DimensionSaved ?
        Salesmen : 2 * Salesmen;
    if (MaxDepotCandidates < MaxCandidates)
        MaxDepotCandidates = MaxCandidates;
    From = FirstNode;
    do {
        MaxCand = From->DepotId == 0 ? MaxCandidates : MaxDepotCandidates;
        Count = 0;
        for (NFrom = From->CandidateSet; NFrom && NFrom->To; NFrom++)
            Count++;
        if (Count > MaxCand) {
            From->CandidateSet =
                (Candidate *) realloc(From->CandidateSet,
                                      (MaxCand + 1) * sizeof(Candidate));
            From->CandidateSet[MaxCand].To = 0;
        }
    } while ((From = From->Suc) != FirstNode);
}
