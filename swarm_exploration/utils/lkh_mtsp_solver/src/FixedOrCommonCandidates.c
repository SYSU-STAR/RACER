#include "LKH.h"

/* 
 * The FixedOrCommonCandidates function returns the number of fixed or
 * common candidate edges emanating from a given node, N.
 */

int FixedOrCommonCandidates(Node * N)
{
    int Count = 0;

    Count = N->FixedTo2 ? 2 : N->FixedTo1 ? 1 : 0;
    if (MergeTourFiles >= 2) {
        if (!Fixed(N, N->MergeSuc[0]) &&
            N->Subproblem == N->MergeSuc[0]->Subproblem &&
            IsCommonEdge(N, N->MergeSuc[0]))
            Count++;
        if (!Fixed(N->MergePred, N) &&
            N->Subproblem == N->MergePred->Subproblem &&
            IsCommonEdge(N->MergePred, N))
            Count++;
    }
    if (Count > 2)
        eprintf("Node %d has more than two required candidate edges",
                N->Id);
    return Count;
}
