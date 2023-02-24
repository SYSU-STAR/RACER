#include "LKH.h"
#include "Segment.h"

GainType Penalty_SOP()
{
    Node *First = &NodeSet[1], *N = First;
    int P, i = 1, j;
    Constraint *ConPred = 0, *ConSuc = 0;
    static Constraint *Con = 0;

    if (SUCC(First)->Id != First->Id + DimensionSaved) {
        do
            if (N->Id <= DimensionSaved)
                N->Loc = i++;
        while ((N = SUCC(N)) != First);
    } else {
        do
            if (N->Id <= DimensionSaved)
                N->Loc = i++;
        while ((N = PREDD(N)) != First);
    }
    P = DimensionSaved - NodeSet[DimensionSaved].Loc;
    if (P > CurrentPenalty || (P == CurrentPenalty && CurrentGain <= 0))
        return CurrentPenalty + (CurrentGain > 0);
    if (CurrentPenalty == 0) {
        if (Con && Con->t1->Loc > Con->t2->Loc)
            return 1;
        for (i = Swaps - 1; i >= 0; i--) {
            for (j = 1; j <= 4; j++) {
                N = j == 1 ? SwapStack[i].t1 :
                    j == 2 ? SwapStack[i].t2 :
                    j == 3 ? SwapStack[i].t3 : SwapStack[i].t4;
                if (N->Id <= DimensionSaved) {
                    for (Con = N->FirstConstraint; Con; Con = Con->Next)
                        if (Con->t1->Loc > Con->t2->Loc)
                            return 1;
                }
            }
        }
    }
    for (Con = FirstConstraint; Con; ConPred = Con, Con = ConSuc) {
        ConSuc = Con->Suc;
        if (Con->t1->Loc > Con->t2->Loc) {
            if (Con != FirstConstraint) {
                ConPred->Suc = ConSuc;
                Con->Suc = FirstConstraint;
                FirstConstraint = Con;
                Con = ConPred;
            }
            if (++P > CurrentPenalty ||
                (P == CurrentPenalty && CurrentGain <= 0))
                return CurrentPenalty + (CurrentGain > 0);
        }
    }
    return P;
}
