#include "Segment.h"
#include "LKH.h"
#include "Sequence.h"
#include "BIT.h"

/*
 * The BestSepcialOptMove function makes sequential as well as non-sequential
 * edge exchanges. If possible, it makes a sequential 2-, 3- or 5-opt move,
 * or a non-sequential 4- or 6-pt move, that improves the tour.
 * Otherwise, it finds the best feasible 5-opt move.
 *
 * If K < 5, then only sequential 2- or 3-opt moves, and non-sequential
 * 4-opt moves are tried.
 * 
 * The function is called from the LinKernighan function.
 */

static void SelectForward(Node ** a, Node ** b, Node * from, Node * to);
static void SelectBackward(Node ** a, Node ** b, Node * from, Node * to);

Node *BestSpecialOptMove(Node * t1, Node * t2, GainType * G0,
                         GainType * Gain)
{
    Node *t3, *t4, *t5 = 0, *t6 = 0, *t7 = 0, *t8 = 0,
        *t9 = 0, *t10 = 0, *t11 = 0, *t12 = 0, *t6Old = 0, *t7Old = 0;
    Node *T3 = 0, *T4 = 0, *T5 = 0, *T6 = 0,
        *T7 = 0, *T8 = 0, *T9 = 0, *T10 = 0;
    Candidate *Nt2, *Nt4;
    GainType G1, G2, G3, G4, BestG4 = MINUS_INFINITY,
        G5, G6, G7, G8, BestG8 = MINUS_INFINITY, NewPenalty;
    int X4, X6, Case6 = 0, BestCase6 = 0, Case56, Case78, Case10,
        BestCase10 = 0, Case;
    int Breadth2 = 0, Breadth4;

    OldSwaps = Swaps;
    if (t2 != SUC(t1))
        Reversed ^= 1;
    K = Swaps == 0 ? MoveType : SubsequentMoveType;

    /* Choose (t2,t3) as a candidate edge emanating from t2 */
    for (Nt2 = t2->CandidateSet; (t3 = Nt2->To); Nt2++) {
        if (t3 == t2->Pred || t3 == t2->Suc ||
            ((G1 = *G0 - Nt2->Cost) <= 0 &&
             GainCriterionUsed &&
             ProblemType != HCP && ProblemType != HPP))
            continue;
        if (++Breadth2 > MaxBreadth)
            break;
        /* Choose t4 as one of t3's two neighbors on the tour */
        for (X4 = 1; X4 <= 2; X4++) {
            t4 = X4 == 1 ? PRED(t3) : SUC(t3);
            if (FixedOrCommon(t3, t4))
                continue;
            G2 = G1 + C(t3, t4);
            if (X4 == 1 && !Forbidden(t4, t1) &&
                (CurrentPenalty != 0 ||
                 TSPTW_Makespan || !c || G2 - c(t4, t1) > 0)) {
                *Gain = G2 - C(t4, t1);
                if (CurrentPenalty != 0 || TSPTW_Makespan || *Gain > 0) {
                    Swap1(t1, t2, t3);
                    if (Improvement(Gain, t1, t2))
                        return 0;
                }
            }
            if (Backtracking && !Excludable(t3, t4))
                continue;
            Breadth4 = 0;
            /* Try 3-opt move */
            /* Choose (t4,t5) as a candidate edge emanating from t4 */
            for (Nt4 = t4->CandidateSet; (t5 = Nt4->To); Nt4++) {
                if (t5 == t4->Pred || t5 == t4->Suc ||
                    ((G3 = G2 - Nt4->Cost) <= 0 &&
                     GainCriterionUsed &&
                     ProblemType != HCP && ProblemType != HPP) ||
                    (X4 == 2 && !BETWEEN(t2, t5, t3)))
                    continue;
                if (++Breadth4 > MaxBreadth)
                    break;
                /* Choose t6 as one of t5's two neighbors on the tour */
                for (X6 = 1; X6 <= X4; X6++) {
                    if (X4 == 1) {
                        Case6 = 1 + !BETWEEN(t2, t5, t4);
                        t6 = Case6 == 1 ? SUC(t5) : PRED(t5);
                    } else {
                        Case6 = 4 + X6;
                        t6 = X6 == 1 ? SUC(t5) : PRED(t5);
                        if (t6 == t1)
                            continue;
                    }
                    if (FixedOrCommon(t5, t6))
                        continue;
                    G4 = G3 + C(t5, t6);
                    if (!Forbidden(t6, t1) &&
                        (CurrentPenalty != 0 ||
                         TSPTW_Makespan || !c || G4 - c(t6, t1) > 0)) {
                        *Gain = G4 - C(t6, t1);
                        if (CurrentPenalty != 0 ||
                            TSPTW_Makespan || *Gain > 0) {
                            NewPenalty =
                                BIT_LoadDiff3Opt(t1, t2, t3, t4, t5, t6) -
                                Capacity;
                            if (NewPenalty < CurrentPenalty ||
                                (NewPenalty == CurrentPenalty
                                 && *Gain > 0)) {
                                Make3OptMove(t1, t2, t3, t4, t5, t6,
                                             Case6);
                                if (Improvement(Gain, t1, t2))
                                    return 0;
                            }
                        }
                    }
                    if (K < 5) {
                        if (GainCriterionUsed && G4 - Precision < t6->Cost)
                            continue;
                        if (!Backtracking || Swaps > 0) {
                            if ((G4 > BestG4 ||
                                 (G4 == BestG4 && !Near(t5, t6) &&
                                  Near(T5, T6))) &&
                                Swaps < MaxSwaps &&
                                Excludable(t5, t6) &&
                                !InInputTour(t5, t6)) {
                                /* Ignore the move if the gain does not vary */
                                if (RestrictedSearch &&
                                    ProblemType != HCP &&
                                    ProblemType != HPP &&
                                    G2 - t4->Pi == G4 - t6->Pi &&
                                    G3 + t5->Pi == G1 + t3->Pi)
                                    continue;
                                T3 = t3;
                                T4 = t4;
                                T5 = t5;
                                T6 = t6;
                                BestCase6 = Case6;
                                BestG4 = G4;
                            }
                        } else if (MaxSwaps > 0) {
                            GainType G = G4;
                            Node *t = t6;
                            Make3OptMove(t1, t2, t3, t4, t5, t6, Case6);
                            Exclude(t1, t2);
                            Exclude(t3, t4);
                            Exclude(t5, t6);
                            while ((t =
                                    BestSubsequentMove(t1, t, &G, Gain)));
                            if (PenaltyGain > 0 || *Gain > 0)
                                return 0;
                            OldSwaps = 0;
                            RestoreTour();
                            if (t2 != SUC(t1))
                                Reversed ^= 1;
                        }
                    }
                }
                if (t6 != t1 && !Forbidden(t6, t1) &&
                    3 + 1 < NonsequentialMoveType &&
                    PatchingC >= 2 && PatchingA >= 1 &&
                    (Swaps == 0 || SubsequentPatching)) {
                    G5 = G4 - C(t6, t1);
                    if ((PatchingCRestricted ? G5 > 0 && IsCandidate(t6, t1) :
                         PatchingCExtended ? G5 > 0
                         || IsCandidate(t6, t1) : G5 > 0)) {
                        incl[incl[2] = 3] = 2;
                        incl[incl[4] = 5] = 4;
                        incl[incl[1] = 6] = 1;
                        t[1] = t1; t[2] = t2;
                        t[3] = t3; t[4] = t4;
                        t[5] = t5; t[6] = t6;
                        MarkDeleted(t1, t2);
                        MarkAdded(t2, t3);
                        MarkDeleted(t3, t4);
                        MarkAdded(t4, t5);
                        MarkDeleted(t5, t6);
                        *Gain = PatchCycles(3, G5);
                        UnmarkDeleted(t1, t2);
                        UnmarkAdded(t2, t3);
                        UnmarkDeleted(t3, t4);
                        UnmarkAdded(t4, t5);
                        UnmarkDeleted(t5, t6);
                        if (PenaltyGain > 0 || *Gain > 0)
                            return 0;
                    }
                }
            }
            if (t4 != t1 && !Forbidden(t4, t1) &&
                2 + 1 < NonsequentialMoveType &&
                PatchingC >= 2 && PatchingA >= 1 &&
                (Swaps == 0 || SubsequentPatching)) {
                G3 = G2 - C(t4, t1);
                if ((PatchingCRestricted ? G3 > 0 && IsCandidate(t4, t1) :
                    PatchingCExtended ? G3 > 0
                    || IsCandidate(t4, t1) : G3 > 0)) {
                    incl[incl[2] = 3] = 2;
                    incl[incl[4] = 1] = 4;
                    t[1] = t1; t[2] = t2;
                    t[3] = t3; t[4] = t4;
                    MarkDeleted(t1, t2);
                    MarkAdded(t2, t3);
                    MarkDeleted(t3, t4);
                    *Gain = PatchCycles(2, G3);
                    UnmarkDeleted(t1, t2);
                    UnmarkAdded(t2, t3);
                    UnmarkDeleted(t3, t4);
                    if (PenaltyGain > 0 || *Gain > 0)
                        return 0;
                }
            }
            if (X4 == 1)
                continue;
            /* Try special 3-opt */
            for (Case56 = 1; Case56 <= 2; Case56++) {
                if (Case56 == 1) {
                    SelectBackward(&t6, &t5, t3, t2);
                    if (t6 == t2)
                        break;
                    t6Old = t6;
                } else {
                    SelectForward(&t5, &t6, t2, t3);
                    if (t5 == t3 || t6 == t6Old)
                        break;
                }
                if (!Forbidden(t4, t5) && !Forbidden(t6, t1)) {
                    G3 = G2 - C(t4, t5);
                    G4 = G3 + C(t5, t6);
                    if (CurrentPenalty != 0 ||
                        TSPTW_Makespan || !c || G4 - c(t6, t1) > 0) {
                        *Gain = G4 - C(t6, t1);
                        if (CurrentPenalty != 0 ||
                            TSPTW_Makespan || *Gain > 0) {
                            NewPenalty =
                                BIT_LoadDiff3Opt(t1, t2, t3, t4, t5, t6) -
                                Capacity;
                            if (NewPenalty < CurrentPenalty ||
                                (NewPenalty == CurrentPenalty &&
                                 *Gain > 0)) {
                                Make3OptMove(t1, t2, t3, t4, t5, t6, 5);
                                if (Improvement(Gain, t1, t2))
                                    return 0;
                            }
                        }
                    }
                }
                if (!Asymmetric &&
                    !Forbidden(t4, t6) && !Forbidden(t5, t1)) {
                    G4 = G2 - C(t4, t6) + C(t5, t6);
                    if (CurrentPenalty != 0 ||
                        TSPTW_Makespan || !c || G4 - c(t5, t1) > 0) {
                        *Gain = G4 - C(t5, t1);
                        if (CurrentPenalty != 0 ||
                            TSPTW_Makespan || *Gain > 0) {
                            Make3OptMove(t1, t2, t3, t4, t6, t5, 6);
                            if (Improvement(Gain, t1, t2))
                                return 0;
                        }
                    }
                }
            }
            /* Try special 4-opt */
            if (!Forbidden(t4, t1)) {
                G3 = G2 - C(t4, t1);
                if (!Asymmetric) {
                    SelectBackward(&t6, &t5, t3, t2);
                    if (t6 != t2 && !FixedOrCommon(t5, t6)) {
                        SelectForward(&t8, &t7, t4, t1);
                        if (t8 != t1 &&
                            !Forbidden(t6, t7) &&
                            !Forbidden(t8, t5) && !FixedOrCommon(t7, t8)) {
                            *Gain = G3 + C(t5, t6) - C(t6, t7) +
                                C(t7, t8) - C(t8, t5);
                            if (CurrentPenalty != 0 ||
                                TSPTW_Makespan || *Gain > 0) {
                                Swap2(t5, t6, t7, t1, t2, t3);
                                if (Improvement(Gain, t1, t2))
                                    return 0;
                            }
                        }
                    }
                }
                for (Case56 = 1; Case56 <= 2; Case56++) {
                    if (Case56 == 1) {
                        SelectBackward(&t6, &t5, t3, t2);
                        if (t6 == t2)
                            break;
                        t6Old = t6;
                    } else {
                        SelectForward(&t5, &t6, t2, t3);
                        if (t5 == t3 || t6 == t6Old)
                            break;
                    }
                    for (Case78 = 1; Case78 <= 2; Case78++) {
                        if (Case78 == 1) {
                            SelectBackward(&t8, &t7, t1, t4);
                            if (t8 == t4)
                                break;
                            t7Old = t7;
                        } else {
                            SelectForward(&t7, &t8, t4, t1);
                            if (t7 == t1 || t7 == t7Old)
                                break;
                        }
                        G4 = G3 + C(t5, t6) + C(t7, t8);
                        if (!Forbidden(t6, t7) && !Forbidden(t8, t5)) {
                            if (CurrentPenalty != 0 ||
                                TSPTW_Makespan ||
                                !c || G4 - c(t6, t7) - c(t8, t5) > 0) {
                                *Gain = G4 - C(t6, t7) - C(t8, t5);
                                if (CurrentPenalty != 0 ||
                                    TSPTW_Makespan || *Gain > 0) {
                                    NewPenalty =
                                        BIT_LoadDiff4Opt(t1, t2, t3, t4,
                                                         t5, t6, t7, t8) -
                                        Capacity;
                                    if (NewPenalty < CurrentPenalty ||
                                        (NewPenalty == CurrentPenalty &&
                                         *Gain > 0)) {
                                        Swap3(t1, t2, t4, t7, t8, t5,
                                              t1, t3, t2);
                                        if (Improvement(Gain, t1, t2))
                                            return 0;
                                    }
                                }
                            }
                        }
                        if (!Asymmetric &&
                            !Forbidden(t5, t7) && !Forbidden(t6, t8)) {
                            if (CurrentPenalty != 0 ||
                                TSPTW_Makespan ||
                                !c || G4 - c(t5, t7) - c(t6, t8) > 0) {
                                *Gain = G4 - C(t5, t7) - C(t6, t8);
                                if (CurrentPenalty != 0 ||
                                    TSPTW_Makespan || *Gain > 0) {
                                    Swap2(t5, t6, t8, t1, t2, t3);
                                    if (Improvement(Gain, t1, t2))
                                        return 0;
                                }
                            }
                        }
                    }
                }
            }
            if (K < 5)
                continue;
            /* Try special 5-opt */
            for (Case = 1; Case <= 33; Case++) {
                if (Case == 1 && (t4 == t1 || SUC(t4) == t1))
                    Case = 13;
                if (Case == 13 && PRED(PRED(t3)) == t2)
                    Case = 25;
                if (Case == 25 && (t4 == t1 || SUC(t4) == t1))
                    Case = 31;
                if (Case == 31 && PRED(PRED(t3)) == t2)
                    break;
                if (Case <= 6) {
                    if (Case <= 3) {
                        if (Case == 1) {
                            SelectBackward(&t8, &t7, t3, t2);
                            if (t8 == t2) {
                                Case = 3;
                                continue;
                            }
                        }
                    } else {
                        SelectForward(&t7, &t8, t2, t3);
                        if (t7 == t3)
                            continue;
                    }
                    if (Case == 1 || Case == 2 || Case == 4 || Case == 5) {
                        SelectForward(&t9, &t10, t4, t1);
                        if (t9 == t1)
                            continue;
                        if (Case == 1 || Case == 4) {
                            SelectForward(&t5, &t6, t10, t1);
                            if (t5 == t1)
                                continue;
                        } else {
                            SelectBackward(&t6, &t5, t1, t10);
                            if (t6 == t10)
                                continue;
                        }
                    } else {
                        SelectBackward(&t6, &t5, t1, t4);
                        if (t6 == t4)
                            continue;
                        SelectBackward(&t10, &t9, t5, t4);
                        if (t10 == t4)
                            continue;
                    }
                    Case10 = 4;
                } else if (Case <= 12) {
                    if (Case <= 9) {
                        SelectBackward(&t10, &t9, t3, t2);
                        if (t10 == t2)
                            continue;
                    } else {
                        SelectForward(&t9, &t10, t2, t3);
                        if (t9 == t3)
                            continue;
                    }
                    if (Case == 7 || Case == 8 || Case == 10 || Case == 11) {
                        SelectForward(&t7, &t8, t4, t1);
                        if (t7 == t1)
                            continue;
                        if (Case == 7 || Case == 10) {
                            SelectForward(&t5, &t6, t8, t1);
                            if (t5 == t1)
                                continue;
                        } else {
                            SelectBackward(&t6, &t5, t1, t8);
                            if (t6 == t8)
                                continue;
                        }
                    } else {
                        SelectBackward(&t6, &t5, t1, t4);
                        if (t6 == t4)
                            continue;
                        SelectBackward(&t8, &t7, t5, t4);
                        if (t8 == t4)
                            continue;
                    }
                    Case10 = 5;
                } else if (Case <= 18) {
                    if (Case <= 15) {
                        SelectForward(&t7, &t8, t4, t1);
                        if (t7 == t1)
                            continue;
                    } else {
                        SelectBackward(&t8, &t7, t1, t4);
                        if (t8 == t4)
                            continue;
                    }
                    if (Case == 13 || Case == 14 ||
                        Case == 16 || Case == 17) {
                        SelectBackward(&t6, &t5, t3, t2);
                        if (t6 == t2)
                            continue;
                        if (Case == 13 || Case == 16) {
                            SelectBackward(&t10, &t9, t5, t2);
                            if (t10 == t2)
                                continue;
                        } else {
                            SelectForward(&t9, &t10, t2, t5);
                            if (t9 == t5)
                                continue;
                        }
                    } else {
                        SelectForward(&t9, &t10, t2, t3);
                        if (t9 == t3)
                            continue;
                        SelectForward(&t5, &t6, t10, t3);
                        if (t5 == t3)
                            continue;
                    }
                    Case10 = 13;
                } else if (Case <= 24) {
                    if (Case <= 21) {
                        SelectForward(&t7, &t8, t4, t1);
                        if (t7 == t1)
                            continue;
                    } else {
                        SelectBackward(&t8, &t7, t1, t4);
                        if (t8 == t4)
                            continue;
                    }
                    if (Case == 19 || Case == 20 ||
                        Case == 22 || Case == 23) {
                        SelectBackward(&t10, &t9, t3, t2);
                        if (t10 == t2)
                            continue;
                        if (Case == 19 || Case == 22) {
                            SelectBackward(&t6, &t5, t9, t2);
                            if (t6 == t2)
                                continue;
                        } else {
                            SelectForward(&t5, &t6, t2, t9);
                            if (t5 == t9)
                                continue;
                        }
                    } else {
                        SelectForward(&t5, &t6, t2, t3);
                        if (t5 == t3)
                            continue;
                        SelectForward(&t9, &t10, t6, t3);
                        if (t9 == t3)
                            continue;
                    }
                    Case10 = 14;
                } else if (Case <= 30) {
                    if (Case <= 27) {
                        SelectBackward(&t6, &t5, t3, t2);
                        if (t6 == t2)
                            continue;
                    } else {
                        SelectForward(&t5, &t6, t2, t3);
                        if (t5 == t3)
                            continue;
                    }
                    if (Case == 25 || Case == 26 ||
                        Case == 28 || Case == 29) {
                        SelectForward(&t9, &t10, t4, t1);
                        if (t9 == t1)
                            continue;
                        if (Case == 25 || Case == 28) {
                            SelectForward(&t7, &t8, t10, t1);
                            if (t7 == t1)
                                continue;
                        } else {
                            SelectBackward(&t8, &t7, t1, t10);
                            if (t8 == t10)
                                continue;
                        }
                    } else {
                        SelectBackward(&t8, &t7, t1, t4);
                        if (t8 == t4)
                            continue;
                        SelectBackward(&t10, &t9, t7, t4);
                        if (t10 == t4)
                            continue;
                    }
                    Case10 = 15;
                } else if (Case == 31) {
                    SelectBackward(&t8, &t7, t3, t2);
                    if (t8 == t2)
                        continue;
                    SelectBackward(&t10, &t9, t7, t2);
                    if (t10 == t2)
                        continue;
                    SelectBackward(&t6, &t5, t9, t2);
                    if (t6 == t2)
                        continue;
                    Case10 = 14;
                } else if (Case == 32) {
                    SelectBackward(&t6, &t5, t3, t2);
                    if (t6 == t2)
                        continue;
                    SelectBackward(&t8, &t7, t5, t2);
                    if (t8 == t2)
                        continue;
                    SelectBackward(&t10, &t9, t7, t2);
                    if (t10 == t2)
                        continue;
                    Case10 = 15;
                } else {
                    SelectBackward(&t10, &t9, t3, t2);
                    if (t10 == t2)
                        continue;
                    SelectBackward(&t6, &t5, t9, t2);
                    if (t6 == t2)
                        continue;
                    SelectBackward(&t8, &t7, t5, t2);
                    if (t8 == t2)
                        continue;
                    Case10 = 15;
                }
                if (t10 == t1 ||
                    t5 == t4 ||
                    Forbidden(t4, t5) ||
                    Forbidden(t6, t7) ||
                    Forbidden(t8, t9) ||
                    Forbidden(t10, t1) ||
                    FixedOrCommon(t5, t6) ||
                    FixedOrCommon(t7, t8) || FixedOrCommon(t9, t10))
                    continue;
                G3 = G2 - C(t4, t5);
                G4 = G3 + C(t5, t6);
                G5 = G4 - C(t6, t7);
                G6 = G5 + C(t7, t8);
                G7 = G6 - C(t8, t9);
                G8 = G7 + C(t9, t10);
                *Gain = G8 - C(t10, t1);
                if (CurrentPenalty != 0 || TSPTW_Makespan || *Gain > 0) {
                    NewPenalty =
                        BIT_LoadDiff5Opt(t1, t2, t3, t4, t5, t6,
                                         t7, t8, t9, t10, Case10) -
                        Capacity;
                    if (NewPenalty < CurrentPenalty ||
                        (NewPenalty == CurrentPenalty && *Gain > 0)) {
                        Make5OptMove(t1, t2, t3, t4, t5, t6, t7,
                                     t8, t9, t10, Case10);
                        if (Improvement(Gain, t1, t2))
                            return 0;
                    }
                }
                if (GainCriterionUsed && G8 - Precision < t10->Cost)
                    continue;
                if (!Backtracking || Swaps > 0) {
                    if ((G8 > BestG8 || (G8 == BestG8 && !Near(t9, t10)
                                         && Near(T9, T10)))
                        && Swaps < MaxSwaps && Excludable(t9, t10)
                        && !InInputTour(t9, t10)) {
                        /* Ignore the move if the gain does not vary */
                        if (RestrictedSearch &&
                            ProblemType != HCP &&
                            ProblemType != HPP &&
                            G2 - t4->Pi == G4 - t6->Pi &&
                            G4 - t6->Pi == G6 - t8->Pi &&
                            G6 - t8->Pi == G8 - t10->Pi &&
                            G3 + t5->Pi == G1 + t3->Pi &&
                            G5 + t7->Pi == G3 + t5->Pi &&
                            G7 + t9->Pi == G5 + t7->Pi)
                            continue;
                        T3 = t3;
                        T4 = t4;
                        T5 = t5;
                        T6 = t6;
                        T7 = t7;
                        T8 = t8;
                        T9 = t9;
                        T10 = t10;
                        BestCase10 = Case10;
                        BestG8 = G8;
                    }
                } else if (MaxSwaps > 0) {
                    GainType G = G8;
                    Node *t = t10;
                    Make5OptMove(t1, t2, t3, t4, t5,
                                 t6, t7, t8, t9, t10, Case10);
                    Exclude(t1, t2);
                    Exclude(t3, t4);
                    Exclude(t5, t6);
                    Exclude(t7, t8);
                    Exclude(t9, t10);
                    while ((t = BestSubsequentMove(t1, t, &G, Gain)));
                    if (*Gain > 0)
                        return 0;
                    RestoreTour();
                    if (t2 != SUC(t1))
                        Reversed ^= 1;
                }
            }
            /* Try special 6-opt */
            t7Old = 0;
            for (Case = 1; Case <= 8; Case++) {
                if (Case <= 4) {
                    if (Case == 1) {
                        SelectBackward(&t8, &t7, t3, t2);
                        if (t8 == t2) {
                            Case = 4;
                            continue;
                        }
                        t7Old = t7;
                    }
                } else {
                    SelectForward(&t7, &t8, t2, t3);
                    if (t7 == t3 || t7 == t7Old)
                        break;
                }
                if (Case == 4 || Case == 8) {
                    SelectBackward(&t12, &t11, t1, t4);
                    if (t12 == t4)
                        continue;
                    SelectBackward(&t6, &t5, t11, t4);
                    if (t6 == t4)
                        continue;
                    SelectBackward(&t10, &t9, t5, t4);
                    if (t10 == t4)
                        continue;
                } else {
                    SelectForward(&t9, &t10, t4, t1);
                    if (t9 == t1)
                        continue;
                    if (Case == 3 || Case == 7) {
                        SelectBackward(&t12, &t11, t1, t10);
                        if (t12 == t10)
                            continue;
                        SelectBackward(&t6, &t5, t11, t10);
                        if (t6 == t10)
                            continue;
                    } else {
                        SelectForward(&t5, &t6, t10, t1);
                        if (t5 == t1)
                            continue;
                        if (Case == 1 || Case == 5) {
                            SelectForward(&t11, &t12, t6, t1);
                            if (t11 == t1)
                                continue;
                        } else {
                            SelectBackward(&t12, &t11, t1, t6);
                            if (t12 == t6)
                                continue;
                        }
                    }
                }
                if (t6 == t1 ||
                    t12 == t7 ||
                    Forbidden(t4, t5) ||
                    Forbidden(t6, t1) ||
                    Forbidden(t8, t9) ||
                    Forbidden(t10, t11) ||
                    Forbidden(t12, t7) ||
                    FixedOrCommon(t5, t6) ||
                    FixedOrCommon(t7, t8) ||
                    FixedOrCommon(t9, t10) || FixedOrCommon(t11, t12))
                    continue;
                G4 = G2 + C(t5, t6) + C(t7, t8) + C(t9, t10) + C(t11, t12);
                if (CurrentPenalty != 0 ||
                    TSPTW_Makespan ||
                    !c || G4 - c(t4, t5) - c(t6, t1) - c(t8, t9)
                    - c(t10, t11) - c(t12, t7) > 0) {
                    *Gain = G4 - C(t4, t5) - C(t6, t1) - C(t8, t9)
                        - C(t10, t11) - C(t12, t7);
                    if (CurrentPenalty != 0 || TSPTW_Makespan || *Gain > 0) {
                        NewPenalty =
                            BIT_LoadDiff6Opt(t1, t2, t3, t4, t5, t6,
                                             t7, t8, t9, t10, t11, t12) -
                            Capacity;
                        if (NewPenalty < CurrentPenalty ||
                            (NewPenalty == CurrentPenalty && *Gain > 0)) {
                            Swap5(t1, t2, t4, t7, t8, t9, t2, t4, t5,
                                  t7, t10, t11, t6, t2, t3);
                            if (Improvement(Gain, t1, t2))
                                return 0;
                        }
                    }
                }
            }
        }
    }
    *Gain = PenaltyGain = 0;
    if (T10) {
        /* Make the best 5-opt move */
        Make5OptMove(t1, t2, T3, T4, T5, T6, T7, T8, T9, T10, BestCase10);
        Exclude(t1, t2);
        Exclude(T3, T4);
        Exclude(T5, T6);
        Exclude(T7, T8);
        Exclude(T9, T10);
        *G0 = BestG8;
        return T10;
    }
    if (T6) {
        /* Make the best 3-opt move */
        Make3OptMove(t1, t2, T3, T4, T5, T6, BestCase6);
        Exclude(t1, t2);
        Exclude(T3, T4);
        Exclude(T5, T6);
        *G0 = BestG4;
        return T6;
    }
    return 0;
}

static void SelectForward(Node ** a, Node ** b, Node * from, Node * to)
{
    *a = from;
    *b = SUC(*a);
    while (*a != to && FixedOrCommon(*a, *b)) {
        *a = *b;
        *b = SUC(*a);
    }
}

static void SelectBackward(Node ** a, Node ** b, Node * from, Node * to)
{
    *a = from;
    *b = PRED(*a);
    while (*a != to && FixedOrCommon(*a, *b)) {
        *a = *b;
        *b = PRED(*a);
    }
}
