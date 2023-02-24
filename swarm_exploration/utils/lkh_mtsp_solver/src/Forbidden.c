#include "LKH.h"

/*
 * The Forbidden function is used to test if an edge, (Na,Nb),
 * is not allowed to belong to a tour.
 * If the edge is forbidden, the function returns 1; otherwise 0.
 */

int Forbidden(Node * Na, Node * Nb)
{
    if (Na == Nb)
        return 1;
    if ((Salesmen == 1 &&
         (ProblemType == TSP ||
          ProblemType == HCP || ProblemType == HPP)) ||
        InInitialTour(Na, Nb) ||
        Na->Id == 0 || Nb->Id == 0)
        return 0;
    if (Asymmetric &&
        (Na->Id <= DimensionSaved) == (Nb->Id <= DimensionSaved))
        return 1;
    if (ProblemType == SOP &&
        ((Na->Id == 1 && Nb->Id == Dimension + 1) ||
         (Na->Id == Dimension + 1 && Nb->Id == 1)))
        return 1;
    if (Na->Head && !FixedOrCommon(Na, Nb) &&
        (Na->Head == Nb->Head ||
         (Na->Head != Na && Na->Tail != Na) ||
         (Nb->Head != Nb && Nb->Tail != Nb)))
        return 1;
    if (Salesmen > 1 && Dimension == DimensionSaved) {
        if (Na->DepotId) {
            if ((Nb->DepotId && MTSPMinSize >= 1) ||
                (Nb->Special &&
                 Nb->Special != Na->DepotId &&
                 Nb->Special != Na->DepotId % Salesmen + 1))
                return 1;
        }
        if (Nb->DepotId) {
            if ((Na->DepotId && MTSPMinSize >= 1) ||
                (Na->Special &&
                 Na->Special != Nb->DepotId &&
                 Na->Special != Nb->DepotId % Salesmen + 1))
                return 1;
        }
    }
    if (Salesmen > 1)
        return 0;
    if (ProblemType == PDTSP || ProblemType == PDPTW ||
        ProblemType == PDTSPL || ProblemType == PDTSPF) {
        if (Na->Id <= Dim) {
            Nb = &NodeSet[Nb->Id - Dim];
            if (Na == Depot && Nb->Pickup)
                return 1;
            if (Na->Delivery && Nb == Depot)
                return 1;
            if (ProblemType == PDTSPL) {
                if (Na->Pickup && Nb->Delivery == Na->Id)
                    return 1;
                if (Na->Delivery && Nb->Pickup && Nb->Pickup != Na->Id)
                    return 1;
            }
        } else {
            Na = &NodeSet[Na->Id - Dim];
            if (Na == Depot && Nb->Delivery)
                return 1;
            if (Na->Pickup && Nb == Depot)
                return 1;
            if (ProblemType == PDTSPL) {
                if (Na->Pickup && Nb->Delivery && Nb->Delivery != Na->Id)
                    return 1;
                if (Na->Delivery && Nb->Pickup == Na->Id)
                    return 1;
            }
        }
    }
    return 0;
}
