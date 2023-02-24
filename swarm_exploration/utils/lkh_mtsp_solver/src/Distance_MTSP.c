#include "LKH.h"

/*
 * The Distance_MTSP function computes the transformed distance for
 * an mTSP instance.
 */

int Distance_MTSP(Node * Na, Node * Nb)
{
    const int M = INT_MAX / 2 / Precision;
    int i;

    if (Fixed(Na, Nb))
        return 0;
    if (Forbidden(Na, Nb))
        return M;
    if (Na->DepotId != 0 && Nb->DepotId != 0)
        return 0;
    if (DimensionSaved != Dimension) {
        if (Nb->DepotId && Na->Id <= Dim) {
            for (i = ExternalSalesmen; i >= 1; i--)
                if (Nb->DepotId == i)
                    break;
            if (i >= 1)
                return 0;
        }
        if (Na->DepotId && Nb->Id <= Dim) {
            for (i = ExternalSalesmen; i >= 1; i--)
                if (Na->DepotId == i)
                    break;
            if (i >= 1)
                return 0;
        }
        if (Na->DepotId != 0)
            Na = Na->Id <= DimensionSaved ? Depot :
                &NodeSet[Depot->Id + DimensionSaved];
        else if (Nb->DepotId != 0)
            Nb = Nb->Id <= DimensionSaved ? Depot :
                &NodeSet[Depot->Id + DimensionSaved];
    } else if (Dim != Dimension) {
        if (Na->Id > Dim)
            Na = Depot;
        if (Nb->Id > Dim)
            Nb = Depot;
    }
    return OldDistance(Na, Nb);
}
