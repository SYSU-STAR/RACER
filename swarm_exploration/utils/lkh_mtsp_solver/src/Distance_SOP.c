#include "LKH.h"

/*
 * The Distance_SOP function computes the distance for a SOP instance.
 */

int Distance_SOP(Node * Na, Node * Nb)
{
    int d = OldDistance(Na, Nb);
    return d >= 0 ? d : M;
}
