#include "LKH.h"
#include "Segment.h"

static int *Queue;

GainType Penalty_PDTSPF()
{
    static Node *StartRoute = 0;
    Node *N, *NextN, *CurrentRoute;
    GainType P = 0;
    int Forward = SUCC(Depot)->Id != Depot->Id + DimensionSaved;
    int QueueLength, Front, Back, Capacity = Dim;

    if (!Queue)
        Queue = (int *) malloc(Capacity * sizeof(int));
    if (!StartRoute)
        StartRoute = Depot;
    if (StartRoute->Id > DimensionSaved)
        StartRoute -= DimensionSaved;
    N = StartRoute;
    do {
        CurrentRoute = N;
        QueueLength = Front = 0;
        Back = -1;
        do {
            if (N->Id <= Dim && N != Depot) {
                if (N->Pickup) {
                    if (QueueLength > 0) {
                        if (Queue[Front] != N->Id)
                            P++;
                        Front = (Front + 1) % Capacity;
                        QueueLength--;
                    } else
                        P++;
                    if (P > CurrentPenalty ||
                        (P == CurrentPenalty && CurrentGain <= 0)) {
                        StartRoute = CurrentRoute;
                        return CurrentPenalty + (CurrentGain > 0);
                    }
                } else if (N->Delivery) {
                    Back = (Back + 1) % Capacity;
                    Queue[Back] = N->Delivery;
                    QueueLength++;
                }
            }
            NextN = Forward ? SUCC(N) : PREDD(N);
            N = Forward ? SUCC(NextN) : PREDD(NextN);
        } while (N->DepotId == 0);
        P += QueueLength;
    } while (N != StartRoute);
    return P;
}
