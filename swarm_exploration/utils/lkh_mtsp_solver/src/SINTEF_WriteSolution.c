#include "LKH.h"

void SINTEF_WriteSolution(char *FileName, GainType Cost)
{
    FILE *ResultFile;
    Node *N, *NextN;
    char *FullFileName;
    int Route, Forward;
    time_t Now;

    if (FileName == 0)
        return;
    FullFileName = FullName(FileName, Cost);
    Now = time(&Now);
    if (TraceLevel >= 1)
        printff("Writing SINTEF_SOLUTION_FILE: \"%s\" ... ",
                FullFileName);
    assert(ResultFile = fopen(FullFileName, "w"));
    fprintf(ResultFile, "Instance name : %s\n", Name);
    fprintf(ResultFile, "Authors       : Keld Helsgaun\n");
    fprintf(ResultFile, "Date          : %s", ctime(&Now));
    fprintf(ResultFile, "Reference     : "
            "http://webhotel4.ruc.dk/~keld/research/LKH-3\n");
    fprintf(ResultFile, "Solution\n");
    N = Depot;
    Forward = N->Suc->Id != N->Id + DimensionSaved;
    Route = 0;
    do {
        Route++;
        fprintf(ResultFile, "Route %d : ", Route);
        do {
            if (N->Id <= Dim && N != Depot)
                fprintf(ResultFile, "%d ", N->Id - 1);
            NextN = Forward ? N->Suc : N->Pred;
            if (NextN->Id > DimensionSaved)
                NextN = Forward ? NextN->Suc : NextN->Pred;
            N = NextN;
        } while (N->DepotId == 0);
        fprintf(ResultFile, "\n");
    } while (N != Depot);
    fclose(ResultFile);
    if (TraceLevel >= 1)
        printff("done\n");
}
