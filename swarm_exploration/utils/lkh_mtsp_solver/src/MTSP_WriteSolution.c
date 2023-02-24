#include "LKH.h"

void MTSP_WriteSolution(char *FileName, GainType Penalty, GainType Cost)
{
    FILE *SolutionFile;
    Node *N, *NextN;
    int Size, Forward;
    char *FullFileName;
    GainType Sum;

    if (FileName == 0)
        return;
    FullFileName = FullName(FileName, Cost);
    if (TraceLevel >= 1)
        printff("Writing MTSP_SOLUTION_FILE: \"%s\" ... ", FullFileName);
    assert((SolutionFile = fopen(FullFileName, "w")));
    fprintf(SolutionFile, "%s, Cost: " GainFormat "_" GainFormat "\n",
            Name, Penalty, Cost);
    fprintf(SolutionFile, "The tours traveled by the %d salesmen are:\n",
            Salesmen);
    N = Depot;
    Forward = N->Suc->Id != N->Id + DimensionSaved;
    do {
        Sum = 0;
        Size = -1;
        do {
            fprintf(SolutionFile, "%d ", N->Id <= Dim ? N->Id : Depot->Id);
            NextN = Forward ? N->Suc : N->Pred;
            Sum += C(N, NextN) - N->Pi - NextN->Pi;
            Size++;
            if (NextN->Id > DimensionSaved)
                NextN = Forward ? NextN->Suc : NextN->Pred;
            N = NextN;
        } while (N->DepotId == 0);
        if (N->DepotId <= ExternalSalesmen)
            fprintf(SolutionFile, "(#%d)  Cost: " GainFormat "\n",
                    Size, Sum / Precision);
        else
            fprintf(SolutionFile, "%d (#%d)  Cost: " GainFormat "\n",
                    Depot->Id, Size, Sum / Precision);
    } while (N != Depot);
    fclose(SolutionFile);
    if (TraceLevel >= 1)
        printff("done\n");
}
