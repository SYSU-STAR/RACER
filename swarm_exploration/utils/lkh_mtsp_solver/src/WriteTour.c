#include "LKH.h"

/*
 * The WriteTour function writes a tour to file. The tour 
 * is written in TSPLIB format to file FileName. 
 * 
 * The tour is written in "normal form": starting at node 1,
 * and continuing in direction of its lowest numbered
 * neighbor.
 * 
 * Nothing happens if FileName is 0. 
 */

void WriteTour(char *FileName, int *Tour, GainType Cost)
{
    FILE *TourFile;
    int i, j, k, n, Forward, a, b;
    char *FullFileName;
    time_t Now;

    if (CurrentPenalty != 0 && MTSPObjective == -1 &&
        ProblemType != CCVRP && ProblemType != TRP &&
        ProblemType != MLP)
        return;
    if (FileName == 0)
        return;
    FullFileName = FullName(FileName, Cost);
    Now = time(&Now);
    if (TraceLevel >= 1)
        printff("Writing%s: \"%s\" ... ",
                FileName == TourFileName ? " TOUR_FILE" :
                FileName == OutputTourFileName ? " OUTPUT_TOUR_FILE" : "",
                FullFileName);
    TourFile = fopen(FullFileName, "w");
    if (CurrentPenalty == 0) {
        fprintf(TourFile, "NAME : %s." GainFormat ".tour\n", Name, Cost);
        fprintf(TourFile, "COMMENT : Length = " GainFormat "\n", Cost);
    } else {
        fprintf(TourFile, "NAME : %s." GainFormat "_" GainFormat ".tour\n",
                Name, CurrentPenalty, Cost);
        fprintf(TourFile,
                "COMMENT : Cost = " GainFormat "_" GainFormat "\n",
                CurrentPenalty, Cost);
    }
    fprintf(TourFile, "COMMENT : Found by LKH [Keld Helsgaun] %s",
            ctime(&Now));
    fprintf(TourFile, "TYPE : TOUR\n");
    fprintf(TourFile, "DIMENSION : %d\n", DimensionSaved);
    fprintf(TourFile, "TOUR_SECTION\n");

    n = DimensionSaved;
    for (i = 1; i < n && Tour[i] != 1; i++);
    Forward = Asymmetric ||
        Tour[i < n ? i + 1 : 1] < Tour[i > 1 ? i - 1 : Dimension];
    for (j = 1; j <= n; j++) {
        if ((a = Tour[i]) <= n)
            fprintf(TourFile, "%d\n", 
                    ProblemType != STTSP ? a : NodeSet[a].OriginalId);
        if (Forward) {
            if (++i > n)
                i = 1;
        } else if (--i < 1)
            i = n;
        if (ProblemType == STTSP) {
            b = Tour[i];
            for (k = 0; k < NodeSet[a].PathLength[b]; k++)
                fprintf(TourFile, "%d\n", NodeSet[a].Path[b][k]);
        }
    }
    fprintf(TourFile, "-1\nEOF\n");
    fclose(TourFile);
    free(FullFileName);
    if (TraceLevel >= 1)
        printff("done\n");
}

/*
 * The FullName function returns a copy of the string Name where all 
 * occurrences of the character '$' have been replaced by Cost.        
 */

char *FullName(char *Name, GainType Cost)
{
    char *NewName = 0, *CostBuffer, *Pos;

    if (!(Pos = strstr(Name, "$"))) {
        NewName = (char *) calloc(strlen(Name) + 1, 1);
        strcpy(NewName, Name);
        return NewName;
    }
    CostBuffer = (char *) malloc(400);
    if (CurrentPenalty != 0)
        sprintf(CostBuffer, GainFormat "_" GainFormat,
                CurrentPenalty, Cost);
    else
        sprintf(CostBuffer, GainFormat, Cost);
    do {
        free(NewName);
        NewName = (char *) calloc(strlen(Name) + strlen(CostBuffer) + 1, 1);
        strncpy(NewName, Name, Pos - Name);
        strcat(NewName, CostBuffer);
        strcat(NewName, Pos + 1);
        Name = NewName;
    }
    while ((Pos = strstr(Name, "$")));
    free(CostBuffer);
    return NewName;
}
