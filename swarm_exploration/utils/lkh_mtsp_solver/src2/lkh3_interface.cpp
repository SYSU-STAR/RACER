#include <lkh_mtsp_solver/lkh3_interface.h>

/*
 * This file contains the main function of the program.
 */

int solveMTSPWithLKH3(const char* input_file) {
  GainType Cost, OldOptimum;
  double Time, LastTime = GetTime();
  Node* N;
  int i;

  /* Read the specification of the problem */
  // if (argc >= 2)
  //     ParameterFileName = argv[1];
  ParameterFileName = const_cast<char*>(input_file);
  ReadParameters();
  MaxMatrixDimension = 20000;
  MergeWithTour = Recombination == IPT ? MergeWithTourIPT : MergeWithTourGPX2;
  ReadProblem();
  if (SubproblemSize > 0) {
    if (DelaunayPartitioning)
      SolveDelaunaySubproblems();
    else if (KarpPartitioning)
      SolveKarpSubproblems();
    else if (KCenterPartitioning)
      SolveKCenterSubproblems();
    else if (KMeansPartitioning)
      SolveKMeansSubproblems();
    else if (RohePartitioning)
      SolveRoheSubproblems();
    else if (MoorePartitioning || SierpinskiPartitioning)
      SolveSFCSubproblems();
    else
      SolveTourSegmentSubproblems();
    return EXIT_SUCCESS;
  }
  AllocateStructures();
  if (ProblemType == TSPTW) TSPTW_Reduce();
  if (ProblemType == VRPB || ProblemType == VRPBTW) VRPB_Reduce();
  if (ProblemType == PDPTW) PDPTW_Reduce();
  CreateCandidateSet();
  InitializeStatistics();

  if (Norm != 0 || Penalty) {
    Norm = 9999;
    BestCost = PLUS_INFINITY;
    BestPenalty = CurrentPenalty = PLUS_INFINITY;
  } else {
    /* The ascent has solved the problem! */
    Optimum = BestCost = (GainType)LowerBound;
    UpdateStatistics(Optimum, GetTime() - LastTime);
    RecordBetterTour();
    RecordBestTour();
    CurrentPenalty = PLUS_INFINITY;
    BestPenalty = CurrentPenalty = Penalty ? Penalty() : 0;
    WriteTour(OutputTourFileName, BestTour, BestCost);
    WriteTour(TourFileName, BestTour, BestCost);
    Runs = 0;
  }

  /* Find a specified number (Runs) of local optima */

  for (Run = 1; Run <= Runs; Run++) {
    LastTime = GetTime();
    Cost = FindTour(); /* using the Lin-Kernighan heuristic */
    if (MaxPopulationSize > 1 && !TSPTW_Makespan) {
      /* Genetic algorithm */
      int i;
      for (i = 0; i < PopulationSize; i++) {
        GainType OldPenalty = CurrentPenalty;
        GainType OldCost = Cost;
        Cost = MergeTourWithIndividual(i);
        if (TraceLevel >= 1 &&
            (CurrentPenalty < OldPenalty || (CurrentPenalty == OldPenalty && Cost < OldCost))) {
          if (CurrentPenalty)
            printff("  Merged with %d: Cost = " GainFormat, i + 1, Cost);
          else
            printff(
                "  Merged with %d: Cost = " GainFormat "_" GainFormat, i + 1, CurrentPenalty, Cost);
          if (Optimum != MINUS_INFINITY && Optimum != 0) {
            if (ProblemType != CCVRP && ProblemType != TRP && ProblemType != MLP &&
                MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE)
              printff(", Gap = %0.4f%%", 100.0 * (Cost - Optimum) / Optimum);
            else
              printff(", Gap = %0.4f%%", 100.0 * (CurrentPenalty - Optimum) / Optimum);
          }
          printff("\n");
        }
      }
      if (!HasFitness(CurrentPenalty, Cost)) {
        if (PopulationSize < MaxPopulationSize) {
          AddToPopulation(CurrentPenalty, Cost);
          if (TraceLevel >= 1) PrintPopulation();
        } else if (SmallerFitness(CurrentPenalty, Cost, PopulationSize - 1)) {
          i = ReplacementIndividual(CurrentPenalty, Cost);
          ReplaceIndividualWithTour(i, CurrentPenalty, Cost);
          if (TraceLevel >= 1) PrintPopulation();
        }
      }
    } else if (Run > 1 && !TSPTW_Makespan)
      Cost = MergeTourWithBestTour();
    if (CurrentPenalty < BestPenalty || (CurrentPenalty == BestPenalty && Cost < BestCost)) {
      BestPenalty = CurrentPenalty;
      BestCost = Cost;
      RecordBetterTour();
      RecordBestTour();
      WriteTour(TourFileName, BestTour, BestCost);
    }
    OldOptimum = Optimum;
    if (!Penalty || (MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE)) {
      if (CurrentPenalty == 0 && Cost < Optimum) Optimum = Cost;
    } else if (CurrentPenalty < Optimum)
      Optimum = CurrentPenalty;
    if (Optimum < OldOptimum) {
      printff("*** New OPTIMUM = " GainFormat " ***\n\n", Optimum);
      if (FirstNode->InputSuc) {
        Node* N = FirstNode;
        while ((N = N->InputSuc = N->Suc) != FirstNode)
          ;
      }
    }
    Time = fabs(GetTime() - LastTime);
    UpdateStatistics(Cost, Time);
    if (TraceLevel >= 1 && Cost != PLUS_INFINITY) {
      printff("Run %d: ", Run);
      StatusReport(Cost, LastTime, "");
      printff("\n");
    }
    if (StopAtOptimum && MaxPopulationSize >= 1) {
      if (ProblemType != CCVRP && ProblemType != TRP && ProblemType != MLP &&
                  MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE ?
              CurrentPenalty == 0 && Cost == Optimum :
              CurrentPenalty == Optimum) {
        Runs = Run;
        break;
      }
    }
    if (PopulationSize >= 2 &&
        (PopulationSize == MaxPopulationSize || Run >= 2 * MaxPopulationSize) && Run < Runs) {
      Node* N;
      int Parent1, Parent2;
      Parent1 = LinearSelection(PopulationSize, 1.25);
      do
        Parent2 = LinearSelection(PopulationSize, 1.25);
      while (Parent2 == Parent1);
      ApplyCrossover(Parent1, Parent2);
      N = FirstNode;
      do {
        if (ProblemType != HCP && ProblemType != HPP) {
          int d = C(N, N->Suc);
          AddCandidate(N, N->Suc, d, INT_MAX);
          AddCandidate(N->Suc, N, d, INT_MAX);
        }
        N = N->InitialSuc = N->Suc;
      } while (N != FirstNode);
    }
    SRandom(++Seed);
  }
  PrintStatistics();
  if (Salesmen > 1) {
    if (Dimension == DimensionSaved) {
      for (i = 1; i <= Dimension; i++) {
        N = &NodeSet[BestTour[i - 1]];
        (N->Suc = &NodeSet[BestTour[i]])->Pred = N;
      }
    } else {
      for (i = 1; i <= DimensionSaved; i++) {
        Node* N1 = &NodeSet[BestTour[i - 1]];
        Node* N2 = &NodeSet[BestTour[i]];
        Node* M1 = &NodeSet[N1->Id + DimensionSaved];
        Node* M2 = &NodeSet[N2->Id + DimensionSaved];
        (M1->Suc = N1)->Pred = M1;
        (N1->Suc = M2)->Pred = N1;
        (M2->Suc = N2)->Pred = M2;
      }
    }
    CurrentPenalty = BestPenalty;
    MTSP_Report(BestPenalty, BestCost);
    MTSP_WriteSolution(MTSPSolutionFileName, BestPenalty, BestCost);
    SINTEF_WriteSolution(SINTEFSolutionFileName, BestCost);
  }
  if (ProblemType == ACVRP || ProblemType == BWTSP || ProblemType == CCVRP || ProblemType == CTSP ||
      ProblemType == CVRP || ProblemType == CVRPTW || ProblemType == MLP ||
      ProblemType == M_PDTSP || ProblemType == M1_PDTSP || MTSPObjective != -1 ||
      ProblemType == ONE_PDTSP || ProblemType == OVRP || ProblemType == PDTSP ||
      ProblemType == PDTSPL || ProblemType == PDPTW || ProblemType == RCTVRP ||
      ProblemType == RCTVRPTW || ProblemType == SOP || ProblemType == TRP || ProblemType == TSPTW ||
      ProblemType == VRPB || ProblemType == VRPBTW || ProblemType == VRPPD) {
    printff("Best %s solution:\n", Type);
    CurrentPenalty = BestPenalty;
    SOP_Report(BestCost);
  }
  printff("\n");
  return EXIT_SUCCESS;
}
