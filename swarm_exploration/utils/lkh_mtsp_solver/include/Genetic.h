#ifndef _GENETIC_H
#define _GENETIC_H

/*
 * This header specifies the interface for the genetic algorithm part of LKH.
 */

#define SmallerFitness(Penalty, Cost, i)\
    (((Penalty) < PenaltyFitness[i]) ||\
     ((Penalty) == PenaltyFitness[i] && (Cost) < Fitness[i]))

#define LargerFitness(Penalty, Cost, i)\
    (((Penalty) > PenaltyFitness[i]) ||\
     ((Penalty) == PenaltyFitness[i] && (Cost) > Fitness[i]))

typedef void (*CrossoverFunction) ();

int MaxPopulationSize; /* The maximum size of the population */ 
int PopulationSize;    /* The current size of the population */

CrossoverFunction Crossover;

int **Population;      /* Array of individuals (solution tours) */
GainType *PenaltyFitness;  /* The f itnesslty  (tour penalty) of each
i                             individual */
GainType *Fitness;     /* The fitness (tour cost) of each individual */

void AddToPopulation(GainType Penalty, GainType Cost);
void ApplyCrossover(int i, int j);
void FreePopulation();
int HasFitness(GainType Penalty, GainType Cost);
int LinearSelection(int Size, double Bias);
GainType MergeTourWithIndividual(int i);
void PrintPopulation();
void ReplaceIndividualWithTour(int i, GainType Penalty, GainType Cost);
int ReplacementIndividual(GainType Penalty, GainType Cost);

void ERXT();

#endif
