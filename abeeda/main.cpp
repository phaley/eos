/*
 * main.cpp
 *
 * This file is part of the Evolution of Swarming project.
 *
 * Copyright 2012 Randal S. Olson, Arend Hintze.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <sstream>
#include <string.h>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <sstream>

#include "globalConst.h"
#include "tHMM.h"
#include "tAgent.h"
#include "tGame.h"

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */

#include "helper.h"           /*  our own helper functions  */

#define ECHO_PORT          (2002)
#define MAX_LINE           (100000)

#define BUFLEN              512
#define NPACK               10
#define PORT                9930

int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
short int port;                  /*  port number               */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */

void    setupBroadcast(void);
void    doBroadcast(string data);
string  findBestRun(tAgent *swarmAgent);

string videoFileName = "many-eyes-default.txt";
ofstream outputFile;

using namespace std;

//double  replacementRate             = 0.1;
double  perSiteMutationRate         = 0.005;
double  groupSizeMutationRate       = 0.01;
int     populationSize              = 100;
int     groupSize                   = 50;
int     groupSizeRange              = 10;
int     totalGenerations            = 2000;
tGame   *game                       = NULL;

bool    evolveGroupSize             = false;
bool    homogeneous                 = false;
bool    zeroOutDeadPrey             = false;
bool    relativeAttackRate          = false;
bool    penalizeGrouping            = false;
int     groupMode                   = 0;

bool    track_best_brains           = false;
int     track_best_brains_frequency = 25;
bool    display_directory           = false;
bool    make_logic_table            = false;
bool    make_dot_pred               = false;
bool    make_dot_swarm              = false;
int     killDelay                   = 10;
int     attackRate                  = 50;
double  confusionMultiplier         = 1.0;
double  vigilanceFoodPenalty        = 1.0;
double  groupingPenalty             = 1.0;
double  foragingFood                = 1.0;

int main(int argc, char *argv[])
{
  vector<tAgent*> swarmAgents, SANextGen;
  tAgent *swarmAgent = NULL, *bestSwarmAgent = NULL;
  double swarmMaxFitness = 0.0;
  string LODFileName = "", swarmGenomeFileName = "", inputGenomeFileName = "";
  string swarmDotFileName = "", logicTableFileName = "";
  int displayDirectoryArgvIndex = 0;
  deque<double> genAvgFitness,genFitnessVar,genAvgVigilance,genVigilanceVar,genAvgGrouped,genGroupedVar,genAvgVigilantGrouped,genVigilantGroupedVar;

  // time-based seed by default. can change with command-line parameter.
  srand((unsigned int)time(NULL));

  for (int i = 0; i < argc; ++i)
    {
      cout << argv[i];
      cout << " ";
    }

  cout << endl;

  for (int i = 1; i < argc; ++i)
    {
      // -dd [directory]: display all genome files in a given directory
      if (strcmp(argv[i], "-dd") == 0 && (i + 1) < argc)
        {
	  ++i;
	  displayDirectoryArgvIndex = i;
          
	  display_directory = true;
        }
        
      // -e [out file name] [out file name]: evolve
      else if (strcmp(argv[i], "-e") == 0 && (i + 2) < argc)
        {
	  ++i;
	  stringstream lodfn;
	  lodfn << argv[i];
	  LODFileName = lodfn.str();
          
	  ++i;
	  stringstream sgfn;
	  sgfn << argv[i];
	  swarmGenomeFileName = sgfn.str();
        }
      
      // -s [int]: set seed
      else if (strcmp(argv[i], "-s") == 0 && (i + 1) < argc)
        {
	  ++i;
	  srand(atoi(argv[i]));
        }
        
      // -g [int]: set generations
      else if (strcmp(argv[i], "-g") == 0 && (i + 1) < argc)
	{
	  ++i;
          
	  // add 2 generations because we look at ancestor->ancestor as best agent at end of sim
	  totalGenerations = atoi(argv[i]) + 2;
          
	  if (totalGenerations < 3)
            {
	      cerr << "minimum number of generations permitted is 3." << endl;
	      exit(0);
            }
        }
      
      // -t [int]: track best brains
      else if (strcmp(argv[i], "-t") == 0 && (i + 1) < argc)
        {
	  track_best_brains = true;
	  ++i;
	  track_best_brains_frequency = atoi(argv[i]);
          
	  if (track_best_brains_frequency < 1)
            {
	      cerr << "minimum brain tracking frequency is 1." << endl;
	      exit(0);
            }
        }
      
      // -lt [in file name] [out file name]: create logic table for given genome
      else if (strcmp(argv[i], "-lt") == 0 && (i + 2) < argc)
        {
	  ++i;
	  swarmAgent->loadAgent(argv[i]);
	  swarmAgent->setupPhenotype();
	  ++i;
	  stringstream ltfn;
	  ltfn << argv[i];
	  logicTableFileName = ltfn.str();
	  make_logic_table = true;
        }
      
      // -dfs [in file name] [out file name]: create dot image file for given swarm genome
      else if (strcmp(argv[i], "-dfs") == 0 && (i + 2) < argc)
        {
	  ++i;
	  swarmAgent->loadAgent(argv[i]);
	  swarmAgent->setupPhenotype();
	  ++i;
	  stringstream dfn;
	  dfn << argv[i];
	  swarmDotFileName = dfn.str();
	  make_dot_swarm = true;
        }
      
      // -kd [int]: set predator kill attempt delay (default: 10)
      else if (strcmp(argv[i], "-kd") == 0 && (i + 1) < argc)
        {
	  ++i;
          
	  killDelay = atoi(argv[i]);
        }
      
      // -cm [float]: set predator confusion multiplier (default: 1.0)
      else if (strcmp(argv[i], "-cm") == 0 && (i + 1) < argc)
        {
	  ++i;
          
	  confusionMultiplier = atof(argv[i]);
        }
     
      // -vfp [float]: set prey penalty to food when vigilant (default: 1.0, range: [0,1])
      else if (strcmp(argv[i], "-vfp") == 0 && (i + 1) < argc)
	{
	  ++i;
	  
	  vigilanceFoodPenalty = atof(argv[i]);
	}
      // -pop [float]: set population size for the GA
      else if (strcmp(argv[i], "-pop") == 0 && (i + 1) < argc)
	{
	  ++i;
	  
	  populationSize = atof(argv[i]);
	}
      // -gsz [float]: set group size for the evaluation
      else if (strcmp(argv[i], "-gsz") == 0 && (i + 1) < argc)
	{
	  ++i;
	  
	  groupSize = atof(argv[i]);
	}
      // -cln: set evaluation to work on clones of genotypes
      else if (strcmp(argv[i], "-cln") == 0)
	{
	  homogeneous = true;
	}
      // -zdp: use to zero out the fitness of prey that die
      else if (strcmp(argv[i], "-zdp") == 0)
	{
	  zeroOutDeadPrey = true;
	}
      // -aar [float]: set absolute attack rate for the evaluation
      else if (strcmp(argv[i], "-aar") == 0 && (i + 1) < argc)
	{
	  ++i;
	  
	  attackRate = atof(argv[i]);
	}
      // -rar [float]: set attacks per individual for the evaluation
      else if (strcmp(argv[i], "-rar") == 0 && (i + 1) < argc)
	{
	  ++i;
	  
	  relativeAttackRate = true;
	  attackRate = atof(argv[i]);
	}
      // -grp [float]: set grouping mode for the evaluation
      // 0 is forced group, 1 is forced ungrouping, 2 is choose to group
      else if (strcmp(argv[i], "-grp") == 0 && (i + 1) < argc)
	{
	  ++i;
	  
	  groupMode = atof(argv[i]);
	}
      // -gp [float]: penalize grouping relative to group size (default: 1.0, range: [0,1])
      else if (strcmp(argv[i], "-gp") == 0 && (i + 1) < argc)
	{
	  ++i;

	  penalizeGrouping = true;
	  groupingPenalty = atof(argv[i]);
	}
      // -ff [float]: set the amount of food a non-vigilant prey receives
      else if (strcmp(argv[i], "-ff") == 0 && (i + 1) < argc)
	{
	  ++i;

	  foragingFood = atof(argv[i]);
	}
      // -egs: set program to evolve group sizes in addition to agents
      else if (strcmp(argv[i], "-egs") == 0)
	{
	  evolveGroupSize = true;
	}
    }
  
  // initial object setup
  swarmAgents.resize(populationSize);
  game = new tGame;
  swarmAgent = new tAgent;
  swarmAgent->setupRandomAgent(5000);

  // setup is drastically different depending on 
  if(evolveGroupSize)
    {
      FILE *LOD = fopen(LODFileName.c_str(), "w");
      
      fprintf(LOD, "generation,dom_group_fitness,dom_group_size,dom_group_vigilance,avg_group_fitness,avg_group_size,avg_group_vigilance\n");
      
      int* groupSizes = new int[populationSize];
      int* GSnextGen = new int[populationSize];
      double* groupFitnesses = new double[populationSize];
      double* maxAgentFitnesses = new double[populationSize];
      // set up the initial sizes of each group
      vector<tAgent*>* groupAgents = new vector<tAgent*>[populationSize];
      vector<tAgent*>* GAnextGen = new vector<tAgent*>[populationSize];
      for(int i = 0; i < populationSize; ++i)
	{
	  groupSizes[i] = (int) ((randDouble * 20) + 1);
	  // set up the initial agents for this group
	  groupAgents[i].resize(groupSizes[i]);
	  groupAgents[i][0] = new tAgent;
	  groupAgents[i][0]->inherit(swarmAgent, perSiteMutationRate, 1);
	  for(int j = 1; j < groupSizes[i]; ++j)
	    {
	      groupAgents[i][j] = new tAgent;
	      // homogeneous populations are identical
	      if(homogeneous)
		{
		  groupAgents[i][j]->inherit(groupAgents[i][0], 0.0, 1);
		}
	      // heterogeneous populations can have variation at setup
	      else
		{
		  groupAgents[i][j]->inherit(swarmAgent, perSiteMutationRate, 1);
		}
	    }
	}
      
      cout << "setup complete" << endl;
      cout << "starting evolution" << endl;

      double maxGroupFitness, domGroupSize, domVigilance, avgFitness, avgGroupSize, avgVigilance, grpVigilance, agtVigilance;

      // main loop
      for(int update = 1; update <= totalGenerations; ++update)
	{
	  maxGroupFitness = domGroupSize = domVigilance = avgFitness = avgGroupSize = avgVigilance = grpVigilance = agtVigilance = 0.0;
	  for(int i = 0; i < populationSize; ++i)
	    {
	      // reset each member of the population before each update
	      maxAgentFitnesses[i] = 0.0;
	      groupFitnesses[i] = 0.0;
	      grpVigilance = 0.0;
	      for(int j = 0; j < groupSizes[i]; ++j)
		{
		  groupAgents[i][j]->fitness = 0.0;
		}
	      // evaluate the fitness of the group
	      game->executeGame(groupAgents[i], groupSizes[i], NULL, false, confusionMultiplier, vigilanceFoodPenalty, zeroOutDeadPrey,
				groupMode, relativeAttackRate, attackRate, penalizeGrouping, groupingPenalty, foragingFood);
	      // fitness and vigilance values are evaluated across the agents in the group
	      for(int j = 0; j < groupSizes[i]; ++j)
		{
		  agtVigilance = 0.0;
		  groupFitnesses[i] += groupAgents[i][j]->fitness;
		  if(maxAgentFitnesses[i] < groupAgents[i][j]->fitness)
		    {
		      maxAgentFitnesses[i] = groupAgents[i][j]->fitness;
		    }
		  tAgent* testSubject = new tAgent;
		  testSubject->inherit(groupAgents[i][j], 0.0, update);
		  testSubject->setupPhenotype();
		  for(int k = 0; k < 1000; ++k)
		    {
		      testSubject->updateStates();
		      if((testSubject->states[0] & 1) == 1)
			{
			  agtVigilance++;
			}
		    }
		  testSubject->retire();
		  testSubject->nrPointingAtMe--;
		  delete testSubject;
		  agtVigilance /= 1000;
		  //cout << "AGENT VIGILANCE:" << endl;
		  //cout << agtVigilance << endl;
		  grpVigilance += agtVigilance;
		  //cout << "GROWING GROUP VIGILANCE:" << endl;
		  //cout << grpVigilance << endl;
		}
	      groupFitnesses[i] /= groupSizes[i];
	      avgFitness += groupFitnesses[i];
	      grpVigilance /= groupSizes[i];
	      //cout << "GROUP VIGILANCE:" << endl;
	      //cout << grpVigilance << endl;
	      avgVigilance += grpVigilance;
	      //cout << "GROWING AVERAGE VIGILANCE:" << endl;
	      //cout << avgVigilance << endl;
	      avgGroupSize += groupSizes[i];
	      // keep track of the group with the highest fitness
	      if(maxGroupFitness < groupFitnesses[i])
		{
		  maxGroupFitness = groupFitnesses[i];
		  domGroupSize = groupSizes[i];
		  domVigilance = grpVigilance;
		}
	    }
	  avgFitness /= populationSize;
	  avgGroupSize /= populationSize;
	  avgVigilance /= populationSize;
	  //cout << "***THE AVERAGE VIGILANCE***" << endl;
	  //cout << avgVigilance << endl;
	  // print the experimental variables to an output file
	  fprintf(LOD, "%d,%f,%f,%f,%f,%f,%f\n", update, maxGroupFitness, domGroupSize, domVigilance, avgFitness, avgGroupSize, avgVigilance);
	  // construct group sizes for the next update
	  for(int i = 0; i < populationSize; ++i)
	    {
	      int j = 0;
	      // selection for which group will reproduce is fitness-proportional
	      do
		{
		  j = rand() % populationSize;
		} while((j == i) || (randDouble  > (groupFitnesses[j] / maxGroupFitness)));
	      int newGroupSize = groupSizes[j];
	      // group size can mutate according to a normal distribution with the current value as the mean
	      if(randDouble < groupSizeMutationRate)
		{
		  do {
		    int sign = randDouble > 0.5 ? 1 : -1;
		    newGroupSize = (((int) sqrt(-2 * log(randDouble)) * cos(2 * cPI * randDouble)) * groupSizeRange * sign) + groupSizes[j];
		  } while(newGroupSize <= 0);
		}
	      GSnextGen[i] = newGroupSize;
	      GAnextGen[i].resize(GSnextGen[i]);
	      // if groups are homogeneous, construct groups based on a single agent
	      if(homogeneous)
		{
		  tAgent* offspring = new tAgent;
		  int l = 0;
		  // although all agents should be identical, we go ahead and use fitness-proportional selection
		  do
		    {
		      l = rand() % groupSizes[j];
		    } while(randDouble > (groupAgents[j][l]->fitness / maxAgentFitnesses[j]));
		  offspring->inherit(groupAgents[j][l], perSiteMutationRate, update);
		  GAnextGen[i][0] = offspring;
		  // fill the rest of the group with clones
		  for(int k = 1; k < GSnextGen[i]; ++k)
		    {
		      tAgent* clone = new tAgent;
		      clone->inherit(offspring, 0.0, update);
		      GAnextGen[i][k] = clone;
		    }
		}
	      // if groups are heterogeneous, agents are selected to pass on to the next generation based on fitness
	      else
		{
		  for(int k = 0; k < GSnextGen[i]; ++k)
		    {
		      tAgent* offspring = new tAgent;
		      int l = 0;
		      do
			{
			  l = rand() % groupSizes[j];
			} while(randDouble > (groupAgents[j][l]->fitness / maxAgentFitnesses[j]));
		      offspring->inherit(groupAgents[j][l], perSiteMutationRate, update);
		      GAnextGen[i][k] = offspring;
		    }
		}
	    }
	  // delete any old agents to save on memory
	  for(int i = 0; i < populationSize; ++i)
	    {
	      for(int j = 0; j < groupSizes[i]; ++j)
		{
		  groupAgents[i][j]->retire();
		  groupAgents[i][j]->nrPointingAtMe--;
		  if(groupAgents[i][j]->nrPointingAtMe == 0)
		    {
		      delete groupAgents[i][j];
		    }
		}
	      groupSizes[i] = GSnextGen[i];
	      groupAgents[i].resize(groupSizes[i]);
	      // copy over the agents to the next generation
	      for(int j = 0; j < groupSizes[i]; ++j)
		{
		  groupAgents[i][j] = GAnextGen[i][j];
		}
	    }
	  cout << "generation " << update << ": swarm [" << (int)avgFitness << " : " << (int)maxGroupFitness << "]" << endl;
	}
      delete [] groupFitnesses;
      delete [] maxAgentFitnesses;
      for(int i = 0; i < populationSize; ++i)
	{
	  for(int j = 0; j < GAnextGen[i].size(); ++j)
	    {
	      GAnextGen[i][j]->retire();
	      GAnextGen[i][j]->nrPointingAtMe--;
	      if(GAnextGen[i][j]->nrPointingAtMe == 0)
		{
		  delete GAnextGen[i][j];
		}
	    }
	}
      delete [] groupSizes;
      delete [] GSnextGen;
      delete [] groupAgents;
      delete [] GAnextGen;
      fclose(LOD);
    }
  // if we are not evolving the size of the group...
  else
    {
      // make mutated copies of the start genome to fill up the initial population
      for(int i = 0; i < populationSize; ++i)
	{
	  swarmAgents[i] = new tAgent;
	  swarmAgents[i]->inherit(swarmAgent, 0.01, 1);
	}
      
      SANextGen.resize(populationSize);
      
      cout << "setup complete" << endl;
      cout << "starting evolution" << endl;
      
      // main loop
      for (int update = 1; update <= totalGenerations; ++update)
	{
	  // reset fitnesses
	  for(int i = 0; i < populationSize; ++i)
	    {
		 swarmAgents[i]->fitness = 0.0;
	    }
	  
	  // determine fitness of population
	  swarmMaxFitness = 0.0;
	  double swarmAvgFitness = 0.0;
	  double swarmAvgVigilance = 0.0;
	  double swarmAvgGrouped = 0.0;
	  double swarmAvgVigilantGrouped = 0.0;
	  double swarmFitnessVar = 0.0;
	  double swarmVigilanceVar = 0.0;
	  double swarmGroupedVar = 0.0;
	  double swarmVigilantGroupedVar = 0.0;
	  double fitnesses[populationSize];
	  double vigilances[populationSize];
	  double groupings[populationSize];
	  double vigilanceGroupings[populationSize];
	  
	  // if groups are homogeneous, then each genotype is evaluated individually
	  if(homogeneous)
	    {
	      for(int i = 0; i < populationSize; ++i)
		{
		  vector<tAgent*> gameGroup(groupSize);
		  // copies of the genotype are made to fill the simulation
		  for(int j = 0; j < groupSize; ++j)
		    {
		      gameGroup[j] = new tAgent;
		      gameGroup[j]->inherit(swarmAgents[i], 0.0, 0);
		    }
		  game->executeGame(gameGroup, groupSize, NULL, false, confusionMultiplier, vigilanceFoodPenalty, zeroOutDeadPrey,
				    groupMode, relativeAttackRate, attackRate, penalizeGrouping, groupingPenalty, foragingFood);
		  // the fitness of a genotype is the average of all the agents representing it in the simulation
		  for(int j = 0; j < groupSize; ++j)
		    {
		      swarmAgents[i]->fitness += gameGroup[j]->fitness;
		      delete gameGroup[j];
		    }
		  swarmAgents[i]->fitness = swarmAgents[i]->fitness / groupSize;
		  gameGroup.clear();
		}
	    }
	  // if groups are heterogeneous, then we can assign fitnesses to multiple genotypes at once
	  else
	    {
	      int startAgent = 0;
	      // we want to assign fitnesses to all members of the population
	      while(startAgent < populationSize)
		{
		  // the first and last iterators represent the array of individuals to be placed in the simulation
		  vector<tAgent*>::const_iterator first = swarmAgents.begin() + startAgent;
		  vector<tAgent*>::const_iterator last = swarmAgents.begin() + startAgent + groupSize;
		  vector<tAgent*> gameGroup(first, last);
		  game->executeGame(gameGroup, groupSize, NULL, false, confusionMultiplier, vigilanceFoodPenalty, zeroOutDeadPrey,
				    groupMode, relativeAttackRate, attackRate, penalizeGrouping, groupingPenalty, foragingFood);
		  startAgent += groupSize;
		  gameGroup.clear();
		}
	    }
	  //many of our behaviors are averaged across all the members of the population
	  for(int i = 0; i < populationSize; ++i)
	    {
	      fitnesses[i] = swarmAgents[i]->fitness;
	      swarmAvgFitness += swarmAgents[i]->fitness;
	      
	      swarmAgents[i]->setupPhenotype();
	      double agentAvgVigilance = 0;
	      double agentAvgGrouped = 0;
	      double agentAvgVigilantGrouped = 0;
	      for(int j = 0; j < 1000; ++j)
		{
		  swarmAgents[i]->updateStates();
		  bool vigilant = (swarmAgents[i]->states[0] & 1) == 1;
		  bool grouped = (swarmAgents[i]->states[1] & 1) == 1;
		  if(vigilant)
		    agentAvgVigilance++;
		  if(grouped)
		    agentAvgGrouped++;
		  if(vigilant && grouped)
		    agentAvgVigilantGrouped++;
		}
	      agentAvgVigilance /= 1000;
	      vigilances[i] = agentAvgVigilance;
	      swarmAvgVigilance += agentAvgVigilance;
	      agentAvgGrouped /= 1000;
	      groupings[i] = agentAvgGrouped;
	      swarmAvgGrouped += agentAvgGrouped;
	      agentAvgVigilantGrouped /= 1000;
	      vigilanceGroupings[i] = agentAvgVigilantGrouped;
	      swarmAvgVigilantGrouped += agentAvgVigilantGrouped;
	      // we keep track of the individual with the highest fitness
	      if(swarmAgents[i]->fitness > swarmMaxFitness)
		{
		  swarmMaxFitness = swarmAgents[i]->fitness;
		  bestSwarmAgent = swarmAgents[i];
		}
	    }
	  
	  swarmAvgFitness /= (double)populationSize;
	  genAvgFitness.push_back(swarmAvgFitness);
	  swarmAvgVigilance /= (double)populationSize;
	  genAvgVigilance.push_back(swarmAvgVigilance);
	  swarmAvgGrouped /= (double)populationSize;
	  genAvgGrouped.push_back(swarmAvgGrouped);
	  swarmAvgVigilantGrouped /= (double)populationSize;
	  genAvgVigilantGrouped.push_back(swarmAvgVigilantGrouped);

	  // we calculate variances for each of the metrics
	  for(int i = 0; i < populationSize; ++i)
	    {
	      swarmFitnessVar += pow(fitnesses[i] - swarmAvgFitness, 2);
	      swarmVigilanceVar += pow(vigilances[i] - swarmAvgVigilance, 2);
	      swarmGroupedVar += pow(groupings[i] - swarmAvgGrouped, 2);
	      swarmVigilantGroupedVar += pow(vigilanceGroupings[i] - swarmAvgVigilantGrouped, 2);
	    }

	  swarmFitnessVar /= (double)populationSize;
	  genFitnessVar.push_back(swarmFitnessVar);
	  swarmVigilanceVar /= (double)populationSize;
	  genVigilanceVar.push_back(swarmVigilanceVar);
	  swarmGroupedVar /= (double)populationSize;
	  genGroupedVar.push_back(swarmGroupedVar);
	  swarmVigilantGroupedVar /= (double)populationSize;
	  genVigilantGroupedVar.push_back(swarmVigilantGroupedVar);
	  
	  cout << "generation " << update << ": swarm [" << (int)swarmAvgFitness << " : " << (int)swarmMaxFitness << "]" << endl;
	  
	  // we select individuals to reproduce into the next generation
	  for(int i = 0; i < populationSize; ++i)
	    {
	      tAgent *offspring = new tAgent;
	      int j = 0;
	      // selection for reproduction is fitness-proportionate
	      do
		{
		  j = rand() % populationSize;
		} while((j == i) || (randDouble > (swarmAgents[j]->fitness / swarmMaxFitness)));
	      // there is a chance of mutation upon reproduction
	      offspring->inherit(swarmAgents[j], perSiteMutationRate, update);
	      SANextGen[i] = offspring;
	    }
	  
	  // we mix the population to ensure heterogeneous groups have different genotypes between updates
	  random_shuffle(SANextGen.begin(), SANextGen.end());
	  
	  for(int i = 0; i < populationSize; ++i)
	    {
	      // retire and replace the swarm agents from the previous generation
	      swarmAgents[i]->retire();
	      swarmAgents[i]->nrPointingAtMe--;
	      if(swarmAgents[i]->nrPointingAtMe == 0)
		{
		  delete swarmAgents[i];
		}
	      swarmAgents[i] = SANextGen[i];
	    }
	  
	  swarmAgents = SANextGen;
	  
	  if (track_best_brains && update % track_best_brains_frequency == 0)
	    {
	      stringstream sss, pss;
	      
	      sss << "swarm" << update << ".genome";
	      
	      swarmAgents[0]->ancestor->ancestor->saveGenome(sss.str().c_str());
	    }
	}
      
      // save the genome file of the lmrca
      swarmAgents[0]->ancestor->ancestor->saveGenome(swarmGenomeFileName.c_str());
      
      // save video and quantitative stats on the best swarm agent's LOD
      vector<tAgent*> saveLOD;
      
      cout << "building ancestor list" << endl;
      
      // use 2 ancestors down from current population because that ancestor is highly likely to have high fitness
      tAgent* curAncestor = swarmAgents[0]->ancestor->ancestor;
      
      while (curAncestor != NULL)
	{
	  // don't add the base ancestor
	  if (curAncestor->ancestor != NULL)
	    {
	      saveLOD.insert(saveLOD.begin(), curAncestor);
	    }
	  
	  curAncestor = curAncestor->ancestor;
	}
      
      FILE *LOD = fopen(LODFileName.c_str(), "w");
      
      //fprintf(LOD, "generation,prey_fitness,num_alive_end,num_prey_vigilant\n");
      fprintf(LOD, "generation,line_of_descent_time_vigilant,average_time_vigilant,time_vigilant_variance,average_fitness,fitness_variance,average_time_grouped,time_grouped_variance,average_time_vigilant_and_grouped,time_vigilant_and_grouped_variance\n");
      
      cout << "analyzing ancestor list" << endl;
      
      // we want to look at the ancestors of our most fit organism
      for (vector<tAgent*>::iterator it = saveLOD.begin(); it != saveLOD.end(); ++it)
	{
	  (*it)->setupPhenotype();
	  double timeVigilant = 0;
	  // measure vigilance behavior along the line-of-descent
	  for(int i = 0; i < 1000; ++i)
	       {
		 (*it)->updateStates();
		 if(((*it)->states[0] & 1) == 1)
		   timeVigilant++;
	       }
	  timeVigilant = timeVigilant / 1000;
	  fprintf(LOD, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", (*it)->born, timeVigilant, genAvgVigilance.front(), genVigilanceVar.front(),
		  genAvgFitness.front(), genFitnessVar.front(), genAvgGrouped.front(), genGroupedVar.front(), genAvgVigilantGrouped.front(),
		  genVigilantGroupedVar.front());
	  genAvgFitness.pop_front();
	  genFitnessVar.pop_front();
	  genAvgVigilance.pop_front();
	  genVigilanceVar.pop_front();
	  genAvgGrouped.pop_front();
	  genGroupedVar.pop_front();
	  genAvgVigilantGrouped.pop_front();
	  genVigilantGroupedVar.pop_front();
	  // collect quantitative stats
	  //game->executeGame(*it, LOD, false, killDelay, confusionMultiplier, vigilanceFoodPenalty);
	}
      
      cout << "finished analyzing ancestor list" << endl;
      
      fclose(LOD);
    }
  delete game;
  delete swarmAgent;
  return 0;
}

string findBestRun(tAgent *swarmAgent)
{
    string reportString = "", bestString = "";
    double bestFitness = 0.0;
    
    for (int rep = 0; rep < 100; ++rep)
    {
      //reportString = game->executeGame(swarmAgent, NULL, true, killDelay, confusionMultiplier, vigilanceFoodPenalty);
        
        if (swarmAgent->fitness > bestFitness)
        {
            bestString = reportString;
            bestFitness = swarmAgent->fitness;
        }
    }
    
    return bestString;
}
