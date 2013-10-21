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
int     populationSize              = 50;
int     totalGenerations            = 2000;
tGame   *game                       = NULL;

bool    track_best_brains           = false;
int     track_best_brains_frequency = 25;
bool    display_directory           = false;
bool    make_logic_table            = false;
bool    make_dot_pred               = false;
bool    make_dot_swarm              = false;
int     killDelay                   = 10;
double  confusionMultiplier         = 1.0;
double  vigilanceFoodPenalty        = 1.0;

int main(int argc, char *argv[])
{
  vector<tAgent*> swarmAgents, SANextGen;
  tAgent *swarmAgent = NULL, *bestSwarmAgent = NULL;
  double swarmMaxFitness = 0.0;
  string LODFileName = "", swarmGenomeFileName = "", inputGenomeFileName = "";
  string swarmDotFileName = "", logicTableFileName = "";
  int displayDirectoryArgvIndex = 0;
  
  // initial object setup
  swarmAgents.resize(populationSize);
  game = new tGame;
  swarmAgent = new tAgent;
    
  // time-based seed by default. can change with command-line parameter.
  srand((unsigned int)time(NULL));
  
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
    }
  
  if (display_directory)
    {
      DIR *dir;
      struct dirent *ent;
      dir = opendir(argv[displayDirectoryArgvIndex]);
      
      // map: run # -> [swarm file name, predator file name]
      map< int, vector<string> > fileNameMap;
      
      if (dir != NULL)
        {
	  cout << "reading in files" << endl;
          
	  // read all of the files in the directory
	  while ((ent = readdir(dir)) != NULL)
            {
	      string dirFile = string(ent->d_name);
              
	      if (dirFile.find(".genome") != string::npos)
                {
		  // find the first character in the file name that is a digit
		  int i = 0;
                  
		  for ( ; i < dirFile.length(); ++i)
                    {
		      if (isdigit(dirFile[i]))
                        {
			  break;
                        }
                    }
		  
		  // get the run number from the file name
		  int runNumber = atoi(dirFile.substr(i).c_str());
                  
		  if (fileNameMap[runNumber].size() == 0)
                    {
		      fileNameMap[runNumber].resize(2);
                    }
		  
		  // map the file name into the appropriate location
		  if (dirFile.find("swarm") != string::npos)
                    {
		      fileNameMap[runNumber][0] = argv[displayDirectoryArgvIndex] + dirFile;
                    }
		  else if (dirFile.find("predator") != string::npos)
                    {
		      fileNameMap[runNumber][1] = argv[displayDirectoryArgvIndex] + dirFile;
                    }
                }
            }
	  
	  closedir(dir);
        }
      else
        {
	  cerr << "invalid directory: " << argv[displayDirectoryArgvIndex] << endl;
	  exit(0);
        }
    }
  
  if (make_logic_table)
    {
        swarmAgent->saveLogicTable(logicTableFileName.c_str());
        exit(0);
    }
    
  if (make_dot_swarm)
    {
        swarmAgent->saveToDot(swarmDotFileName.c_str(), false);
        exit(0);
    }
  
    // seed the agents
    delete swarmAgent;
    swarmAgent = new tAgent;
    swarmAgent->setupRandomAgent(5000);
    //swarmAgent->loadAgent("startSwarm.genome");
    
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
        
	game->executeGame(swarmAgents, NULL, false, confusionMultiplier, vigilanceFoodPenalty);
       
	for(int i = 0; i < populationSize; ++i)
	  {
	    swarmAvgFitness += swarmAgents[i]->fitness;
            
            if(swarmAgents[i]->fitness > swarmMaxFitness)
            {
                swarmMaxFitness = swarmAgents[i]->fitness;
                bestSwarmAgent = swarmAgents[i];
            }
	  }
        
        swarmAvgFitness /= (double)populationSize;
		
	cout << "generation " << update << ": swarm [" << (int)swarmAvgFitness << " : " << (int)swarmMaxFitness << "]" << endl;
	
	for(int i = 0; i < populationSize; ++i)
	  {
	    // construct swarm agent population for the next generation
	    tAgent *offspring = new tAgent;
	    int j = 0;
	    
	    do
	      {
		j = rand() % populationSize;
	      } while((j == i) || (randDouble > (swarmAgents[j]->fitness / swarmMaxFitness)));
	    
	    offspring->inherit(swarmAgents[j], perSiteMutationRate, update);
	    SANextGen[i] = offspring;
	  }
	
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
    fprintf(LOD, "generation, percent_time_vigilant\n");
    
    cout << "analyzing ancestor list" << endl;
    
    for (vector<tAgent*>::iterator it = saveLOD.begin(); it != saveLOD.end(); ++it)
    {
      (*it)->setupPhenotype();
      double timeVigilant = 0;
      for(int i = 0; i < 1000; ++i)
	{
	  (*it)->updateStates();
	  if(((*it)->states[0] & 1) == 1)
	    timeVigilant++;
	}
      timeVigilant = timeVigilant / 1000;
      fprintf(LOD, "%d,%f\n", (*it)->born, timeVigilant);
      // collect quantitative stats
      //game->executeGame(*it, LOD, false, killDelay, confusionMultiplier, vigilanceFoodPenalty);
    }

    cout << "finished analyzing ancestor list" << endl;
    
    fclose(LOD);
    
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
