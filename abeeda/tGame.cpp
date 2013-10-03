/*
 * tGame.cpp
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

#include "tGame.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>

// simulation-specific constants
#define preyVisionRange         100.0 * 100.0
#define preyVisionAngle         360.0 / 2.0
#define predatorVisionRange     200.0 * 200.0
#define preySensors             24
#define predatorSensors         12
#define preyStep                1.75
#define predatorStep            2.00
#define preyTurn                8
#define attackDelay             50
#define totalStepsInSimulation  2000
#define gridX                   256.0
#define gridY                   256.0
#define killDist                3.0 * 3.0
#define forageDist              2.0 * 2.0
#define broadcastDist           100.0 * 100.0
#define boundaryDist            250.0
#define artificialReward        25.0
#define artificialPenalty       25.0

// precalculated lookup tables for the game
double cosLookup[360];
double sinLookup[360];
//double atan2Lookup[(int)gridY * 4][(int)gridX * 4];

tGame::tGame()
{
    // fill lookup tables
    for (int i = 0; i < 360; ++i)
    {
        cosLookup[i] = cos((double)i * (cPI / 180.0));
        sinLookup[i] = sin((double)i * (cPI / 180.0));
    }

    /*for (int i = 0; i < (int)(gridY * 4); ++i)
    {
      for (int j = 0; j < (int)(gridX * 4); ++j)
        {
	  atan2Lookup[i][j] = atan2(i - gridY * 2, j - gridX * 2) * 180.0 / cPI;
	  while(atan2Lookup[i][j] < 0)
	    {
	      atan2Lookup[i][j] += 360;
	    }
        }
    }*/
}

tGame::~tGame() { }

// runs the simulation for the given agent(s)
string tGame::executeGame(tAgent* swarmAgent, tAgent* predatorAgent, FILE *data_file, bool report, double safetyDist, double predatorVisionAngle, int killDelay,
			  double confusionMultiplier, double vigilanceFoodPenalty, double foragingMovePenalty, bool artificialBroadcast, bool artificialResponse)
{
    // state of simulation (whether prey should be feeding or fleeing)
    bool foragePhase = true;
    // number of updates remaining until the phase switches from foraging to fleeing
    int phaseDelay = killDelay;

    // LOD data variables
    double swarmFitness = 0.0;
    double predatorFitness = 0.0;
    // counter of how many swarm agents are still alive
    int numAlive = swarmSize;
    // number of attacks the predator has made
    int numAttacks = 0;
    
    vector<double> bbSizes;
    vector<double> shortestDists;
    vector<double> swarmDensityCounts;
    vector<double> numPreyVigilant;

    // set of all prey brains
    tAgent* swarm[swarmSize];

    // swarm agent x, y, angles
    double preyX[swarmSize], preyY[swarmSize], preyA[swarmSize];
    // swarm alive status
    bool preyDead[swarmSize];
    // swarm hidden status
    bool preyFled[swarmSize];
    // swarm food status
    double stomachs[swarmSize];
    // swarm vigilance status
    bool vigilance[swarmSize];
    // food reward while vigilant
    double vigilanceFood = 1 - vigilanceFoodPenalty;

    for(int i = 0; i < swarmSize; ++i)
      {
	stomachs[i] = 0;
	vigilance[i] = false;
      }
    
    // lookup table for distances between predator and swarm agents
    double predDists[predCount][swarmSize];
    
    // lookup table for distances between swarm agents and other swarm agents
    double preyDists[swarmSize][swarmSize];

    // lookup table for distances between swarm agents and food
    //double foodDists[swarmSize][foodCount];

    /*
    // food x, y
    double foodX[foodCount], foodY[foodCount];
    for(int i = 0; i < foodCount; ++i)
    {

      bool goodPos = true;

      do {

	goodPos = true;

	foodX[i] = .9 * ((double)(randDouble * gridX * 2.0) - gridX);
	foodY[i] = .9 * ((double)(randDouble * gridY * 2.0) - gridY);

	for(int j = 0; j < i; ++j)
	  {
	    if(calcDistanceSquared(foodX[i], foodY[i], foodX[j], foodY[j]) < safetyDist)
	      {
		goodPos = false;
		break;
	      }
	  }
      } while(!goodPos);
    }
    */

    // predator X, Y, and angle
    double predX[predCount], predY[predCount], startX[predCount], startY[predCount], predA[predCount];
    bool predatorHasEaten[predCount], predatorOnGrid[predCount];

    // define the predator's maximum turning radius based on speed and kill distance
    double predatorTurn = floor(2 * asin(predatorStep / (2 * sqrt(killDist))) * 180 / cPI);
    predatorTurn += (predatorTurn < 0 ? 360 : 0);
    predatorTurn = 8;

    // tables of agents to receive broadcast signals
    bool receivedBroadcast[swarmSize];
    bool sentBroadcast[swarmSize];

    // string containing the information to create a video of the simulation
    string reportString = "";
    
    for(int i = 0; i < swarmSize; ++i)
      {
	receivedBroadcast[i] = false;
	sentBroadcast[i] = false;

	//setup brain for swarm of individuals
	swarm[i] = new tAgent();
	swarm[i]->inherit(swarmAgent, 0.0, 0);
	swarm[i]->setupPhenotype();
	swarm[i]->fitness = 0.0;
	
        int numTries = 0;
        bool goodPos = true;
        
        do
	  {
            ++numTries;
            goodPos = true;
            
            preyX[i] = 0.4 * ((double)(randDouble * gridX * 2.0) - gridX);
            preyY[i] = 0.4 * ((double)(randDouble * gridY * 2.0) - gridY);
            
            for (int j = 0; j < i; ++j)
            {
	      if (calcDistanceSquared(preyX[i], preyY[i], preyX[j], preyY[j]) < safetyDist)
                {
		  goodPos = false;
		  break;
                }
            }
	    /*
	    if(goodPos)
	      {
		for(int j = 0; j < foodCount; j++)
		  {
		    if(calcDistanceSquared(preyX[i], preyY[i], foodX[j], foodY[j]) < safetyDist)
		      {
			goodPos = false;
			break;
		      }
		  }
	      }
	    */

	  } while (!goodPos && numTries < 10);
	
        preyA[i] = (int)(randDouble * 359.0);
        
	if(!goodPos)
	  {
	    preyDead[i] = true;
	    preyFled[i] = true;
	  }
	else
	  {
	    preyDead[i] = false;
	    preyFled[i] = false;
	  }
      }
    
    // initialize predator and prey lookup tables
    recalcPredAndPreyAndFoodDistTable(foragePhase, preyX, preyY, preyDead, predX, predY, /*foodX, foodY, */ predDists, preyDists/*, foodDists*/);

    /*       BEGINNING OF SIMULATION LOOP       */
    
    for(int step = 0; step < totalStepsInSimulation; ++step)
      {
        /*       CREATE THE REPORT STRING FOR THE VIDEO       */
        if(report)
        {
            // report X, Y, angle of predator
	  for(int i = 0; i < predCount; i++)
	    {
	      if(!foragePhase)
		{
		  char text[1000];
		  sprintf(text,"%f,%f,%f,%d,%d,%d=", predX[i], predY[i], predA[i], 255, 0, 0);
		  reportString.append(text);
		}            
	    }

            // compute center of swarm
            /*double cX = 0.0, cY = 0.0;
            calcSwarmCenter(preyX,preyY, preyDead, cX, cY);
            
            // report X, Y of center of swarm
            char text2[1000];
            sprintf(text2,"%f,%f,%f,%d,%d,%d=", cX, cY, 0.0, 124, 252, 0);
            reportString.append(text2);*/
            
            // report X, Y, angle of all prey
            for(int i = 0; i <swarmSize; ++i)
            {
	      if (!preyDead[i] && !preyFled[i])
                {
                    char text[1000];
                    
		    if(sentBroadcast[i])
		      {
			sprintf(text,"%f,%f,%f,%d,%d,%d=", preyX[i], preyY[i], preyA[i], 0, 0, 255);
		      }
		    else
		      {
			if(vigilance[i])
			  {
			    sprintf(text,"%f,%f,%f,%d,%d,%d=", preyX[i], preyY[i], preyA[i], 0, 255, 0);
			  }
			else
			  {
			    sprintf(text,"%f,%f,%f,%d,%d,%d=", preyX[i], preyY[i], preyA[i], 255, 255, 255);
			  }
		      }
                    reportString.append(text);
                }
            }
	    /*
	    for(int i = 0; i < foodCount; ++i)
	      {
		char text[1000];
		sprintf(text,"%f,%f,%f,%d,%d,%d=", foodX[i], foodY[i], 0, 0, 255, 0);
		reportString.append(text);
	      }
	    */
            reportString.append("N");
            
        }
        /*       END OF REPORT STRING CREATION       */
        
        
        /*       SAVE DATA FOR THE LOD FILE       */
        if(data_file != NULL)
        {
            // calculate bounding box size for this update
            // lu = Left Uppermost point
            // rb = Right Bottommost point
            double luX = DBL_MAX, luY = DBL_MAX;
            double rbX = -DBL_MAX, rbY = -DBL_MAX;
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    if (preyX[i] < luX)
                    {
                        luX = preyX[i];
                    }
                    
                    if (preyX[i] > rbX)
                    {
                        rbX = preyX[i];
                    }
                    
                    if (preyY[i] < luY)
                    {
                        luY = preyY[i];
                    }
                    
                    if (preyY[i] > rbY)
                    {
                        rbY = preyY[i];
                    }
                }
            }
            
            // area = L x W
            //                L = dist (rbX, rbY) to (rbX, luY); W = dist (luX, luY) to (rbX, luY)
            bbSizes.push_back( sqrt(calcDistanceSquared(rbX, rbY, rbX, luY)) * sqrt(calcDistanceSquared(luX, luY, rbX, luY)) );
            
            // calculate mean of shortest distance to other swarm agents
            double meanShortestDist = 0.0;
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    // find closest agent to agent i
                    double shortestDist = DBL_MAX;
                    
                    for(int j = 0; j < swarmSize; ++j)
                    {
                        if (!preyDead[j] && i != j)
                        {
                            if (preyDists[i][j] < shortestDist)
                            {
                                shortestDist = preyDists[i][j];
                            }
                        }
                    }
                    
                    // sum the shortest distance for agent i
                    meanShortestDist += sqrt(shortestDist);
                }
            }
            
            // average the shortest distances
            meanShortestDist /= (double)numAlive;
            
            // store mean shortest dist for this update
            shortestDists.push_back(meanShortestDist);
            
            // calculate swarm density count
            double avgWithin = 0.0;
            
            for(int i = 0; i < swarmSize; ++i)
            {
                if (!preyDead[i])
                {
                    for(int j = 0; j < swarmSize; ++j)
                    {
		      if (!preyDead[j] && i != j)
                        {
                            if (preyDists[i][j] <= safetyDist)
                            {
                                avgWithin += 1.0;
                            }
                        }
                    }
                }
            }
            
            avgWithin /= (double)numAlive;
            
            swarmDensityCounts.push_back(avgWithin);

	    double numVigilant = 0;
	    for(int i = 0; i < swarmSize; ++i)
	      {
		if(!preyDead[i] && vigilance[i])
		  {
		    numVigilant++;
		  }
	      }

	    numPreyVigilant.push_back(numVigilant);
            
        }
        /*       END OF DATA GATHERING       */
	
	/*       CHECK FORAGE-FLEE STATUS       */
	// if no predator is present
	if(foragePhase)
	  {
	    // if the delay between predator appearances is over
	    if(phaseDelay == 0)
	      {
		foragePhase = false;
		// place the predators on the grid
		for(int i = 0; i < predCount; ++i)
		  {
		    if(randDouble < 0.5)
		      {
			startX[i] = (double)(randDouble * boundaryDist * 2.0) - boundaryDist;
			if(randDouble < 0.5)
			  {
			    startY[i] = -boundaryDist;
			  }
			else
			  {
			    startY[i] = boundaryDist;
			  }
		      }
		    else
		      {
			startY[i] = (double)(randDouble * boundaryDist * 2.0) - boundaryDist;
			if(randDouble < 0.5)
			  {
			    startX[i] = -boundaryDist;
			  }
			else
			  {
			    startX[i] = boundaryDist;
			  }
		      }
		    predX[i] = startX[i];
		    predY[i] = startY[i];
		    predA[i] = randDouble * 359.0;
		  }
	      }
	    // if timer > 0, decrement timer
	    else
	      {
		phaseDelay--;
	      }
	  }
	// if predators are already on the grid
	else
	  {
	    bool restartForaging = true;
	    // check if all prey are dead or have fled
	    for(int i = 0; i < swarmSize; ++i)
	      {
		if(!preyDead[i] && !preyFled[i])
		  {
		    restartForaging = false;
		    break;
		  }
	      }
	    // reset the simulation if all the prey are dead or have fled
	    if(restartForaging)
	      {
		foragePhase = true;
		phaseDelay = killDelay;

		for(int i = 0; i < swarmSize; ++i)
		  {
		    if(!preyDead[i])
		      {
			receivedBroadcast[i] = false;
			sentBroadcast[i] = false;
			
			bool goodPos = true;
			
			do
			  {
			    goodPos = true;
			    
			    preyX[i] = 0.4 * ((double)(randDouble * gridX * 2.0) - gridX);
			    preyY[i] = 0.4 * ((double)(randDouble * gridY * 2.0) - gridY);
			    
			    for (int j = 0; j < i; ++j)
			      {
				if(!preyDead[i] && calcDistanceSquared(preyX[i], preyY[i], preyX[j], preyY[j]) < safetyDist)
				  {
				    goodPos = false;
				    break;
				  }
			      }
			  } while (!goodPos);
			
			preyA[i] = (int)(randDouble * 359.0);
			
			preyDead[i] = false;
			preyFled[i] = false;
		      }
		  }
	      }
	  }
	
        /*       UPDATE PREDATOR       */
	if(!foragePhase)
	  {
	    for(int i = 0; i < predCount; i++)
	      {
		// look for nearest prey
		int target = 0;
		double targetDist = DBL_MAX;
		for(int j = 0; j < swarmSize; j++)
		  {
		    if(!preyDead[j] && !preyFled[j])
		      {
			if(predDists[i][j] < targetDist)
			  {
			    target = j;
			    targetDist = predDists[i][j];
			  }
		      }
		  }
		// if the prey is close enough to kill
		if(targetDist < killDist & numAlive > 2)
		  {
		    predX[i] = preyX[target];
		    predY[i] = preyY[target];
		    preyDead[target] = true;
		    numAlive--;
		  }
		// if prey is outside of kill range
		else
		  {
		    // move toward nearest prey
		    double angleDiff = calcAngle(predX[i], predY[i], predA[i], preyX[target], preyY[target]);
		    if(abs(angleDiff) < predatorTurn)
		      {
			predA[i] += angleDiff;
		      }
		    else
		      {
			int sign = (angleDiff < 0 ? 1 : -1);
			predA[i] += sign * predatorTurn;
		      }	
		    predA[i] -= angleDiff;
		    while(predA[i] < 0) {predA[i] += 360;}
		    while(predA[i] >= 360) {predA[i] -= 360;}
		    predX[i] += cosLookup[(int)predA[i]] * predatorStep;
		    predY[i] += sinLookup[(int)predA[i]] * predatorStep;
		  }

		applyBoundary(predX[i]);
		applyBoundary(predY[i]);
	      }        
	  }

	// recalculate the predator distances lookup table since the predator has moved
	recalcPredDistTable(foragePhase, preyX, preyY, preyDead, predX, predY, predDists);
                        
        /*       END OF PREDATOR UPDATE       */
        
        /*       UPDATE SWARM       */
        for(int i = 0; i < swarmSize; ++i)
        {
	  if(!preyDead[i])
	    {
	      if(!preyFled[i])
		{
		  if(!foragePhase)
		    {
		      if(preyX[i] < (-0.8 * boundaryDist) || preyX[i] > (0.8 * boundaryDist) || preyY[i] < (-0.8 * boundaryDist) || preyY[i] > (0.8 * boundaryDist))
			{
			  preyFled[i] = true;
			}
		    }
		  if (preyX[i] > (-0.4 * boundaryDist) && preyX[i] < (0.4 * boundaryDist) && preyY[i] > (-0.4 * boundaryDist) && preyY[i] < (0.4 * boundaryDist))
		    {
		      if(vigilance[i])
			{
			  stomachs[i] += vigilanceFood;
			}
		      else
			{
			  stomachs[i]++;
			}
		    }
		  
		  //clear the sensors of agent i
		  for(int j = 0; j <= preySensors * 2; ++j)
		    {
		      swarm[i]->states[j] = 0;
		    }

		  // if enabled, provide an artificial broadcast if the prey can see a predator
		  if(artificialBroadcast)
		    {
		      for(int j = 0; j < predCount; ++j)
			{
			  if(predDists[j][i] < preyVisionRange)
			    {
			      swarm[i]->states[preySensors * 2] = 1;
			      break;
			    }
			}
		    }
		  // if artificial broadcast not enabled, behave normally
		  else
		    {
		      
		      // indicate that a signal was sent to the agent in the previous update
		      if(receivedBroadcast[i])
			{
			  swarm[i]->states[preySensors * 2] = 1;
			  receivedBroadcast[i] = false;
			}
		    }

		  if(vigilance[i])
		    {
		      // indicate the presence of other visible agents in agent i's retina
		      for(int j = 0; j < swarmSize; ++j)
			{
			  //ignore i==j because an agent can't see itself
			  if(i != j && !preyDead[j])
			    {
			      double d = preyDists[i][j];
			      
			      //don't bother if an agent is too far
			      if(d < preyVisionRange)
				{
				  // ignore if agent i isn't even facing agent j (won't be within retina)
				  if (calcDistanceSquared(preyX[i] + cosLookup[(int)preyA[i]],
							  preyY[i] + sinLookup[(int)preyA[i]],
							  preyX[j], preyY[j]) < d)
				    {
				      double angle = calcAngle(preyX[i], preyY[i], preyA[i], preyX[j], preyY[j]);
				      
				      // here we have to map the angle into the sensor, btw: angle in degrees
				      if(fabs(angle) < preyVisionAngle) // prey has a limited vision field in front of it
					{
					  swarm[i]->states[(int)(angle / (preyVisionAngle / ((double)preySensors / 2.0)) + ((double)preySensors / 2.0))] = 1;
					}
				    }
				}
			    }
			}
		      if(!foragePhase)
			{
			  for(int j = 0; j < predCount; j++)
			    {
			      
			      double d = predDists[j][i];
			      
			      if (d < preyVisionRange)
				{
				  double angle = calcAngle(preyX[i], preyY[i], preyA[i], predX[j], predY[j]);
				  
				  // here we have to map the angle into the sensor, btw: angle in degree
				  // prey has a limited vision field in front of it
				  if(fabs(angle) < preyVisionAngle)
				    {
				      swarm[i]->states[preySensors + (int)(angle / (preyVisionAngle / ((double)preySensors / 2.0)) + ((double)preySensors / 2.0))] = 1;
				    }
				}
			    }
			}
		      /*
		      // indicate the presence of food in agent i's retina
		      for(int j = 0; j < foodCount; ++j)
		      {
		      double d = foodDists[i][j];
		      // ignore if the food is too far
		      if(d < preyVisionRange)
		      {
		      // ignore if agent i is not facing the food
		      if(calcDistanceSquared(preyX[i] + cosLookup[(int)preyA[i]], preyY[i] + sinLookup[(int)preyA[i]], foodX[j], foodY[j]) < d)
		      {
		      double angle = calcAngle(preyX[i], preyY[i], preyA[i], foodX[j], foodY[j]);
		      // mapping the angle into the appropriate sensor
		      if(fabs(angle) < preyVisionAngle)
		      {
		      swarm[i]->states[(preySensors * 2) + (int)(angle / (preyVisionAngle / ((double) preySensors / 2.0)) + ((double) preySensors / 2.0))] = 1;
		      }
		      }
		      }
		      }
		      */
		    }
		}
	    }
        }
        
        // activate each swarm agent's brain, determine its action for this update, and update its position and angle
        for(int i = 0; i < swarmSize; ++i)
	  {
            if (!preyDead[i] && !preyFled[i])
	      {
		
		swarm[i]->updateStates();

		double movMultiplier = 1.0;
		if(!vigilance[i])
		  {
		    movMultiplier -= foragingMovePenalty;
		  }

		bool becomeVigilant = !!(swarm[i]->states[maxNodes - 4]);
		// if the current agent wants to enter a vigilant state for the next update
		if(becomeVigilant)
		  {
		    vigilance[i] = true;
		  }
		else
		  {
		    vigilance[i] = false;
		  }

		bool broadcast = !!(swarm[i]->states[maxNodes - 3]);
		// if the current agent wants to broadcast
		if(broadcast)
		  {
		    sentBroadcast[i] = true;
		    
		    if(artificialResponse)
		      {
			bool reward = false;
			for(int j = 0; j < predCount; ++j)
			  {
			    if(predDists[j][i] < preyVisionRange)
			      {
				reward = true;
				break;
			      }
			  }
			if(reward)
			    stomachs[i] += artificialReward;
			else
			  {
			    stomachs[i] -= artificialPenalty;
			    if(stomachs[i] < 0)
			      stomachs[i] = 0;
			  }
		      }
				
		    // find the alive prey within range
		    for(int j = 0; j < swarmSize; ++j)
		      {
			if(!preyDead[j])
			  {
			    if(preyDists[i][j] < broadcastDist)
			      {
				receivedBroadcast[j] = true;
			      }
			  }
		      }
		  }
		else
		  {
		    sentBroadcast[i] = false;
		  }
		
                //                              node 31                                      node 30
                int action = ((swarm[i]->states[maxNodes - 1] & 1) << 1) + (swarm[i]->states[maxNodes - 2] & 1);
		
                switch(action)
		  {
		    // stand still
		  case 0:
		    break;
                    
		    // turn 8 degrees right
		  case 1:
		    preyA[i] += preyTurn;
                    
		    while(preyA[i] >= 360.0)
		      {
			preyA[i] -= 360.0;
		      }
		    
		    preyX[i] += cosLookup[(int)preyA[i]] * preyStep * movMultiplier;
		    preyY[i] += sinLookup[(int)preyA[i]] * preyStep * movMultiplier;
                    
		    break;
			
		    // turn 8 degrees left
		  case 2:
		    preyA[i] -= preyTurn;
		    while(preyA[i] < 0.0)
		      {
			preyA[i] += 360.0;
		      }
		    
		    preyX[i] += cosLookup[(int)preyA[i]] * preyStep * movMultiplier;
		    preyY[i] += sinLookup[(int)preyA[i]] * preyStep * movMultiplier;
                    
		    break;
                    
		    // move straight ahead
		  case 3:
		    preyX[i] += cosLookup[(int)preyA[i]] * preyStep * movMultiplier;
		    preyY[i] += sinLookup[(int)preyA[i]] * preyStep * movMultiplier;
                    
		    break;
                    
		  default:
		    break;
		  }
                
                // keep position within boundary
                applyBoundary(preyX[i]);
                applyBoundary(preyY[i]);
	      }
	  }
      
        // recalculate both the predator and prey distances lookup tables since the entire swarm has moved
        recalcPredAndPreyAndFoodDistTable(foragePhase, preyX, preyY, preyDead, predX, predY, /*foodX, foodY,*/ predDists, preyDists/*, foodDists*/);
        
        /*       END OF SWARM UPDATE       */
        
        /*       DETERMINE FITNESSES FOR THIS UPDATE       */
        
	// fitness is only determined at the end of the simulation
        
        /*       END OF FITNESS CALCULATIONS       */
        
    }
    /*       END OF SIMULATION LOOP       */

    for(int i = 0; i < swarmSize; i++)
      {
	if(!preyDead[i])
	  {
	    swarmFitness += stomachs[i];
	  }
	delete swarm[i];
      }
    
    predatorFitness = swarmSize - numAlive;

    // compute overall fitness
    swarmAgent->fitness = swarmFitness;
    predatorAgent->fitness = predatorFitness;
    
    if(swarmAgent->fitness <= 0.0)
    {
        swarmAgent->fitness = 1.0;
    }
    
    if(predatorAgent->fitness <= 0.0)
    {
        predatorAgent->fitness = 1.0;
    }
    
    // output to data file, if provided
    if (data_file != NULL)
    {
        fprintf(data_file, "%d,%f,%f,%d,%f,%f,%f,%f,%i,%i,%i,%i,%f\n",
                swarmAgent->born,                               // update born (prey)
                swarmAgent->fitness,                            // swarm fitness
                predatorAgent->fitness,                         // predator fitness
                numAlive,                                       // # alive at end
                average(bbSizes),                               // average bounding box size
                variance(bbSizes),                              // variance in bounding box size
                average(shortestDists),                         // average of avg. shortest distance to other swarm agent
                average(swarmDensityCounts),                    // average # of agents within 20 units of each other
                neuronsConnectedToPreyRetina(swarmAgent),       // # neurons connected to prey part of retina (prey)
                neuronsConnectedToPredatorRetina(swarmAgent),   // # neurons connected to predator part of retina (prey)
                neuronsConnectedToPreyRetina(predatorAgent),    // # neurons connected to prey part of retina (predator)
                numAttacks,
		average(numPreyVigilant)                        // average # of prey vigilant per update
                );
    }
    
    return reportString;
}


// calculates the distance^2 between two points
double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
{
    double diffX = fromX - toX;
    double diffY = fromY - toY;
    
    return ( diffX * diffX ) + ( diffY * diffY );
}

// calculates the angle between two agents
double tGame::calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY)
{
    double Ux = 0.0, Uy = 0.0, Vx = 0.0, Vy = 0.0;
    
    Ux = (toX - fromX);
    Uy = (toY - fromY);
    
    Vx = cosLookup[(int)fromAngle];
    Vy = sinLookup[(int)fromAngle];
    
    double firstTerm = (Ux * Vy) - (Uy * Vx);
    double secondTerm = (Ux * Vx) + (Uy * Vy);
    
    return atan2(firstTerm, secondTerm) * 180.0 / cPI;
    //return atan2Lookup[firstTerm + 400][secondTerm + 400];
}

// calculates the center of the swarm and stores it in (cX, cY)
void tGame::calcSwarmCenter(double preyX[], double preyY[], bool preyDead[], double& preyCenterX, double& preyCenterY)
{
    int aliveCount = 0;
    preyCenterX = 0.0;
    preyCenterY = 0.0;
    
    for(int i = 0; i < swarmSize; ++i)
    {
        if (!preyDead[i])
        {
            preyCenterX += preyX[i];
            preyCenterY += preyY[i];
            ++aliveCount;
        }
    }
    
    preyCenterX /= (double)aliveCount;
    preyCenterY /= (double)aliveCount;
}

// recalculates only the predator distance lookup table
void tGame::recalcPredDistTable(bool foragePhase, double preyX[], double preyY[], bool preyDead[],
                                double predX[], double predY[],
				double predDists[predCount][swarmSize])
{
  if(foragePhase)
    {
      for (int i = 0; i < predCount; ++i)
	{
	  for(int j = 0; j < swarmSize; ++j)
	    {
	      if (!preyDead[j])
		{
		  predDists[i][j] = calcDistanceSquared(predX[i], predY[i], preyX[j], preyY[j]);
		}
	    }
	}
    }
}

// recalculates the predator, prey, and food distance lookup tables
void tGame::recalcPredAndPreyAndFoodDistTable(bool foragePhase, double preyX[], double preyY[], bool preyDead[],
					      double predX[], double predY[], /*double foodX[], double foodY[], */
					      double predDists[predCount][swarmSize], double preyDists[swarmSize][swarmSize]/*,
					      double foodDists[swarmSize][foodCount]*/)
{
  for(int i = 0; i < swarmSize; ++i)
    {
      if(!preyDead[i])
	{
	  if(!foragePhase)
	    {
	      for(int j = 0; j < predCount; ++j)
		{
		  predDists[j][i] = calcDistanceSquared(predX[j], predY[j], preyX[i], preyY[i]);
		}
	    }
	  preyDists[i][i] = 0.0;
	  for(int j = i + 1; j < swarmSize; ++j)
	    {
	    if(!preyDead[j])
	      {
		preyDists[i][j] = calcDistanceSquared(preyX[i], preyY[i], preyX[j], preyY[j]);
		preyDists[j][i] = preyDists[i][j];
	      }
	    }
	  /*
	    for(int k = 0; k < foodCount; ++k)
	    {
	      foodDists[i][k] = calcDistanceSquared(preyX[i], preyY[i], foodX[k], foodY[k]);
	    }
	  */
	}
    }
}

// maintains a position within a preset boundary
void tGame::applyBoundary(double& positionVal)
{
    double val = positionVal;
    
    if (fabs(val) > boundaryDist)
    {
        if (val < 0)
        {
            val = -1.0 * boundaryDist;
        }
        else
        {
            val = boundaryDist;
        }
    }
    
    positionVal = val;
}

// sums a vector of values
double tGame::sum(vector<double> values)
{
    double sum = 0.0;
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sum += *i;
    }
    
    return sum;
}

// averages a vector of values
double tGame::average(vector<double> values)
{
    return sum(values) / (double)values.size();
}

// computes the variance of a vector of values
double tGame::variance(vector<double> values)
{
    double sumSqDist = 0.0;
    double mean = average(values);
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sumSqDist += pow( *i - mean, 2.0 );
    }
    
    return sumSqDist /= (double)values.size();
}

double tGame::mutualInformation(vector<int> A,vector<int>B)
{
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]);
		nrB.insert(B[i]);
		pX[A[i]]=0.0;
		pY[B[i]]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]][B[i]]+=c;
		pX[A[i]]+=c;
		pY[B[i]]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pX[*aI]*pY[*bI]));
	return I;
	
}

double tGame::entropy(vector<int> list){
	map<int, double> p;
	map<int,double>::iterator pI;
	int i;
	double H=0.0;
	double c=1.0/(double)list.size();
	for(i=0;i<list.size();i++)
		p[list[i]]+=c;
	for (pI=p.begin();pI!=p.end();pI++) {
        H+=p[pI->first]*log2(p[pI->first]);	
	}
	return -1.0*H;
}

double tGame::ei(vector<int> A,vector<int> B,int theMask){
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]&theMask);
		nrB.insert(B[i]&theMask);
		pX[A[i]&theMask]=0.0;
		pY[B[i]&theMask]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]&theMask][B[i]&theMask]+=c;
		pX[A[i]&theMask]+=c;
		pY[B[i]&theMask]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pY[*bI]));
	return -I;
}

double tGame::computeAtomicPhi(vector<int>A,int states){
	int i;
	double P,EIsystem;
	vector<int> T0,T1;
	T0=A;
	T1=A;
	T0.erase(T0.begin()+T0.size()-1);
	T1.erase(T1.begin());
	EIsystem=ei(T0,T1,(1<<states)-1);
	P=0.0;
	for(i=0;i<states;i++){
		double EIP=ei(T0,T1,1<<i);
        //		cout<<EIP<<endl;
		P+=EIP;
	}
    //	cout<<-EIsystem+P<<" "<<EIsystem<<" "<<P<<" "<<T0.size()<<" "<<T1.size()<<endl;
	return -EIsystem+P;
}

double tGame::computeR(vector<vector<int> > table,int howFarBack){
	double Iwh,Iws,Ish,Hh,Hs,Hw,Hhws,delta,R;
	int i;
	for(i=0;i<howFarBack;i++){
		table[0].erase(table[0].begin());
		table[1].erase(table[1].begin());
		table[2].erase(table[2].begin()+(table[2].size()-1));
	}
	table[4].clear();
	for(i=0;i<table[0].size();i++){
		table[4].push_back((table[0][i]<<14)+(table[1][i]<<10)+table[2][i]);
	}
	Iwh=mutualInformation(table[0],table[2]);
    Iws=mutualInformation(table[0],table[1]);
    Ish=mutualInformation(table[1],table[2]);
    Hh=entropy(table[2]);
    Hs=entropy(table[1]);
    Hw=entropy(table[0]);
    Hhws=entropy(table[4]);
    delta=Hhws+Iwh+Iws+Ish-Hh-Hs-Hw;
    R=Iwh-delta;
  	return R;
}

double tGame::computeOldR(vector<vector<int> > table){
	double Ia,Ib;
	Ia=mutualInformation(table[0], table[2]);
	Ib=mutualInformation(table[1], table[2]);
	return Ib-Ia;
}

double tGame::predictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return mutualInformation(S, I);
}

double tGame::nonPredictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return entropy(I)-mutualInformation(S, I);
}

double tGame::predictNextInput(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	S.erase(S.begin());
	I.erase(I.begin()+I.size()-1);
	return mutualInformation(S, I);
}

void tGame::loadExperiment(char *filename){
    theExperiment.loadExperiment(filename);
}

int tGame::neuronsConnectedToPreyRetina(tAgent *agent){
    tAgent *A=new tAgent;
    int i,j,c=0;
    A->genome=agent->genome;
    A->setupPhenotype();
    for(i=0;i<A->hmmus.size();i++)
        for(j=0;j<A->hmmus[i]->ins.size();j++)
            if(A->hmmus[i]->ins[j]<preySensors)
                c++;
    delete A;
    return c;
}

int tGame::neuronsConnectedToPredatorRetina(tAgent* agent){
    tAgent *A=new tAgent;
    int i,j,c=0;
    A->genome=agent->genome;
    A->setupPhenotype();
    for(i=0;i<A->hmmus.size();i++)
        for(j=0;j<A->hmmus[i]->ins.size();j++)
            if((A->hmmus[i]->ins[j]<(preySensors*2))&&(A->hmmus[i]->ins[j]>=preySensors))
                c++;
    delete A;
    return c;
    
}

//** tOctuplet implementation
void tOctuplet::loadOctuplet(FILE *f){
    int i,IN;
    data.clear();
    data.resize(8);
    for(i=0;i<8;i++){
        fscanf(f,"  %i",&IN);
        data[i]=IN;
    }
}

//** tEperiment class implementations
void tExperiment::loadExperiment(char *filename){
    FILE *f=fopen(filename,"r+t");
    int i,j,k;
    fscanf(f,"%i:",&j);
    dropSequences.resize(j);
    for(i=0;i<dropSequences.size();i++)
        dropSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    sizeSequences.resize(j);
    for(i=0;i<sizeSequences.size();i++)
        sizeSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    selfSequences.resize(j);
    for(i=0;i<selfSequences.size();i++)
        selfSequences[i].loadOctuplet(f);
    shouldHit.resize(drops());
    for(i=0;i<shouldHit.size();i++){
        shouldHit[i].resize(sizes());
        for(j=0;j<shouldHit[i].size();j++){
            shouldHit[i][j].resize(selves());
            for(k=0;k<shouldHit[i][j].size();k++){
                int l;
                fscanf(f,"%i\n",&l);
                if(l==1)
                    shouldHit[i][j][k]=true;
                else
                    shouldHit[i][j][k]=false;
            }
        }
    }
    fclose(f);
}

void tExperiment::showExperimentProtokoll(void){
    int i,j,k;
    printf("drop directions: %i\n",drops());
    for(i=0;i<drops();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",dropSequences[i].data[j]);
        printf("\n");
    }
    printf("drop sizes: %i\n",sizes());
    for(i=0;i<sizes();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",sizeSequences[i].data[j]);
        printf("\n");
    }
    printf("self sizes: %i\n",selves());
    for(i=0;i<selves();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",selfSequences[i].data[j]);
        printf("\n");
    }
    printf("should hit\n%i means true\nD  B   S   catch\n",(int)true);
    for(i=0;i<shouldHit.size();i++)
        for(j=0;j<shouldHit[i].size();j++)
            for(k=0;k<shouldHit[i][j].size();k++)
                printf("%i  %i  %i  %i\n",i,j,k,(int)shouldHit[i][j][k]);
}

int tExperiment::drops(void){
    return (int) dropSequences.size();
}

int tExperiment::sizes(void){
    return (int) sizeSequences.size();
}

int tExperiment::selves(void){
    return (int) selfSequences.size();
    
}
