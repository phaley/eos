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
#define unawareKillProb         1.00
#define groupAwareKillProb      0.30
#define indvAwareKillProb       0.10
#define groupAwareThreshold     5
#define incrementProbs          false

#define FORCED_GROUP            0
#define FORCED_LONER            1
#define CHOOSE_MODE             2

#define attackDuration          5
#define attackDelayRange        10/2
#define totalStepsInSimulation  5000

// precalculated lookup tables for the game
double cosLookup[360];
double sinLookup[360];
//double atan2Lookup[(int)gridY * 4][(int)gridX * 4];

int attackDelayMean = 5;

tGame::tGame() { }

tGame::~tGame() { }

// runs the simulation for the given agent(s)
string tGame::executeGame(vector<tAgent*> & swarmAgents, int swarmSize, FILE *data_file, bool report, double confusionMultiplier,
			  double vigilanceFoodPenalty, bool zeroOutDeadPrey, int groupMode, bool relativeAttackRate, int attackRate,
			  bool penalizeGrouping, double groupingPenalty, double foragingFood, bool maxGroupSize)
{
    // LOD data variables
    double swarmFitness = 0.0;
    // counter of how many swarm agents are still alive
    int numAlive = swarmSize;
    
    vector<double> numPreyVigilant;

    if(relativeAttackRate)
      {
	attackDelayMean = (int) ((totalStepsInSimulation - (attackDuration * attackRate * swarmSize)) / (attackRate * swarmSize));
      }
    else
      {
	attackDelayMean = attackRate;
      }
    int predatorDelay = 0;
    int attackCounter = attackDuration;

    // set of all prey brains
    tAgent* swarm[swarmSize];
    // whether a given agent is alive
    bool preyDead[swarmSize];
    // whether a given agent is vigilant
    bool vigilance[swarmSize];
    // whether a given agent is aware of the predator
    bool awareness[swarmSize];
    // whether a given agent has chosen to group
    bool grouped[swarmSize];
    // food reward while vigilant
    double vigilanceFood = foragingFood - vigilanceFoodPenalty;

    // tables of agents to receive broadcast signals
    bool receivedBroadcast[swarmSize];
    bool sentBroadcast[swarmSize];

    int groupAwareProbStep = (int) ((groupAwareKillProb - indvAwareKillProb) / (swarmSize / groupAwareThreshold));
 
    // string containing the information to create a video of the simulation
    string reportString = "";
    
    for(int i = 0; i < swarmSize; ++i)
      {
	//setup brain for swarm of individuals
	swarm[i] = new tAgent();
	swarm[i]->inherit(swarmAgents[i], 0.0, 0);
	swarm[i]->setupPhenotype();
	swarm[i]->fitness = 1.0;
	preyDead[i] = false;
	vigilance[i] = false;
	awareness[i] = false;
	receivedBroadcast[i] = false;
	sentBroadcast[i] = false;
	if(maxGroupSize)
	  grouped[i] = (randDouble > 0.5) ? true : false;
	else
	  grouped[i] = (groupMode == FORCED_GROUP) ? true : false;
      }
    
    for(int step = 0; step < totalStepsInSimulation; ++step)
      {        
	/*       SAVE DATA FOR THE LOD FILE       */
	if(data_file != NULL)
	  {
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
	
	/*       UPDATE PREDATOR       */

	// if the predator is on the scene, it either waits or makes an attack
	if(predatorDelay == 0)
	  {
	    // if the predator has waited long enough, it makes an attack
	    if(attackCounter == 0)
	      {
		// check to make sure there are still potential targest available
		if(numAlive > 0)
		  {
		    // randomly select a target from the surviving prey
		    int target;
		    //target = (int) (randDouble * swarmSize); // allow the predator to target dead prey
		    
		    do {
		      target = (int) (randDouble * swarmSize);
		    } while(preyDead[target]);
		    
		    if(!preyDead[target])
		      {
			// if the target sees the predator coming, the attack success varies accordingly
			if(awareness[target])
			  {
			    if(indvAwareKillProb > randDouble)
			      {
				preyDead[target] = true;
				numAlive--;
			      }
			  }
			// if the target is unaware, see who else in the group is aware of the predator
			else
			  {
			    int awareCount = 0;
			    bool groupAware = false;
			    for(int i = 0; i < swarmSize; ++i)
			      {
				if(!preyDead[i] && grouped[i])
				  {
				    if(awareness[i])
				      {
					awareCount++;
					groupAware = true;
				      }
				  }
			      }
			    // if the target is in the group and the group sees the predator coming,
			    // the attack success varies accordingly
			    if(grouped[target] && groupAware)
			      {
				// if the attack success depends on how many conspecifics are aware,
				// then make the attack accordingly
				if(incrementProbs)
				  {
				    if((groupAwareKillProb - (groupAwareProbStep * awareCount)) > randDouble)
				      {
					preyDead[target] = true;
					numAlive--;
				      }
				  }
				// make a regular attack if we don't care about the number of aware conspecifics
				else
				  {
				    if(groupAwareKillProb > randDouble)
				      {
					preyDead[target] = true;
					numAlive--;
				      }
				  }
			      }
			    // if no one sees the predator coming, make the attack accordingly
			    else
			      {
				if(unawareKillProb > randDouble)
				  {
				    preyDead[target] = true;
				    numAlive--;
				  }
			      }
			  }
		      }
		  }		

		if(numAlive > 0) {
		  attackDelayMean = (int) ((totalStepsInSimulation - (attackDuration * numAlive * attackRate)) / (attackRate * numAlive)); // adjust attack rate each time
		}

		predatorDelay = newPredDelay();
		attackCounter = attackDuration;
		// once an attack is made, reset group awareness
		for(int i = 0; i < swarmSize; ++i)
		  {
		    awareness[i] = false;
		  }
	      }
	    // if the predator is not ready to make an attack, decrement the counter
	    else
	      {
		attackCounter--;
	      }
	  }
	// if the predator is not on the scene, decrement the counter
	else
	  {
	    predatorDelay--;
	  }
        
	/*       END OF PREDATOR UPDATE       */
        
	// count the number of prey in the group
	int numGrouped = 0;
	for(int i = 0; i < swarmSize; ++i)
	  {
	    if(!preyDead[i] && grouped[i])
	      {
		numGrouped++;
	      }
	  }

	//	if(step == 1999)
	//cout << "Grouped: " << (((double) numAlive) / swarmSize) << endl;

	/*       UPDATE SWARM       */
	for(int i = 0; i < swarmSize; ++i)
	  {
	    // we do not update dead prey
	    if (!preyDead[i])
	      {
		// if prey are non-vigilant, they get food
		if(!vigilance[i])
		  {
		    if(!preyDead[i])
		      {
			// if the prey is in the group, any penalty is assessed
			if(grouped[i] && penalizeGrouping)
			  {
			    swarm[i]->fitness += foragingFood / pow(numGrouped, groupingPenalty);
			    //swarm[i]->fitness += foragingFood / (numGrouped * groupingPenalty);
			  }
			// otherwise, food intake is normal
			else
			  {
			    swarm[i]->fitness += foragingFood;
			  }
		      }
		  }
		// if the prey is vigilant, then it can see the predator and/or get some food
		else
		  {
		    // if the predator is on the scene, the vigilant prey see it
		    if(attackCounter == 0)
		      {
			awareness[i] = true;
		      }
		    // if the prey is in the group, a group penalty can be assessed on top of the vigilance penalty
		    if(grouped[i] && penalizeGrouping)
		      {
			swarm[i]->fitness += vigilanceFood / pow(numGrouped, groupingPenalty);
			//swarm[i]->fitness += vigilanceFood / (numGrouped * groupingPenalty);
		      }
		    // if the prey is not in the group, the prey get whatever food vigilant prey might receive
		    else
		      {
			swarm[i]->fitness += vigilanceFood;
		      }
		  }
		// activate each swarm agent's brain and determine its action for this update
  		swarm[i]->updateStates();
		// state 0 represents the decision to be vigilant
		if((swarm[i]->states[0] & 1) == 1)
		  {
		    vigilance[i] = true;
		  }
		else
		  {
		    vigilance[i] = false;
		  }
		if(maxGroupSize && (swarm[i]->states[1] & 1) == 1)
		  {
		    if(swarm[i]->maxGroupSize > numGrouped)
		      {
			grouped[i] = true;
		      }
		    else
		      {
			grouped[i] = false;
		      }
		  }
		// we only care about the decision to group if that option is available
		else if(groupMode == CHOOSE_MODE)
		  {
		    // state 1 represents the decision to live in the group or not
		    if((swarm[i]->states[1] & 1) == 1)
		      {
			grouped[i] = true;
		      }
		    else
		      {
			grouped[i] = false;
		      }
		  }
	      }
	  }
        /*       END OF SWARM UPDATE       */
        
      }
    /*       END OF SIMULATION LOOP       */
  
    // fitness is passed back to our original genomes and the copies are deleted
    for(int i = 0; i < swarmSize; ++i)
      {
	if(!preyDead[i] || !zeroOutDeadPrey)
	  {
	    swarmAgents[i]->fitness = swarm[i]->fitness;
	  }
	else
	  {
	    swarmAgents[i]->fitness = 1;
	  }
	delete swarm[i];
      }
  
    // output to data file, if provided
    if (data_file != NULL)
    {
      /*
        fprintf(data_file, "%d,%f,%f,%d,%i,%i,%i,%f\n",
                swarmAgent->born,                               // update born (prey)
                swarmAgent->fitness,                            // swarm fitness
                numAlive,                                       // # alive at end
		average(numPreyVigilant)                        // average # of prey vigilant per update
                );
      */
    }
    
    return reportString;
}

// returns a normally distributed predator delay
int tGame::newPredDelay(void) {
    int delay;
    do {
      int sign = randDouble > 0.5 ? 1 : -1;
      delay = ((int) (sqrt(-2 * log(randDouble)) * cos(2 * cPI * randDouble) * attackDelayRange) * sign) + attackDelayMean;
    } while(delay <= 0);
    return delay;
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
