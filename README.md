This is Evolution of Swarming project.

Copyright 2013 Randal S. Olson, Arend Hintze.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Compiling
====================

Enter the command ./build_eos

If the console gives an error about permissions, enter:

chmod 755 build_eos

then enter the above build command again.

Usage
====================

Type ./eos to run the simulation. Note that by default, EOS attempts to seed the predator population with a Markov Network genome saved in a file called, `startPredator.genome`.

The following parameters can be passed to EOS:

* -e [LOD out file name] [prey genome out file name] [predator genome out file name]: evolve
* -d [prey genome in file name] [predator genome in file name]: display 
* -dd [directory of prey and predator genome files]: display all genome files in a given directory
* -s [int]: set random number generator seed
* -g [int]: set generations to evolve for
* -t [int]: save best brain every [int] generations
* -v [int]: make video of best brains at the given interval
* -lv: make video of LOD of best agent brain at the end of run
* -lt [genome in file name] [out file name]: create logic table for given genome
* -dfs [genome in file name] [dot out file name]: create dot image file for given swarm genome
* -dfp [genome in file name] [dot out file name]: create dot image file for given predator genome
* -sd [int]: set swarm safety distance (default: 30)
* -pva [int]: set predator vision angle (default: 180)
* -kd [int]: set predator kill attempt delay (default: 10)
* -cm [float]: set predator confusion multiplier (default: 1.0)
* -vfp [float]: set prey penalty to food when vigilant (default: 1.0, range: [0,1])
* -pop [float]: set population size for the GA
* -gsz [float]: set group size for the evaluation
* -cln: set evaluation to work on clones of genotypes (default: heterogeneous, with flag: homogeneous)
* -zdp: use to zero out the fitness of prey that die (default: iteroparous, with flag: semelparous)
* -aar [float]: set absolute attack rate for the evaluation
* -rar [float]: set attacks per individual for the evaluation
* -grp [float]: set grouping mode for the evaluation (0 is forced grouping, 1 is forced ungrouping, 2 is choose to group)
* -gp [float]: penalize grouping relative to gruop size (default: 1.0, range: [0,1])
* -ff [float]: set the amount of food a non-vigilant prey receives
* -egs: set program to evolve group sizes in addition to agents
* -mgs: give each agent a maximum group size (will group if current size <= agent's max)

Experimental Replication
====================

To replicate the experiments in the Royal Society Open Science paper, use the following command:

./eos -e ${OUTDIR}/csvs/${RUN}LOD.csv ${OUTDIR}/genomes/${RUN}swarm.genome -s ${PBS_ARRAYID} -g 2500 -pop 100 -gsz ${GROUPSIZE} -rar 05 -grp ${GROUPMODE} -ff 1.0 -vfp 1.0 -gp ${GROUPPENALTY}

Variables:
* ARRAYID: run at 1 through 100
* GROUPSIZE: run at sizes 05, 10, 25, 50
* GROUPMODE: run at 0 (grouped), 1 (individual), and 2 (optional)
* GROUPPENALTY: run at 0 (without -gp flag), 1, and 1000

For the four treatments:
* Heterogeneous/Iteroparous: no flags
* Heterogeneous/Semelparous: -zdp flag
* Homogeneous/Iteroparous: -cln flag
* Homogeneous/Semelparous: -zdp and -cln flags

Output
====================

EOS produces a variety of output files, detailed below.

CSV Files
---------------------

File storing per generation data across the duration of the run.

Columns:
* Generation
* LOD Vigilance: vigilance of ancestor organisms of the final dominant genotype
* Average Vigilance: percentage time vigilant averaged across all the genotypes in the population
* Vigilance Variance: variance of the previous value
* Average Fitness:fitness averaged across all the genotypes in the population
* Fitness Variance: variance of the previous value
* Average Grouping: percentage time gruoped averaged across all the genotypes in the population
* Grouping Variance: variance of the previous value
* Average Vigilance & Grouping: percentage time vigilant while grouped averaged across all the genotypes in the population
* Vigilance & Grouping Variance: variance of the previous value
* Average Maximum Group Size: maximum group size averaged across all the genotypes in the population
* Maximum Group Size Variance: variance of the previous value

LOD files
---------------------

There will be a single entry for each ancestor in the final best swarm agent's LOD.

LOD files will be in csv format with the column headers listed at the top. Column headers are in the following order:

* generation: the generation the ancestor was born
* prey_fitness: the fitness of the ancestor prey
* predator_fitness: the fitness of the ancestor predator
* num_alive_end: the number of surviving prey at the end of the fitness evaluation
* avg_bb_size: the average bounding box size of the swarm during the simulation
* var_bb_size: the variance in the bounding box size of the swarm during the simulation
* avg_shortest_dist: the average distance from every prey agent to the prey agent closest to it
* swarm_density_count: the average number of prey agents within *safety distance* units of each other
* prey_neurons_connected_prey_retina: the number of conspecific sensor neurons that the prey Markov network brain is connected to
* prey_neurons_connected_predator_retina: the number of predator sensor neurons that the prey Markov network brain is connected to
* predator_neurons_connected_prey_retina: the number of prey sensor neurons that the predator Markov network brain is connected to
* mutual_info: the average amount of mutual information between all of the prey's navigation angles and the predator's navigation angle

Markov network brain files
---------------------

Generally, we save Markov network brain files as .genome files.

These files contain integer values which encode the Markov network brain.

Logic table files
---------------------

The logic table files contain the logic table for the most-likely decision made by the Markov network brain.

They are formatted specifically for the Logic Friday logic optimization program. They should be able to be fed directly into the Logic Friday program without any modification.

DOT files
---------------------

DOT files are the picture representations of Markov network brain structure and connectivity. We recommend using the Graphviz software to view these images.
