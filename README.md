# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

#### Submission
All you will submit is your completed version of `particle_filter.cpp`, which is located in the `src` directory. You should probably do a `git pull` before submitting to verify that your project passes the most up-to-date version of the grading code (there are some parameters in `src/main.cpp` which govern the requirements on accuracy and run time.)

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 


## Localasation

### No clue, the initialise belief
By default, our confusion is by "a uniform function that assigns equal weight to 
every possible place in this world. This is the state of maximum confusion.

p[Xi] = 1 / timeSteps

### Three non-distinguishable doors
The measurement of a door transforms our belief function defined over possible locations
to a new function that looks like this. Posterior belief is happened after measurements
We still don't know where we are as there are 3 possible doors, possibility 
the sensors were erroneous and we accidentally saw a door where there's none.
![Posterior belief](/docs/Posterior_belief.png)

As the car moves, when it saw the second door, the prior belief 
from first door plus Posterior belief with second door will looks like
this. The car able to localise it.
![Prior+Posterior_Belief](/docs/Prior+Posterior_Belief.png)

if found 
  x = x * 0.6
else 
  x = x * 0.2
  
As probability distributions always have to add up to 1, the total not equal to 1, we re-normalize it.

![Normalize_Probability_Distribution](/docs/Normalize_Distribution.png)


