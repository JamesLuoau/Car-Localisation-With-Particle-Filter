
## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. 

## Running Results from those code set
![Particle Filter Car Localisation With Landmark in map and sensor obersavations](/docs/Particle Filter Car Localisation With Landmark in map and sensor obersavations.gif)


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


### Move Motion
The movement of car self could be in-accurate.
![Inexact_Motion](/docs/Inexact_Motion.png)


### Probability and Bayes' Rule
![Bayes_Rule_3](/docs/Bayes_Rule_3.png)

![Bayes_Rule](/docs/Bayes_Rule.png)

![Bayes_Rule_2](/docs/Bayes_Rule_2.png)

![Bayes_Rule_2](/docs/Bayes_Rule_Cancer_Test.png)

### Total Probability
![Total_Probability_Event_Space](/docs/Total_Probability_Event_Space.png)

![Total_Probability](/docs/Total_Probability.png)

![Total_Probability_Formal](/docs/Total_Probability_Formal.png)

![Total_Probability_Coin](/docs/Total_Probability_Coin.png)

![Bayes_Rule_Two_Coin](/docs/Bayes_Rule_Two_Coin.png)

![Bayes_Rule_Two_Coin_MyAnswer](/docs/Bayes_Rule_Two_Coin_MyAnswer.png)



## Localization Posterior 

Localization is all about estimating the probability distribution of the state xt (the pose of the car),
on that condition that all previous observations z(range, measurements...) that from time 1 to t,
all previous controls u (yaw/pitch/roll rates, velocities) from time 1 to t, 
also map m (assume not changed) are given

![probability distribution of the state car pose](/docs/probability%20distribution%20of%20the%20state%20car%20pose.png)


### Data Input


m = 1d map

![1D Map](/docs/1D_Map.png)

```text
1	9		
2	15		
3	25		
4	31	
5	59	
6	77
```

observations z (range, measurements...)

![z](/docs/input_z.png)

```text
Observation 1----------
4.5	
32

Observation 2----------
#Empty

Observation 3----------
39.5	
3

Observation 4----------
#Empty

```

Control Vector u (yaw/pitch/roll rates, velocities)


![control_vector_u](/docs/input_control_vector.png)

```text
#For example Delta velocities, below moved 1m for every time interval
1	
1
1
1
1
1
1
1
1
1
1
1
1
1
```


### The Problems 

![two_problems_for_full_calculation](/docs/two_problems_for_full_calculation.png)


We aim to estimate state beliefs bel(x​t) without the need to carry our entire observation 
history. We will accomplish this by manipulating our posterior p(xt∣z1:t−1,μ1:t,m), obtaining 
a recursive state estimator. For this to work, we must demonstrate that our current belief bel(xt) 
can be expressed by the belief one step earlier bel(xt−1), then use new data to update 
only the current belief. This recursive filter is known as the Bayes Localization filter or 
Markov Localization, and enables us to avoid carrying historical observation and motion data. 
We will achieve this recursive state estimator using Bayes Rule, the Law of Total Probability, 
and the Markov Assumption.

convert_to_use_previews_state_only

![convert_to_use_previews_state_only](/docs/convert_to_use_previews_state_only.png)


split_observation_z_to_current_and_previous

![split_observation_z_to_current_and_previous](/docs/split_observation_z_to_current_and_previous.png)

![split_observation_z_to_current_and_previous](/docs/convert_to_use_previews_state_only_new_fomula.png)

Observation_and_Motion_Model

![Observation_and_Motion_Model](/docs/Observation_and_Motion_Model.png)

Observation model describes the probability distribution of the observation vector t
another assumption that a state x_t are previous observations, all controllers, all 
map are given.
Motion model is the probability distribution of x_t given all observations from
one to t-1, all controllers and map and take account that no currect observations are
included in the motion model

Observation_and_Motion_made_Normalization

![Observation_and_Motion_made_Normalization](/docs/Observation_and_Motion_made_Normalization.png)

Observation_and_Motion_made_Normalization

![Observation_and_Motion_made_Normalization](/docs/Observation_and_Motion_made_Normalization_2.png)

Total_Probability_and_Markov_Assumptions???

![Total_Probability_and_Markov_Assumptions](/docs/Total_Probability_and_Markov_Assumptions.png)


Motion_Model_Before_Markov_Assumption

![Motion_Model_Before_Markov_Assumption](/docs/Motion_Model_Before_Markov_Assumption.png)

Motion_Model_After_Markov_Assumption

![Motion_Model_After_Markov_Assumption](/docs/Motion_Model_After_Markov_Assumption.png)

Recursive_Structure

![Recursive_Structure](/docs/Recursive_Structure.png)


### Motion Model Probability

[Motion_Model_Probability PDF](/docs/Motion_Model_Probability.pdf)

[Motion_Model_Probability Source Code](/src/main_motion_model.cpp)


### Observation Model

![Observation_Model_Details](/docs/Observation_Model_Details.png)


[Observation_Model_Probability PDF](/docs/Observation_Model_Probability.pdf)


[Observation_Model_Probability Source Code](/src/main_observation_model.cpp)

```c
class Helpers {
public:

	//definition of one over square root of 2*pi:
	constexpr static float STATIC_ONE_OVER_SQRT_2PI = 1/sqrt(2*M_PI) ;
	float ONE_OVER_SQRT_2PI = 1/sqrt(2*M_PI) ;

	/*****************************************************************************
	 * In Observation Model, x is measurment from car position, measurment has error which put in std
	 * mu is the mean of measurment, which is from car position how far away from HD map
	*****************************************************************************/
	static float normpdf(float x_measurment_from_car_position, float mu_mean_land_mark_from_car_position, float std_car_measurment) {
	    return (STATIC_ONE_OVER_SQRT_2PI/std)*exp(-0.5*pow((x-mu)/std,2));
	}
	
};

```



## Bicycle Motion Models

### Position with Yaw Rate, Velocity, time spent

![Position with Yaw Rate ZERO, Velocity, time spent](/docs/Bicycle_Model_New_Position_When_Yaw_Zero.png)

![Position with Yaw Rate NOT ZERO, Velocity, time spent](/docs/Bicycle_Model_New_Position_When_Yaw_Not_Zero.png)

![New_Position_With_Odometry](/docs/Bicycle_Model_New_Position_With_Odometry.png)

## Particle Filters

![Particle_Filters_Never_Sampled](/docs/Particle_Filters_Never_Sampled.png)

















