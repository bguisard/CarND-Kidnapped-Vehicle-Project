# Particle Filters for Localization
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project's objective is to use a 2D Particle Filter to assist a vehicle in more accurately determining it's location using a map with landmarks and estimated GPS coordinates.


## Particle Filters

Particle filter is a very powerful algorithm that uses a finite number of particles to model the posterior distribution of some stochastic process given a set of partial observations and uncertainties.

Particle filters are also part of the Recursive Bayesian estimators family and is a great alternative to Kalman Filters for low dimensional spaces since it's very easy to calculate and implement.

### Hyperparameters

The only hyperparameter we need to optimize in a Particle Filter is the number of particles (N) that will be used. The more particles you have, the smaller the error of your filter, but conversely, more computation is required at each timestep.

This creates an interesting problem when designing particle filters for real-time applications, as you need enough particles to accurately represent the state space, while being mindful of performance bottlenecks.

For this application I optimized the number of particles to be the smallest number that would keep the localization error at approximately 0.1m.


## The model

The image below shows a roadmap to successfully implement a particle filter algorithm.

![alt text][image1]

Source: Udacity lectures on Particle filters


### 1. Initialization

In the initialization step the algorithm randomly generates N particles in the state space. If your position is completely unknown you would need a very large number of particles to cover most of the state space.

In our case we start with the GPS location of the vehicle, but since the error in GPS measurements are too large for autonomous driving we use particle filters to better estimate the vehicle's location.

As we have a rough estimate of where the vehicle is, we don't need to initialize particles all over the state space, the efficient method is to randomly initialize the particles with the mean being the exact GPS coordinates and a standard deviation equal to the uncertainty of the GPS measurements.

### 2. Prediction

For the prediction step we rely on motion models to predict where each of these particles should be on the next timestep given the state of the particles (yaw rate and velocity). To account for the uncertainty in the control input, I have also added gaussian noise to the velocity and yaw rate for each particle.

### 3. Update

The update step is where we incorporate the measurements from sensors with the map landmark positions and the predicted position of each particle and assign weights that are proportional to the probability of that particle being the one that best represents the current location of our vehicle.

In order to achieve that we assign each sensor observation to a possible map landmark, measure the distance from the particle to all landmarks and finally calculate the posterior probability density function.

### 4. Resample

After all the weights are updated we resample our N particles, with replacement, in proportion to their normalized weights.

This gives particles with higher weights a better chance to "survive" the resampling step, meaning that after each iteration of the algorithm will most likely eliminate particles that are inaccurate, replacing them with particles that have a high likelihood of representing the most accurate position of the vehicle.

## Dealing with different coordinate systems

The measures from the sensors are taken in the car's reference, meaning that all distances are from the car (or each particle) to the object, but the landmarks coordinates are in the map reference. So before assigning each observation to a landmark, we need to convert the observation's coordinates into map coordinates.

The transformation is just a rotation and translation of the map origin into the particle's location. More details can be found on [[1](http://planning.cs.uiuc.edu/node99.html)].


## Assigning landmarks to observations

With the observations and landmarks now using the same reference we can assign each observation to a given landmark in our map.

There are several different methods to do this, and in this project I am using a nearest neighbor algorithm to make this association.

Nearest neighbor is easy to understand and implement, and give great results in our simplified dataset, but may not be ideal for real world applications.

Some of the known weak spots of nearest neighbors are:
- Increased error in an environment with high density of measurements
- Inefficient computation (specially bad for environment with lots of landmarks)
- It does not take the uncertainties of the sensors into account
- If noise of the sensor is higher than distance between landmarks the risk of assigning an observation to the wrong landmark is very high


## Final Considerations

Despite the decision to use a simplistic data association methodology, the results achieved were very inspiring. Using only 10 particles I was able to reduce the vehicle's position uncertainty in half and localize the vehicle with a 15cm precision.

But every cm of performance gets more and more expensive, this model needed 75 particles to get it's uncertainty to 11cm, at which point adding more particles were not giving any meaningful improvement.

The path for further increase in performance, on this dataset and real world applications, is certainly replacing the nearest neighbor algorithm for data association by a more robust methodology.

## References
[1] [Planning Algorithms](http://planning.cs.uiuc.edu/node99.html)


[//]: # (Image References)

[image1]: ./images/particle_filter.png "Particle Filter model"
