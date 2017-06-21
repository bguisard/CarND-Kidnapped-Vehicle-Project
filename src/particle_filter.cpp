/*
 *  particle_filter.cpp
 *  purpose: Uses a 2D particle filter to localize a car in a map
 *
 *  @author Bruno Guisard
 *  @version 0.1 6/21/2017
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

// Initializes the N particles to be determined by num_particles
void ParticleFilter::init(double x, double y, double theta, double std[]) {

  // User defined number of particles => Directly impacts algorithm performance
  num_particles = 10;

  // Initializes a random generator to assist in our gaussian sampling
  default_random_engine gen;

  // Creates normal distribution for each parameter: x, y and theta
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1;

    particles.push_back(p);
  }

  is_initialized = true;

}

// Predicts position of each particle given delta_t, noises, velocity and yaw rate 
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

  // Initializes a random generator to assist in our gaussian sampling
  default_random_engine gen;

    double new_theta;
    double new_x_mean;
    double new_y_mean;

  for (int i = 0; i < num_particles; i++) {

    if (fabs(yaw_rate) < 0.0001) {
      new_theta = particles[i].theta;
      new_x_mean = particles[i].x + (velocity * delta_t * cos(new_theta));
      new_y_mean = particles[i].y + (velocity * delta_t * sin(new_theta));
    } else {
      new_theta = particles[i].theta + yaw_rate * delta_t;
      new_x_mean = particles[i].x + (velocity / yaw_rate) * (sin(new_theta) - sin(particles[i].theta));
      new_y_mean = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(new_theta));
    }

    // Creates normal distribution for each parameter: x, y and theta
    normal_distribution<double> dist_x(new_x_mean, std_pos[0]);
    normal_distribution<double> dist_y(new_y_mean, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

    particles[i].x =  dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

// Unused - to be deleted
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
  //   implement this method and use it as a helper during the updateWeights phase.

}

// Calculates weights for each individual particle
void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
    std::vector<LandmarkObs> observations, Map map_landmarks) {

  // NAIVE WAY - CAN BE OPTIMIZED LATER
  // STEPS: 
  //  1 - For each particle convert measurements to map coordinate
  //  2 - For each observation find the closest landmark
  //  3 - Calculate error for the pair obs x best landmark
  //  4 - Accumulate error for particle
  
  // variable to store coords converted to map scale
  LandmarkObs converted_obs;
  LandmarkObs best_landmark;

  // clears weight vector
  weights.clear();

  // loop through all particles
  for (int i = 0; i < int(particles.size()); i++) {
    double prob = 1.;

    // loop through all observations
    for (int k = 0; k < int(observations.size()); k++) {
      //  1 - For each particle convert measurements to map coordinate
      converted_obs = transformCoords(particles[i], observations[k]);

      //  2 - Assign observation to a landmark
      best_landmark = associateLandmark(converted_obs, map_landmarks, std_landmark);

      //  3 - Calculate weight for the pair obs x best landmark
      double e = calculateWeights(converted_obs, best_landmark, std_landmark);

      //  4 - Accumulate weights for particle
      prob *= e;
    }

    // store weight in particle
    particles[i].weight = prob;

    // and in the weights vector
    weights.push_back(prob);

  }
}

// Weight based resampling the particles with replacement, allowing more relevant particles to be picked more frequently
void ParticleFilter::resample() {

  std::vector<Particle> new_particles;
  int index;

  // Initializes discrete distribution function
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<int> weight_distribution(weights.begin(), weights.end());

  for (int i = 0; i < num_particles; i++) {
    index = weight_distribution(gen);
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
}

// Convert coordinates from particle to map 
LandmarkObs ParticleFilter::transformCoords(Particle part, LandmarkObs obs) {

  LandmarkObs transformed_coords;

  transformed_coords.id = obs.id;
  transformed_coords.x = obs.x * cos(part.theta) - obs.y * sin(part.theta) + part.x;
  transformed_coords.y = obs.x * sin(part.theta) + obs.y * cos(part.theta) + part.y;

  return transformed_coords;
}

// Naive association of landmark to particle
LandmarkObs ParticleFilter::associateLandmark(LandmarkObs converted_obs, Map map_landmarks, double std_landmark[]) {
  LandmarkObs best_landmark;

  for (int m = 0; m < int(map_landmarks.landmark_list.size()); m++) {
    double min_dist;
    double distance = dist(converted_obs.x, converted_obs.y, map_landmarks.landmark_list[m].x_f, map_landmarks.landmark_list[m].y_f);
    if (m==0) {
      min_dist = distance;
      best_landmark.id = map_landmarks.landmark_list[m].id_i;
      best_landmark.x = map_landmarks.landmark_list[m].x_f;
      best_landmark.y = map_landmarks.landmark_list[m].y_f;
    } else if (distance < min_dist) {
      min_dist = distance;
      best_landmark.id = map_landmarks.landmark_list[m].id_i;
      best_landmark.x = map_landmarks.landmark_list[m].x_f;
      best_landmark.y = map_landmarks.landmark_list[m].y_f;
    }
  }

  return best_landmark;
}

// Calculates the error between the estimated position and the ground truth position of the landmark
double ParticleFilter::calculateWeights(LandmarkObs obs, LandmarkObs best_landmark, double std_landmark[]) {

  // storing for readability
  const double sigma_x = std_landmark[0];
  const double sigma_y = std_landmark[1];
  const double d_x = obs.x - best_landmark.x;
  const double d_y = obs.y - best_landmark.y;

  double e = (1/(2. * M_PI * sigma_x * sigma_y)) * exp(-((d_x * d_x / (2 * sigma_x * sigma_x)) + (d_y * d_y / (2 * sigma_y * sigma_y))));

  return e;
}


Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
   particle.sense_x = sense_x;
   particle.sense_y = sense_y;

   return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
  vector<double> v = best.sense_x;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
  vector<double> v = best.sense_y;
  stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
