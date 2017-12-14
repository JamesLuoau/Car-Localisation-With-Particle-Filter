/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
//  cout << "Init Particle Filter with x " << x << " y " << y << " theta " << theta << " std " << std[0] << " / " << std[1] << " / " << std[2] << endl;

  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];

  default_random_engine gen;
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  num_particles = 200;
//  weights.resize(num_particles, 1.0f);

  for (int i = 0; i < num_particles; ++i) {
    double sample_x, sample_y, sample_theta;

    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);

    Particle p;
    p.id = i;
    p.x = sample_x;
    p.y = sample_y;
    p.theta = sample_theta;
    p.weight = 1.0f;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

//  cout << "Prediction, delta_t " << delta_t << " std_pos[]" << " velocity " << velocity << " yaw_rate " << yaw_rate << endl;
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];

  default_random_engine gen;
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);

  for (int i = 0; i < num_particles; i++) {
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_yaw = particles[i].theta;

    if (fabs(yaw_rate) < 0.001) {
      particles[i].x = particle_x + velocity * delta_t * cos(particle_yaw);
      particles[i].y = particle_y + velocity * delta_t * sin(particle_yaw);
    }
    else {
      particles[i].x = particle_x + velocity / yaw_rate * (sin(particle_yaw + yaw_rate*delta_t) - sin(particle_yaw));
      particles[i].y = particle_y + velocity / yaw_rate * (cos(particle_yaw) - cos(particle_yaw + yaw_rate*delta_t));
      particles[i].theta = particle_yaw + yaw_rate * delta_t;
    }

    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

int findClosestMapId(double x, double y, std::vector<LandmarkObs> landmarksFromMap) {
  int map_id = -1;
  double last_min_dist = numeric_limits<double>::max();

  for (unsigned int i = 0; i < landmarksFromMap.size(); i++) {
    LandmarkObs landmarkFromMap = landmarksFromMap[i];
    double cur_dist = dist(x, y, landmarkFromMap.x, landmarkFromMap.y);

    if (cur_dist < last_min_dist) {
      last_min_dist = cur_dist;
      map_id = landmarkFromMap.id;
    }
  }

  return map_id;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  for (int i = 0; i < observations.size(); i++) {
    double x = observations[i].x;
    double y = observations[i].y;
    observations[i].id = findClosestMapId(x, y, predicted);
  }
}

vector<LandmarkObs> landmarksInRange(double particle_x, double particle_y, double sensor_range, const Map &map_landmarks) {

  vector<LandmarkObs> marksInRange;

  for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
    float landmarks_x = map_landmarks.landmark_list[j].x_f;
    float landmarks_y = map_landmarks.landmark_list[j].y_f;
    int landmarks_id = map_landmarks.landmark_list[j].id_i;

    if (fabs(landmarks_x - particle_x) <= sensor_range && fabs(landmarks_y - particle_y) <= sensor_range) {
      marksInRange.push_back(LandmarkObs{ landmarks_id, landmarks_x, landmarks_y });
    }
  }

  return marksInRange;
}

vector<LandmarkObs> transformFromVehicleToMapCoordination(double particle_x, double particle_y, double particle_theta,
                                                          const std::vector<LandmarkObs> &observations) {
  vector<LandmarkObs> result;
  for (unsigned int i = 0; i < observations.size(); i++) {
    LandmarkObs observation = observations[i];
    double map_x = cos(particle_theta)*observation.x - sin(particle_theta)*observation.y + particle_x;
    double map_y = sin(particle_theta)*observation.x + cos(particle_theta)*observation.y + particle_y;
    result.push_back(LandmarkObs{observation.id, map_x, map_y });
  }
  return result;
}

LandmarkObs findLandmarkById(vector<LandmarkObs> landmarks, int id) {
  for (unsigned int k = 0; k < landmarks.size(); k++) {
    if (landmarks[k].id == id) {
      return landmarks[k];
    }
  }

  throw "Index out of range";
}

double normal_pdf_2d(LandmarkObs landmarkFromMap, LandmarkObs landmarkObservation, double std_landmark[]) {
  double std_map_x = std_landmark[0];
  double std_map_y = std_landmark[1];

  double observe_x = landmarkObservation.x;
  double observe_y = landmarkObservation.y;
  double map_mark_x = landmarkFromMap.x;
  double map_mark_y = landmarkFromMap.y;

  //The normal pdf of 1d is y=f(x;mu,std)= 1/(std*sqrt(2pi)) e[ -(x−mu)^2 / 2*std^2 ]
  auto normalizer = 2.0 * M_PI * std_map_x * std_map_y;
  auto delta_x = map_mark_x - observe_x;
  auto cov_x = std_map_x * std_map_x;
  auto delta_y = map_mark_y - observe_y;
  auto cov_y = std_map_y * std_map_y;

  double weight = ( 1/normalizer) * exp(-((delta_x * delta_x)/(2 * cov_x) + (delta_y * delta_y/(2 * cov_y))));

  return weight;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  for (int i = 0; i < num_particles; i++) {
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

    vector<LandmarkObs> landmarksFromMap = landmarksInRange(particle_x, particle_y, sensor_range, map_landmarks);
    vector<LandmarkObs> landmarksFromObservation = transformFromVehicleToMapCoordination(particle_x, particle_y, particle_theta, observations);

    dataAssociation(landmarksFromMap, landmarksFromObservation);

    particles[i].weight = 1.0;

    for (LandmarkObs landmarkObservation : landmarksFromObservation) {
      LandmarkObs landmarkFromMap = findLandmarkById(landmarksFromMap, landmarkObservation.id);
      double weight = normal_pdf_2d(landmarkFromMap, landmarkObservation, std_landmark);
      particles[i].weight *= weight;
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  vector<Particle> new_particles;

  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
  }

  default_random_engine gen;
  discrete_distribution<int> index(weights.begin(), weights.end());

  for (int c = 0; c < num_particles; c++) {
    const int i = index(gen);
    new_particles.push_back(particles[i]);
  }

  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
