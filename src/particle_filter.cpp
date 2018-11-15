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
#include "map.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).	

	// Number of particles
	num_particles = 100;

	// Standard deviations for x, y, and theta
	double std_x, std_y, std_theta; 
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// Normal (Gaussian) distribution for x, y, and theta
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	// Initialize
	for (int i = 0; i < num_particles; i++) {
		// Create particle
		Particle p;
		p.id = i;
		// Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS)
		p.x = x;
		p.y = y;
		p.theta = theta;
		// Set weights to 1
		p.weight = 1.0;
		//weights.push_back(p.weight);

		// Add random Gaussian noise to each particle
		p.x += dist_x(gen);
		p.y += dist_y(gen);
		p.theta += dist_theta(gen);

		//Add particle
		particles.push_back(p);
	}
	// Initialization Done
	is_initialized = true;
	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	// Standard deviations for x, y, and theta
	double std_x, std_y, std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	// Normal (Gaussian) distribution
	normal_distribution<double> dist_x(0, std_x);
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

	// Add Measurements
	for (auto& p : particles) {

		// when the yaw rate is equal to zero:
		if (fabs(yaw_rate) < 0.00001) {
			p.x += velocity * delta_t * cos(p.theta);
			p.y += velocity * delta_t * sin(p.theta);
		}
		// when the yaw rate is not equal to zero:
		else {
			p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
			p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
			p.theta += yaw_rate * delta_t;
		}

		// Add random Gaussian noise to each particle
		p.x += dist_x(gen);
		p.y += dist_y(gen);
		p.theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (auto& obs : observations) {

		// Initialize min distance to a large number
		double minDistance = numeric_limits<double>::max();

		// Initialize id of closest predicted measurement
		int closest = -1;

		// Find closest predicted measurement
		for (auto& pred : predicted) {

			// Calculate distance between observed/predicted landmarks
			double distance = dist(obs.x, obs.y, pred.x, pred.y);

			// Calculate the closest predicted landmark
			if (distance < minDistance) {
				minDistance = distance;
				closest = pred.id;
			}
		}

		// Update the observation with closest predicted landmark
		obs.id = closest;
	}
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

	
	for (auto& p : particles) {

		// Vector to store predicted landmarks within sensor range of a particle
		vector<LandmarkObs> prLandmarks;
		
		for (auto& lm : map_landmarks.landmark_list) {			

			// Filter particles within sensor range
			if (pow(lm.x_f - p.x, 2) + pow(lm.y_f - p.y, 2) <= pow(sensor_range, 2)) {

				// Add prediction
				prLandmarks.push_back(LandmarkObs{ lm.id_i, lm.x_f, lm.y_f });
			}
		}

		// Vector to store observations transformed from vehicle coordinates to map coordinates
		vector<LandmarkObs> tfObservations;
		for (auto& obs: observations) {
			double tf_x = cos(p.theta)*obs.x - sin(p.theta)*obs.y + p.x;
			double tf_y = sin(p.theta)*obs.x + cos(p.theta)*obs.y + p.y;
			tfObservations.push_back(LandmarkObs{ obs.id, tf_x, tf_y });
		}

		// Data Association for the predictions and transformed observations of the current particle
		dataAssociation(prLandmarks, tfObservations);

		// Reset weights
		p.weight = 1.0;

		// Initialize Range & Bearing
		double range = std_landmark[0];
		double bearing = std_landmark[1];

		for (auto& tfObs : tfObservations) {

			// Find the predicted obervation associated with current transformed observation
			for (auto& pr : prLandmarks) {
				if (pr.id == tfObs.id) {
					// Update the weights of each particle using a mult-variate Gaussian distribution					
					double obsWeight = (1 / (2 * M_PI*range*bearing)) * exp(-(pow(pr.x - tfObs.x, 2) / (2 * pow(range, 2)) + (pow(pr.y - tfObs.y, 2) / (2 * pow(bearing, 2)))));

					// Update weight by multiplying obersvation weight with total observations weight
					p.weight *= obsWeight;

					break;
				}
			}	
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution	

	// Vector to store all weights
	vector<double> weightsAll;
	for (auto& p : particles) {
		weightsAll.push_back(p.weight);
	}

	// Uniform Random Distribution for particles
	uniform_int_distribution<int> intDist(0, num_particles - 1);

	// Generate Starting index for resampling wheel
	auto index = intDist(gen);

	// Calculate max weight
	double maxWeight  = *max_element(weightsAll.begin(), weightsAll.end());

	// Uniform Random Distribution for weights
	uniform_real_distribution<double> realDist(0.0, maxWeight);

	double beta = 0.0;

	// Vector to store resampled particles
	vector<Particle> resampledParticles;

	// Generate resampled particles by spinning the wheel
	for (int i = 0; i < num_particles; i++) {
		beta += realDist(gen) * 2.0;
		while (beta > weightsAll[index]) {
			beta -= weightsAll[index];
			index = (index + 1) % num_particles;
		}
		resampledParticles.push_back(particles[index]);
	}

	particles = resampledParticles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

    particle.associations = associations;
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
