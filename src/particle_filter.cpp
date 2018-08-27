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

	num_particles = 300;
	particles = std::vector<Particle>(num_particles);

	default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta

	// TODO: Set standard deviations for x, y, and theta.
	std_x = std[0];
	std_y = std[1];
	std_theta = std[2];

	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std_x);

	// TODO: Create normal distributions for y and theta.
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);

	for (int i = 0; i < ParticleFilter::num_particles; ++i) {
		// TODO: Sample  and from these normal distrubtions like this: 
		// sample_x = dist_x(gen);
		// where "gen" is the random engine initialized earlier.
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1;

	}

	cout << "Sample " << particles[0].x << endl;
	ParticleFilter::is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[],
		double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
	double x = 0.0, y = 0.0 ; // Standard deviations for x, y
	double theta_t0 = 0.0, theta_t1 = 0.0 ;
	// TODO: Set standard deviations for x, y, and theta.
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta_t1, std_theta);

	if (yaw_rate == 0.0){
		return;
	}

	double v_yaw_rate = velocity / yaw_rate;
	double dt_yaw_rate = delta_t * yaw_rate;

	for (int i = 0; i < ParticleFilter::num_particles; ++i) {

		theta_t0 = particles[i].theta ;
		theta_t1 = theta_t0 + dt_yaw_rate;

		x = particles[i].x + v_yaw_rate * (sin(theta_t1) - sin(theta_t0));
		y = particles[i].y + v_yaw_rate * (cos(theta_t0) - cos(theta_t1));

		normal_distribution<double> dist_x(x, std_x);
		normal_distribution<double> dist_y(y, std_y);
		normal_distribution<double> dist_theta(theta_t1, std_theta);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);

	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted,
		std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
		const std::vector<LandmarkObs> &observations,
		const Map &map_landmarks) {
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

	int observations_size = observations.size();
	vector<int> associations(observations_size);
	vector<double> sense_x(observations_size);
	vector<double> sense_y(observations_size);

	double x_part = 0;
	double y_part = 0;
	double x_obs = 0;
	double y_obs = 0;
	double theta = 0;

	double sig_x = std_landmark[0]; //std_landmark_x
	double sig_y = std_landmark[1]; //std_landmark_y
	double x_map = 0; 
	double y_map = 0; 
	double mu_x = 0;
	double mu_y = 0;

	// calculate normalization term
	double gauss_norm =  0;

	// calculate exponent
	double exponent= 0;

	// calculate weight using normalization terms and exponent
	double weight = 0;

	double weight_all = 1;

	for (int i = 0; i < ParticleFilter::num_particles; ++i) {

		associations.clear();
		sense_x.clear();
		sense_y.clear();

		// calculate normalization term
		gauss_norm =  0;

		// calculate exponent
		exponent= 0;

		// calculate weight using normalization terms and exponent
		weight = 0;

		// the total weight
		weight_all = 0;

		x_part = particles[i].x;
		y_part = particles[i].y;
		theta = particles[i].theta;

		for(int j = 0 ; j < observations_size; j++){

			x_obs = observations[j].x;
			y_obs = observations[j].y;

			// transform to map x coordinate
			x_map= x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);

			//  transform to map y coordinate
			y_map= y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);

			double mindis_to_l = std::numeric_limits<double>::infinity();
			int min_l =0;
			int mu_id = 0;

			for(int l = 0; l < int(map_landmarks.landmark_list.size()); l++){
				mu_x = map_landmarks.landmark_list[l].x_f;
				mu_y = map_landmarks.landmark_list[l].y_f;

				double dis_m_l = sqrt ( (pow(x_map - mu_x, 2) + pow(y_map - mu_y, 2)) );
				if  ( (dis_m_l < mindis_to_l) && (dis_m_l <= sensor_range) ){
					
					mindis_to_l = dis_m_l;
					min_l = l;
				} 

			}

			mu_x = map_landmarks.landmark_list[min_l].x_f;
			mu_y = map_landmarks.landmark_list[min_l].y_f;
			mu_id = map_landmarks.landmark_list[min_l].id_i;

			// calculate normalization term
			gauss_norm = (1 / (2 * M_PI * sig_x * sig_y));

			// calculate exponent
			exponent= ( pow((x_map - mu_x), 2) )/(2 * pow(sig_x, 2) ) + (pow((y_map - mu_y), 2))/(2 * pow(sig_y, 2) );

			// calculate weight using normalization terms and exponent
			weight= gauss_norm * exp(-exponent);

			if(weight_all == 0){
				
				weight_all = weight;

			}
			else{

				weight_all *= weight;

			}

			sense_x.push_back(x_map);
			sense_y.push_back(y_map);
			associations.push_back(mu_id);

		}

                particles[i].weight = weight_all; 
		ParticleFilter::SetAssociations(particles[i], associations, sense_x, sense_y);

	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<Particle> temp_particles(num_particles);
 	temp_particles = particles;
	particles.clear();

	double sum_w = 0;
	double step = 0;

 	for (int i = 0; i < ParticleFilter::num_particles; ++i) {
		sum_w += temp_particles[i].weight; 
                if (step < temp_particles[i].weight){
			step = temp_particles[i].weight;
		}

	}

 	for (int i = 0; i < ParticleFilter::num_particles; ++i) {
		temp_particles[i].weight =temp_particles[i].weight / sum_w; 
	}

	step = step / sum_w;
	double sum_temp = 0;
	int x = 1;
        sum_w = 0;

	for (int i = 0; i < ParticleFilter::num_particles; ++i) {

		while (sum_temp < step){
			x += 1;
			sum_temp += temp_particles[(x - 1) % ParticleFilter::num_particles ].weight;

		}

		particles.push_back(temp_particles[(x - 1) % ParticleFilter::num_particles ]);
                sum_temp -= step;

	}

}

void ParticleFilter::SetAssociations(Particle& particle,
	const std::vector<int>& associations, const std::vector<double>& sense_x,
	const std::vector<double>& sense_y) {
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	particle.associations = associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

}

string ParticleFilter::getAssociations(Particle best) {
	vector<int> v = best.associations;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}

string ParticleFilter::getSenseX(Particle best) {
	vector<double> v = best.sense_x;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}

string ParticleFilter::getSenseY(Particle best) {
	vector<double> v = best.sense_y;
	stringstream ss;
	copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length() - 1);  // get rid of the trailing space
	return s;
}

