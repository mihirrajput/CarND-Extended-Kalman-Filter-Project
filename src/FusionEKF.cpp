#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
	  0, 1, 0, 0;

  // measurement function Jacobian - radar
  Hj_ << 0, 0, 0, 0,
	  0, 0, 0, 0,
	  0, 0, 0, 0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		float range = measurement_pack.raw_measurements_[0];
		float bearing = measurement_pack.raw_measurements_[1];
		float px = range * cos(bearing);
		float py = range * sin(bearing);
		ekf_.x_ << px, py, 1, 1;

		// Initialize the state covariance matrix
		ekf_.P_ << 1, 0, 0, 0;
		0, 1, 0, 0;
		0, 0, 1000, 0;
		0, 0, 0, 1000;

		// Set the previous time stamp to time stamp of first measurement
		previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		float px = measurement_pack.raw_measurements_[0];
		float py = measurement_pack.raw_measurements_[1];
		ekf_.x_ << px, py, 1, 1;

		// Initialize the state covariance matrix
		ekf_.P_ << 1, 0, 0, 0;
		0, 1, 0, 0;
		0, 0, 1000, 0;
		0, 0, 0, 1000;

		// Set the previous time stamp to time stamp of first measurement
		previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // calculate delta_t
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/pow(10,6);
  previous_timestamp_ = measurement_pack.timestamp_;

  // update F with the computed delta_t
  ekf_.F_ << 1, 0, dt, 0,
	  0, 1, 0, dt,
	  0, 0, 1, 0,
	  0, 0, 0, 1;
  
  // set up the motion noise
  float noise_ax = 9, noise_ay = 9;

  // compute the complete process noise covariance matrix
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
	  0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
	  dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
	  0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	 // Radar updates
	// Update the Jacobian
	  Hj_<< tools.CalculateJacobian(ekf_.x_);
	  ekf_.H_ = Hj_;
	// Update measurement noise with radar
	  ekf_.R_ = R_radar_;
	//Update state and state covariance matrix
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
	// Update the Jacobian
	  ekf_.H_ = H_laser_;
	// Update measurement noise with radar
	  ekf_.R_ << R_laser_;
	//Update state and state covariance matrix
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
