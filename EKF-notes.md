# Handling first frame:
Kalman Filter is a recursive optimal filter for linear systems. Because of which it is necessary to address the problem of selecting a proper initial guess for the algorithm to converge.
## LIDAR:
When our first measurement is from LIDAR sensor we are certain about the position but highly uncertain about the velocity.
We can initialize the state variables as **x0 = px_measured, py_measured, vx_guess, vy_guess**.
We can initialize state covariance as shown below:
                             
                                           P0 = R_laser_(0,0), 0, 0, 0,
                                           0, R_laser(1,1), 0, 0,
                                           0, 0, 10000, 0,
                                           0, 0, 0, 10000;

## RADAR:
When our first measurement is from RADAR sensor we are certain about the range and bearing, and uncertain about the range rate. We can initialize the state variables as **x0 = range*cos(bearing), range*sin(bearing), rate*cos(bearing), rate*sin(bearing)**.
We can initialize state covariance as shown below:
                             
                                           P0 = R_radar_(0,0)+R_radar_(1,1), 0, 0, 0,
                                           0, R_radar_(0,0)+R_radar_(1,1), 0, 0,
                                           0, 0, 10000, 0,
                                           0, 0, 0, 10000;
