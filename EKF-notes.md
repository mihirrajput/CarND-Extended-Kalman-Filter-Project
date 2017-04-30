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
When our first measurement is from RADAR sensor we are certain about the range and bearing, and uncertain about the range rate. We can initialize the state variables as **x0 = range x cos(bearing), range x sin(bearing), rate x cos(bearing), rate x sin(bearing)**.
We can initialize state covariance as shown below. 'R_radar_(0,0)+R_radar_(1,1)' shows additive nature of unreliability of two independent entities. '10000' signifies the high uncertainty in range rate measurement.
                             
                                           P0 = R_radar_(0,0)+R_radar_(1,1), 0, 0, 0,
                                           0, R_radar_(0,0)+R_radar_(1,1), 0, 0,
                                           0, 0, 10000, 0,
                                           0, 0, 0, 10000;
# Handling division by 0 in Jacobian:
The individual terms of the Jacobian matrix for **this** specific problem have denominator as a power of px*px + py*py. If absolute value of px*px + py*py is close to zero, I simple use the previous 'good Jacobian'. I had learnt about a similar technique called **'Dishonest' Newton Raphson** for Power Flow analysis where the Jacobian is computed occasionally or in extreme cases only once.

# Visualization:
LIDAR gives us good resolution. From EKF-LIDAR image we can see that we have a pretty good idea about the position of the object but no idea about velocity. This is where RADAR comes in. Despite the errors in measurement we do get some sense of velocity and position. Combining both RADAR and LIDAR measurements as shown in EKF-RADAR-LIDAR image we can get a good RMSE value for our estimated state vs ground reality.
![EKF-LIDAR](https://github.com/mihirrajput/CarND-Extended-Kalman-Filter-Project/blob/master/EKF-LIDAR.png)
![EKF-RADAR](https://github.com/mihirrajput/CarND-Extended-Kalman-Filter-Project/blob/master/EKF-RADAR.png)
![EKF-RADAR-LIDAR](https://github.com/mihirrajput/CarND-Extended-Kalman-Filter-Project/blob/master/EKF-RADAR-LIDAR.jpg)
