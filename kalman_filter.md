
# Kalman Filter Simulation
## Min Young Chang

[//]: # (Image References)
[image1]: ./kf_images/writeup_img1.png
[image2]: ./kf_images/writeup_img2.png
[image3]: ./kf_images/writeup_img3.png
[image4]: ./kf_images/writeup_img4.png
[image5]: ./kf_images/writeup_img5.png
[image6]: ./kf_images/writeup_img6.png
[image7]: ./kf_images/writeup_img7.png
[image8]: ./kf_images/writeup_img8.png
[image9]: ./kf_images/writeup_img9.png
[image10]: ./kf_images/writeup_img10.png
[image11]: ./kf_images/writeup_img11.png
[image12]: ./kf_images/writeup_img12.png

---
MAIN FILES <br>
`main.cpp`: main running file<br>
`measurement_package.h`: class for processed data input<br>
`Tools.cpp`: includes functions calculating Root Mean Square Errors and a Jacobian Matrix<br>
`Tools.h`: header file of Tools.cpp<br>
`kalman_filter.cpp`: class and functions for general kalman filter calculation<br>
`kalman_filter.h`: header file of kalman_filter.cpp<br>
`Fusion.cpp`: class and functions that integrates the kalman filter calculation of LIDAR input data and RADAR input data<br>
`Fusion.h`: header file of Fusion.cpp<br>
`EIGEN/Dense` contains important classes like VectorXd and MatrixXd

PLOT FILES <br>
`data/obj_pose-laser-radar-synthetic-input.txt`: input data file, which I attained from Udacity github<br>
`data/output_rmse`: root mean square error for each step<br>
`data/output_radar`: radar measurements<br>
`data/output_lidar`: lidar measurements<br>
`data/output_predict`: predictions for each step<br>
`data/output_kf`: kalman filter - measurement update for each step<br>
`data/output_gt`: ground truth for rmse calculation<br>
<br>
`kf_live_plot.py`: reads in the output text files and live-plot the points


---
## 1. Simulation Setup
This is a simulation for Kalman Filter that tracks the position and velocity of a moving bike. Based on the sensor measurement received from LIDAR and RADAR, I calculated the "believed position and velocity" of the bike with Kalman Filter, and compared the result with corresponding ground truth by calculating root mean square errors. 

`obj_pose-laser-radar-synthetic-input.txt` contains data as following:<br><br>
LIDAR measurement "L" (9 columns):<br>
 | sensor_type | x_measured | y_measured | timestamp | x_gt | y_gt | vx_gt | vy_gt | yaw_gt | yawrate_gt |  <br>
<br>
RADAR measurement "R" (10 columns):<br>
 | sensor_type | rho_measured | phi_measured | rhodot_measured | timestamp | x_gt | y_gt | vx_gt | vy_gt | yaw_gt | yawrate_gt | <br>

![alt text][image1]

## 2. Kalman Filter Explanation
Kalman filter is mainly comprised of 2 steps: predict and update. Prediction is done based on the most recent state of the object, which is the bike in this simulation. For example, if a bike was at x = 0 m, and it was moving at 1 me/s speed, we can easily predict that the bike will be at x = 1 m after one second. Then, the system receives the measurement from sensors(LIDAR and RADAR in this project), integrate the measurement with the predicted state from the first step, and update the position of the bike. The system repeats these two steps and tracks the position and velocity of the bike.

#### Legend
![alt text][image2]
### Step 1: Predict

![alt text][image3]
In this step, we have to predict the next position of the bike based on the current state. As you can tell from the trajectory(black line), the bike is moving to the right. There for, it is not too difficult to predict that the bike will move right for some distance. Also, we know that the velocity of the bike will not change very much in a short period of time (about 0.05 seconds here), so we predict the velocity to be about the same as the previous time step. Based on this information, we predict the next position of the bike to be this: 
![alt text][image4]

This prediction for next state can be easily calculated with a simple matrix multiplication. For example, px' = px + dx*vx, and vx' = vx.
![alt text][image8]

Here, we define the state transition matrix 'F', which was multiplied to the state matrix for prediction, as following:
![alt text][image9]

This matrix can also be used for state covariance update. If you would like to know more about "covariance" or any mathematical proof for this, look up "Gaussian Distribution for Kalman Filter".
![alt text][image10]


### Step 2: Update (LIDAR)
Now, we need to calculate the position of the bike based on the received sensor data. In this simulation, it is either a LIDAR data or a RADAR data. 
![alt text][image5]
This time, we received a LIDAR measurement (yellow x-mark), which seems to be very accurate given that it is almost right on the current position of the bike (big red dot). Since we received a measurement, we need to update the position, which is calculated based on the most recently predicted position(yellow dot) and the sensor measurement(yellow x-mark).
![alt text][image6]
The updated position is shown with a green dot.<br><br>

Here are constant matrices for sensore measurement update calculation. Matrix 'R' is a measurement noise (or covariance) matrix, which reflects the possible noise of the sensor measurements. These values are usually provided by sensor manufacturers. Matrix 'H' is a measurement function matrix for measurement update calculation. 
![alt text][image11]

The reason we have different matrices for lidar and radar are because the measurement output for each sensor is different as discussed under "Simulation Setup" section. Following is the math done for calculation:

![alt text][image12]

Here, matrix 'z' is a measurement matrix, matrix 'y' is called error matrix, and matrix 'K' is called Kalman gain. 


### Step nX: Repeats Step 1 and Step 2
The system repeats step 1 and step 2 until the simulation ends. The only thing that changes is that the type of measurement we receive varies: sometimes it is a LIDAR measurement, and somethimes it is a RADAR measurement. The math calculation for the two types of measurement are different, becausre LIDAR measurement outputs the position and velocity in cartesian coordinates while RADAR measurement outpus those in polar coordinates. This different methods are processed in `Fusion.cpp`.

## 3. Root Mean Square Error
In order to figure out how accurate the system is working, I calculated the "Root Mean Square Error" for each time step. The equation of this error is:
![alt text][image7]
