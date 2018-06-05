#  Estimation Project Writeup

## Implementation Details ##

### Compute StdDev for GPS and Accelerometer data ###

Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

I have loaded the two generated `config/log/Graph1.txt` and `config/log/Graph2.txt` into Numbers program and used to compute the standard deviation (using STDEV function). Then set the parameters witin `07_AttitudeEstimation.txt` to the following values:

```
MeasuredStdDev_GPSPosXY = 0.717664
MeasuredStdDev_AccelXY = 0.512158
```

The calculated standard deviation correctly capturea ~68% of the sensor measurements and the tests pass.

### UpdateFromIMU ###

I have followed the details outlined in the `Estimation for Quadrotors` document. The idea is to use quaternions when performing the predict step for the attitude estimation. First I convert the estimated roll, pitch, and yaw into a quaternion and also create another quaternion that holds measured body rates from the IMU. Multiplying these two quaternions yeilds a new quaternions that represent the predicted values. Finally I use supplied quaternion methods to extract Pitch, Roll and Yaw values. then pass Pitch and Roll to the Complementary Filter as before.

```
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

auto qt = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, ekfState(6));
auto dq = Quaternion<float>::FromEuler123_RPY(dtIMU * gyro.x, dtIMU * gyro.y, dtIMU * gyro.z);
auto qt_bar = dq * qt;

auto predictedRoll = qt_bar.Roll();
auto predictedPitch = qt_bar.Pitch();
ekfState(6) = qt_bar.Yaw();

// normalize yaw to -pi .. pi
if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;
if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

/////////////////////////////// END STUDENT CODE ////////////////////////////
```
### PredictState ###
