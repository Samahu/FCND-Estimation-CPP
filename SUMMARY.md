#  Estimation Project Writeup

## Step 1: Sensor Noise ##

### Compute StdDev for GPS and Accelerometer data ###

Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

I have loaded the two generated `config/log/Graph1.txt` and `config/log/Graph2.txt` into Numbers program and used to compute the standard deviation (using STDEV function). Then set the parameters witin `07_AttitudeEstimation.txt` to the following values:

```
MeasuredStdDev_GPSPosXY = 0.717664
MeasuredStdDev_AccelXY = 0.512158
```

The calculated standard deviation correctly capturea ~68% of the sensor measurements and the tests pass.

## Step 2: Attitude Estimation ##

### UpdateFromIMU Implementation ###

I have followed the details outlined in the `Estimation for Quadrotors` document. The idea is to use quaternions when performing the predict step for the attitude estimation. First I convert the estimated roll, pitch, and yaw into a quaternion and also create another quaternion that holds measured body rates from the IMU. Multiplying these two quaternions yeilds a new quaternions that represent the predicted values. Finally I use supplied quaternion methods to extract Pitch, Roll and Yaw values. then pass Pitch and Roll to the Complementary Filter as before.

```C++
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

## Step 3: Prediction Step ##

### PredictState Implementation ###


I have implemented the prediction step as indicated below. I construct two terms A and B. Term A is straighforward and mainly updates the position. Term B defines the velocity update values. I use the supplied quaternion to transform the control/acceleration vector from Body Frame to Intertial Frame. The sum of A and B represents the predictedState.

In additon to the above I made the same implementation by directly incorporating the rotation matrix (to compare results) but I have omitted this part out from the writup.

```C++
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  VectorXf A(QUAD_EKF_NUM_STATES); // A Term
  VectorXf B(QUAD_EKF_NUM_STATES); // B Term

  A(0) = curState(0) + curState(3) * dt;
  A(1) = curState(1) + curState(4) * dt;
  A(2) = curState(2) + curState(5) * dt;
  A(3) = curState(3);
  A(4) = curState(4);
  A(5) = curState(5) - float(CONST_GRAVITY) * dt;
  A(6) = curState(6);

#if true  // USE SUPPLIED QUATERNION TO SOLVE

  auto accelG = attitude.Rotate_BtoI(accel);
  B.setZero();
  B(3) = accelG.x * dt;
  B(4) = accelG.y * dt;
  B(5) = accelG.z * dt;

#else   // CONSTRUCT ROTATION MATRIX ACCORDING TO NOTES
  ...
  ...
#endif

  predictedState = A + B;

/////////////////////////////// END STUDENT CODE ////////////////////////////
```
### RbgPrime Implementation ###

A correct calculation of the Rgb prime matrix.

```C++
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  auto θ = pitch, φ = roll, ψ = yaw;
  RbgPrime(0, 0) = - cos(θ) * sin(ψ);
  RbgPrime(0, 1) = - sin(φ) * sin(θ) * sin(ψ) - cos(φ) * cos(ψ);
  RbgPrime(0, 2) = - cos(φ) * sin(θ) * sin(ψ) + sin(φ) * cos(ψ);
  RbgPrime(1, 0) = cos(θ) * cos(ψ);
  RbgPrime(1, 1) = sin(φ) * sin(θ) * cos(ψ) - cos(φ) * sin(ψ);
  RbgPrime(1, 2) = cos(φ) * sin(θ) * cos(ψ) + sin(φ) * sin(ψ);

  // Last row is all zeros which is already set

/////////////////////////////// END STUDENT CODE ////////////////////////////
```

### Predict Implementation ###

The following part uses RbgPrime and control input (acceleration) to first construct gPrime Jacobian matrix.
Using the result it updates the state covariance according to the classic EKF update equation.

```C++
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  VectorXf u_t(3);
  u_t(0) = accel.x;
  u_t(1) = accel.y;
  u_t(2) = accel.z;
  auto j = RbgPrime * u_t * dt;

  gPrime(0, 3) = dt;
  gPrime(1, 4) = dt;
  gPrime(2, 5) = dt;
  gPrime(3, 6) = j(0);
  gPrime(4, 6) = j(1);
  gPrime(5, 6) = j(2);

  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;

/////////////////////////////// END STUDENT CODE ////////////////////////////
```

### Tuning Covariance Update ###
QPosXYStd = 0.05  // left unchanged
QVelXYStd = .15     // original value was .05


## Step 4: Magnetometer Update ##

### Tuning QYawStd ###
QYawStd = .1  // original value was .05

### UpdateFromMag Implementation ###

hPrime is basically [0 0 0 0 0 0 1] while zFromeX can be obtained from ekfState(6).
I also address the cases when one of the measurements switches sign when it exceeds 180 degrees either way. 

```C++
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  hPrime(6) = 1;  // the rest are zeros

  zFromX(0) = ekfState(6);

  // printf("measured: %f, estimated: %f\n", magYaw, ekfState(6));

  // Normailze the error between z, zFromX
  const auto epsilon = F_PI;
  auto error = z - zFromX;

  if (fabsf(error(0)) > epsilon)
  {
    // Normailzation Required
    if (z(0) < 0 && zFromX(0) > 0)
      z(0) += 2 * F_PI;

    if (zFromX(0) < 0 && z(0) > 0)
      zFromX(0) += 2 * F_PI;
  }

/////////////////////////////// END STUDENT CODE ////////////////////////////
```

## Step 5: Closed Loop + GPS Update ##

First thing I followed instructions to disable the Quad Ideal Estimator (by setting `Quad.UseIdealEstimator` to zero) and also commented out `SimIMU.AccelStd` and `SimIMU.GyroStd`

### UpdateFromGPS Implementation ###

Pretty straight forward, I construct hPrime and then copy all ekfState values (except yaw) to the zFromX vector.

```C++
////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  for (auto i = 0; i < 6; ++i)
    hPrime(i, i) = 1;

  for (auto i = 0; i < 6; ++i)
    zFromX(i) = ekfState(i);

/////////////////////////////// END STUDENT CODE ////////////////////////////
```
For the tuning part I only increased `QPosZStd` from .05 to .1 which seem to yeild slightly better results.


## Step 6: Adding Your Controller ##

I have replaced both QuadControl.cpp and QuadControlParams.txt with the versions I had from previous project.
Runing scenario 11 it still passes the criteria.

### Tuning ###
I have reduced all previous settings by 30% from their previous values.

Old Values:
```
# Position control gains
kpPosXY = 2.5
kpPosZ = 2.5
KiPosZ = 50

# Velocity control gains
kpVelXY = 11
kpVelZ = 8

# Angle control gains
kpBank = 12
kpYaw = 3

# Angle rate gains
kpPQR = 75, 75, 15
```

New Values
```
# Position control gains
kpPosXY = 1.8
kpPosZ = 1.8
KiPosZ = 35

# Velocity control gains
kpVelXY = 8
kpVelZ = 5.5

# Angle control gains
kpBank = 9
kpYaw = 2

# Angle rate gains
kpPQR = 50, 50, 10
```

The drone performs as expected with scenario 11 shows a passing states.
