function [phi theta psi] = EulerKalman(A, Y)

persistent H Q R
persistent X P
persistent firstRun


%% Kalman Parameters: 

if isempty(firstRun)

  H = eye(4); % Initialize measurement matrix
  
  Q = 0.1*eye(4); % Process noise covariance matrix
  
  R = 10*eye(4); % Measurement noise covariance matrix

  X = [1 0 0 0]'; % State estimate vector
  
  P = 1*eye(4); % Process covariance matrix
  
  firstRun = 1;

end

%% Predict step:

xp = A*X;        % Predicted state estimate

Pp = A*P*A' + Q; % Predicted error covariance

%% Update step: update the predicted state estimate and error covariance based on the measurement

K = Pp*H'* inv(H*Pp*H' + R); % Compute Kalman gain

X = xp + K*(Y - H*xp); % Update state estimate matrix based on measurement

P = Pp - K*H*Pp;       % Update process covariance matrix

%% Conversion of quaternion values to Euler angles (Roll, Pitch, Yaw)

eul = quat2eul(X','XYZ'); % Convert quaternion to Euler angles using 'XYZ' rotation sequence

phi = eul(1);             % Roll angle (rotation about X-axis)

theta = eul(2);           % Pitch angle (rotation about Y-axis)

psi = eul(3);             % Yaw angle (rotation about Z-axis)
