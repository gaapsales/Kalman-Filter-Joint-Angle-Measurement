close all; clear all; clc

%% Import XYZ Sensor Data

imudata = readmatrix('.csv');

%% Sensor Data
acc = imudata(:, 2:4);
accx = imudata(:, 2);
accy = imudata(:, 3);
accz = imudata(:, 4);

gyro = imudata(:, 5:7);
gyx = imudata(:, 5);
gyy = imudata(:, 6);
gyz = imudata(:, 7);

mag = imudata(:, 8:10);
magx = imudata(:, 8);
magy = imudata(:, 9);
magz = imudata(:, 10);


imu_roll = imudata(:, 15);
imu_pitch = imudata(:, 16);
imu_yaw = imudata(:, 17);

ard_pitch = imudata(:, 18);
ard_roll = imudata(:, 19);

timexyz = imudata(:, 24) / 1000;

% Reset time to start from zero
initial_time = timexyz(1);
timexyz = timexyz - initial_time;

%% Kalman Filter

kalm_rpy = zeros(length(timexyz), 3);  % Preallocate array to save Euler angles

for t = 1:length(timexyz) % Calculate time step
    if t > 1
        dt = (timexyz(t) - timexyz(t-1));
    else
        dt = (timexyz(2) - timexyz(1));
    end
    
    % Gyro XYZ -> P, Q, R
    p = gyx(t);  % Angular velocity around x-axis (roll rate)
    q = gyy(t);  % Angular velocity around y-axis (pitch rate)
    r = gyz(t);  % Angular velocity around z-axis (yaw rate)
    
    % Discrete-time state transition matrix
    A = eye(4) + dt * [ 0  -p  -q  -r;
                        p   0   r  -q;
                        q  -r   0   p;
                        r   q  -p   0 ];

    

    % Estimate Euler angles from accelerometer and magnetometer data
    [phi, theta, psi] = EulerAccel(accx(t), accy(t), accz(t), magx(t), magy(t), magz(t));  
    
    % Convert Euler angles to quaternion representation
    Y = eul2quat([phi theta psi], 'XYZ')';  % Convert Euler angles to quaternion
    
    % Perform Kalman filtering to estimate Euler angles
    [phi, theta, psi] = EulerKalman(A, Y);  % Estimate Euler angles using Kalman filter

    % Save the estimated Euler angles
    kalm_rpy(t, :) = [ phi theta psi ];
end

% Convert Euler angles from radians to degrees
kalm_roll   = kalm_rpy(:, 1) * 180/pi;
kalm_pitch = kalm_rpy(:, 2) * 180/pi;
kalm_yaw   = kalm_rpy(:, 3) * 180/pi;

%% Plot Filters RPY
fig1 = figure('WindowState', 'maximized');

subplot(3,1,1);
plot(timexyz, kalm_roll, 'LineWidth', 1.5);
hold on;
plot(timexyz, imu_roll, 'LineWidth', 1.5);
hold on;
xlabel('Time [s]');
ylabel('Angle [Degrees]');
title('Roll');
legend('Kalman', 'Unfiltered');
grid minor;
axis tight;

subplot(3,1,2);
plot(timexyz, kalm_pitch, 'LineWidth', 1.5);
hold on;
plot(timexyz, imu_pitch, 'LineWidth', 1.5);
hold on;
xlabel('Time [s]');
ylabel('Angle [Degrees]');
title('Pitch');
legend('Kalman', 'Unfiltered');
grid minor;
axis tight;

subplot(3,1,3);
plot(timexyz, kalm_yaw, 'LineWidth', 1.5);
hold on;
plot(timexyz, imu_yaw, 'LineWidth', 1.5);
hold on;
xlabel('Time [s]');
ylabel('Angle [Degrees]');
title('Yaw');
legend('Kalman', 'Unfiltered');
grid minor;
axis tight;

%% Plot XYZ Data
fig2 = figure('WindowState', 'maximized');

subplot(3,1,1);
hold on;
plot(timexyz, accx, 'LineWidth', 1.5);
plot(timexyz, accy, 'LineWidth', 1.5);
plot(timexyz, accz, 'LineWidth', 1.5);
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Accelerometer');
axis tight;
grid minor;
hold off;

subplot(3,1,2);
hold on;
plot(timexyz, gyx, 'LineWidth', 1.5);
plot(timexyz, gyy, 'LineWidth', 1.5);
plot(timexyz, gyz, 'LineWidth', 1.5);
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (Radians)');
title('Gyroscope');
axis tight;
grid minor;
hold off;

subplot(3,1,3);
hold on;
plot(timexyz, magx, 'LineWidth', 1.5);
plot(timexyz, magy, 'LineWidth', 1.5);
plot(timexyz, magz, 'LineWidth', 1.5);
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (uT)');
title('Magnetometer');
axis tight;
grid minor;
hold off;