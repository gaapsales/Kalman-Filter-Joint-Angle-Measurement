% Function to calculate Euler angles based on accelerometer and magnetometer data

function [phi, theta, psi] = EulerAccel(ax, ay, az, mx, my, mz)

theta = -atan2(ax, sqrt(ay * ay + az * az)); % Pitch angle

phi = atan2(ay, sqrt(ax * ax + az * az)); % Roll angle (phi)

psi = atan2(-my, mx); % Magnetometer Heading, Yaw angle (psi)
