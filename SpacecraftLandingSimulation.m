% Lunar Surface Simulation and Spacecraft Landing with Kalman Filter

% Parameters
gravity = 1.625; % Lunar gravity in m/s^2
dt = 0.1; % Time step in seconds
simulation_time = 100; % Total simulation time in seconds
initial_altitude = 1000; % Initial altitude in meters
landing_zone_radius = 500; % Radius of the landing zone in meters

% Generate simulated lunar surface data
[X, Y] = meshgrid(linspace(-landing_zone_radius, landing_zone_radius, 100));
Z = 50 * exp(-(X.^2 + Y.^2) / (2 * landing_zone_radius^2)); % Example surface profile

% Find the best landing zone using a statistical method (you can customize this)
landing_zone_mean = mean(Z(:));
landing_zone_std = std(Z(:));

% Identify the landing zone as the area with the lowest altitude
landing_zone = Z < landing_zone_mean - 0.5 * landing_zone_std;

% Initialize spacecraft state
state = struct('position', [0; 0; initial_altitude], 'velocity', [0; 0; 0]);

% Kalman Filter parameters
A = [1, dt, 0.5*dt^2; 0, 1, dt; 0, 0, 1]; % State transition matrix
H = [1, 0, 0]; % Measurement matrix
Q = 0.01 * eye(3); % Process noise covariance
R = 0.1; % Measurement noise covariance

% Initial state estimate and covariance matrix
x = [0; 0; initial_altitude];
P = eye(3);

% Initialize Kalman Filter
kf = struct('A', A, 'H', H, 'Q', Q, 'R', R, 'x', x, 'P', P);

% Create figure for 3D visualization
figure;
ax = axes;
surf(X, Y, Z, 'EdgeColor', 'none');
hold on;
plot3(0, 0, landing_zone_mean, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Mark the best landing zone
title('Spacecraft Landing Simulation');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
view(3);
axis equal;

% Animation loop
for t = 0:dt:simulation_time
    % Simulate spacecraft motion
    state.velocity(3) = state.velocity(3) - gravity * dt; % Gravity effect
    state.position = A * state.position + sqrt(Q) * randn(3, 1); % Process noise

    % Kalman Filter prediction
    kf.x = A * kf.x;
    kf.P = A * kf.P * A' + Q;

    % Kalman Filter correction (update with simulated altitude measurement)
    measurement = state.position(3) + sqrt(R) * randn;
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (measurement - H * kf.x);
    kf.P = (eye(3) - K * H) * kf.P;

    % Update spacecraft position based on Kalman Filter estimate
    state.position = kf.x;


    % Update spacecraft trajectory in the plot
    plot3(state.position(1), state.position(2), state.position(3), 'b.', 'MarkerSize', 5);

    % Update landing zone position in the plot
    plot3(0, 0, landing_zone_mean, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Pause for animation
    pause(0.05);
end
clc
hold off;
