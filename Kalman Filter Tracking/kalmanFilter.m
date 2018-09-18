function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    if previous_t < 0
        state = [x; y; 0; 0];
        param.P = 2 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %%% TODO: Add Kalman filter updates
    %% As an example, here is a Naive estimate without a Kalman filter
    %% You should replace this code
    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    %% Predict 330ms into the future
    %predictx = x + vx * 0.330;
    %predicty = y + vy * 0.330;
    %% State is a four dimensional element
    %state = [x, y, vx, vy];

    dt = t - previous_t;

    A = [1, 0, dt, 0;
         0, 1, 0, dt;
         0, 0, 1, 0;
         0, 0, 0, 1];

    C = [1, 0, 0, 0;
         0, 1, 0, 0];

    % Error of motion and oservation
    omega_m = [dt*dt/4, 0, dt/2, 0;
               0, dt*dt/4, 0, dt/2;
               dt/2, 0, 1, 0;
               0, dt/2, 0, 1];

    omega_o = [0.01, 0;
               0, 0.01];

    P = A * param.P * A' + omega_m;
    R = omega_o;

    zt = [x; y];

    % Computer the kalman gain
    K = P * C' * inv(R + C * P * C');
    % Correction
    state = A * state + K * (zt - C * A * state);
    param.P = P - K * C * P;
    predictx = state(1) + state(3) * 0.330;
    predicty = state(2) + state(4) * 0.330;
end
