% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of myPoses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial myPose is given
myPose(:,1) = param.init_pose;
% You should put the given initial myPose into myPose for j=1, ignoring the j=1 ranges. 
% The myPose(:,1) should be the myPose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 20;                      % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
score = zeros(1, M);
weight = ones(1, M) * 1/M;

% Build the odometry noise model
mu = zeros(1, 3);
sig = diag([0.01, 0.01, 0.01]);

load practice-answer.mat;

figure;

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    % 1) Propagate the particles 
    noises = mvnrnd(mu, sig, M)';
    P = P + noises;
    
    % 2) Measurement Update 
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
    rays = ranges(:, j);

    %   2-2) For each particle, calculate the correlation scores of the particles
    for i = 1:M
        x = P(1, i);
        y = P(2, i);
        theta = P(3, i);

        ix_robot = ceil(x * myResolution) + myOrigin(1);
        iy_robot = ceil(y * myResolution) + myOrigin(2);

        x_occ = rays .* cos(scanAngles + theta) + x;
        y_occ = -rays .* sin(scanAngles + theta) + y;
        ix_occ = ceil(x_occ * myResolution) + myOrigin(1);
        iy_occ = ceil(y_occ * myResolution) + myOrigin(2);

        % Drop invalid values
        validPos = (ix_occ < size(map, 2)) & (iy_occ < size(map, 1)) & (ix_occ > 0) & (iy_occ > 0); 
        ix_occ = ix_occ(validPos);
        iy_occ = iy_occ(validPos);

        occ = sub2ind(size(map), iy_occ, ix_occ);

        free = [];
        for k = 1:numel(occ)
            [ix_free, iy_free] = bresenham(ix_robot, iy_robot, ix_occ(k), iy_occ(k));  
            % Drop invalid values
            validPos = (ix_free < size(map, 2)) & (iy_free < size(map, 1)) & (ix_free > 0) & (iy_free > 0);
            ix_free = ix_free(validPos);
            iy_free = iy_free(validPos);

            free = [free; iy_free, ix_free];
        end

        free = sub2ind(size(map), free(:, 1), free(:, 2));

        numGrids = numel(occ) + numel(free);
        numOcc = sum(map(occ) > 0.5);
        numFree = sum(map(free) < 0.5);
        score(i) = 10 * numOcc + 1 * numFree + (-5) * (numGrids - numOcc - numFree);
    end

    %   2-3) Update the particle weights
    weight = weight .* score;
    weight = weight ./ sum(weight);
 
    %   2-4) Choose the best particle to update the myPose
    [~, bestIndex] = max(weight);
    myPose(:, j) = P(:, bestIndex);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    num_eff = floor(sum(weight)^2 / sum(weight .^2));
    if num_eff < M * 0.5
        newP = zeros(size(myPose, 1), M);
        newW = zeros(1, M);
        resampleProbs = unifrnd(0, 1, 1, M);
        sumOfTopKWeight = cumsum(weight);
        for i = 1:M
            idx = find(sumOfTopKWeight >= resampleProbs(i), 1);
            newP(:, i) = P(:, idx);
            newM(i) = weight(idx);
        end
        P = newP;
        weight = newM;
        weight = weight ./ sum(weight);
    end

    % 4) Visualize the myPose on the map as needed
    ix_robot = ceil(x * myResolution) + myOrigin(1);
    iy_robot = ceil(y * myResolution) + myOrigin(2);
    ix_robot_true = ceil(pose(1, j) * myResolution) + myOrigin(1);
    iy_robot_true = ceil(pose(2, j) * myResolution) + myOrigin(2);

    imagesc(map); hold on;
    plot(ix_robot, iy_robot, 'r*');
    for i = 1:M % Plot all particles
        ix_robot = ceil(P(1, i) * myResolution) + myOrigin(1);
        iy_robot = ceil(P(2, i) * myResolution) + myOrigin(2);
        plot(ix_robot, iy_robot, 'y.');
    end
    plot(ix_robot_true, iy_robot_true, 'g*');
    pause(0.001);
end

end

