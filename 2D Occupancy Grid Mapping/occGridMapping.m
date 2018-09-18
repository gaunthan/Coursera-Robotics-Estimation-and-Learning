% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial myMap size in pixels
myMap = zeros(param.size);
% the origin of the myMap in pixels
myOrigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

%figure,
%imagesc(myMap); hold on;
%plot(myOrigin(1), myOrigin(2), 'rx', 'LineWidth', 3); % indicate start point
%axis equal;

N = size(pose,2);
numScans = size(scanAngles);
for j = 1:N % for each time,
    x = pose(1, j);
    y = pose(2, j);
    theta = pose(3, j);

    ix_robot = ceil(x * myResol) + myOrigin(1);
    iy_robot = ceil(y * myResol) + myOrigin(2);
    %disp([ix_robot, iy_robot]);

    % Find grids hit by the rays (in the gird myMap coordinate)
    rays = ranges(:, j);
    x_occ = rays .* cos(scanAngles + theta) + x;
    y_occ = -rays .* sin(scanAngles + theta) + y;
    ix_occ = ceil(x_occ * myResol) + myOrigin(1);
    iy_occ = ceil(y_occ * myResol) + myOrigin(2);

    % Find occupied-measurement cells and free-measurement cells
    occ = sub2ind(size(myMap), iy_occ, ix_occ); % Convert to 1d

    free = [];
    for k = 1:numScans
        [ix_free, iy_free] = bresenham(ix_robot, iy_robot, ix_occ(k), iy_occ(k));  
        free = [free; iy_free, ix_free];
    end
    free = sub2ind(size(myMap), free(:, 1), free(:, 2)); % Convert to 1d

    % Update the log-odds
    myMap(occ) = myMap(occ) + lo_occ;
    myMap(free) = myMap(free) - lo_free;

    % Saturate the log-odd values
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;

    %% Visualize the myMap as needed
    %imagesc(myMap); hold on;
    %plot(ix_robot, iy_robot, 'rx', 'LineWidth', 3); % indicate robot location
    %pause(0.001);
end

end
