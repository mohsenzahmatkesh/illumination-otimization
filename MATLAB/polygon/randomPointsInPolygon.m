function [x_rand, y_rand] = randomPointsInPolygon(pgon, num_points)
% RANDOMPOINTSINPOLYGON Generate random points inside a polygon.
%
% Inputs:
%   pgon        - polyshape object defining the polygon
%   num_points  - number of random points to generate
%
% Outputs:
%   x_rand      - x coordinates of the points
%   y_rand      - y coordinates of the points

    % Get bounding box of polygon
    [x_bounds, y_bounds] = boundingbox(pgon);
    x_min = x_bounds(1);
    x_max = x_bounds(2);
    y_min = y_bounds(1);
    y_max = y_bounds(2);
    
    % Initialize arrays
    x_rand = zeros(num_points, 1);
    y_rand = zeros(num_points, 1);
    
    count = 0;
    max_attempts = num_points * 10;  % prevent infinite loop
    attempts = 0;
    
    while count < num_points && attempts < max_attempts
        % Generate random point in bounding box
        x_try = x_min + (x_max - x_min) * rand();
        y_try = y_min + (y_max - y_min) * rand();
        
        % Check if point is inside polygon
        if isinterior(pgon, x_try, y_try)
            count = count + 1;
            x_rand(count) = x_try;
            y_rand(count) = y_try;
        end
        
        attempts = attempts + 1;
    end
    
    % Warn if not enough points found
    if count < num_points
        warning('Only %d points were generated inside the polygon.', count);
        x_rand = x_rand(1:count);
        y_rand = y_rand(1:count);
    end
    
end
