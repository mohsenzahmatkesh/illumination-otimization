function [pgon, center, vertex_coords] = generateRandomConvexPolygon(Nsides, r_lim, r_min, r_max)
% generateRandomConvexPolygon
% Generates a convex polygon with Nsides, random center, and vertices
% randomly distributed between r_min and r_max from the center.
%
% Inputs:
%   Nsides - Number of vertices/sides of the polygon
%   r_lim  - Maximum radius from the origin for polygon center
%   r_min  - Minimum radius from polygon center to a vertex
%   r_max  - Maximum radius from polygon center to a vertex
%
% Outputs:
%   pgon         - polyshape object for plotting
%   center       - 1x2 array with center [x, y]
%   vertex_coords - Nx2 matrix with vertex coordinates [x_i, y_i]

    % Input sanity checks
    if r_min < 0 || r_max < 0 || r_min > r_max
        error('Invalid r_min / r_max inputs.');
    end
    if r_lim < 0
        error('r_lim must be non-negative.');
    end

    % Step 1: Random center inside a circle with radius r_lim
    center_theta = 2 * pi * rand;
    center_radius = r_lim * sqrt(rand);   % sqrt for uniform distribution over area
    [center_x, center_y] = pol2cart(center_theta, center_radius);
    center = [center_x, center_y];

    % Step 2: Generate random angles sorted to ensure convexity
    angles = sort(2 * pi * rand(Nsides, 1));

    % Step 3: Random distances for each vertex between r_min and r_max
    radii = r_min + (r_max - r_min) * rand(Nsides, 1);

    % Step 4: Convert polar coordinates to Cartesian, relative to center
    [x_offsets, y_offsets] = pol2cart(angles, radii);
    x_vertices = center_x + x_offsets;
    y_vertices = center_y + y_offsets;

    % Step 5: Create the polyshape
    pgon = polyshape(x_vertices, y_vertices);

    % Return vertex coordinates for reference
    vertex_coords = [x_vertices, y_vertices];

end
