function [equal] = zyz_compare(zyz1, zyz2)
%ZYZ_COMPARE Determines whether two sets of ZYZ euler angles are
%functionally equal
%   Inputs:
%       zyz1, zyz2 = 1x3 vectors of zyz euler angles
%   Outputs:
%       equal = boolean, true if zyz1 and zyz2 are equivalent rotations

    tol = 1e-4; % Equality tolerance
    
    % Non-singularity case - equivalent angles
    % theta ~= 0 for both sets
    if zyz1(2) ~= 0 && all(ismembertol(zyz1, zyz2, tol))
        equal = true;
    % Singularity case - theta = 0 for both sets
    elseif zyz1(2) == 0 && zyz2(2) == 0
        % Get sums of z-rotations
        total_z_1 = zyz1(1) + zyz1(3);
        total_z_2 = zyz2(1) + zyz2(3);

        % Account for total_z outside (-2pi, 2pi) range
        if total_z_1 >= 2 * pi 
            total_z_1 = total_z_1 - 2 * pi;
        elseif total_z_1 <= -2 * pi
            total_z_1 = total_z_1 + 2 * pi;
        end

        if total_z_2 >= 2 * pi 
            total_z_2 = total_z_2 - 2 * pi;
        elseif total_z_2 <= -2 * pi
            total_z_2 = total_z_2 + 2 * pi;
        end

        % Compare adjusted z-rotations
        if ismembertol(total_z_1, total_z_2, tol)
            equal = true;
        else
            equal = false;
        end
    % Inequality case
    else
        equal = false;
    end
end