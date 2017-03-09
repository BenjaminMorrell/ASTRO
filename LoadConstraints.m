function [constr_count] = LoadConstraints()
% Function loads constraints for ASTRO for the given test case
global OPT

% start count for constraints
constr_count = 0;

%% Performance Constaints
if OPT.performance
    [constr_count] = initAccelerationConstraint(constr_count);
end

%% Static Constraints
if OPT.static
    [constr_count] = loadStaticConstraints(constr_count);
end

%% Dynamic Constraints
if OPT.dynamic
    [constr_count] = loadDynamicConstraints(constr_count);
end

%% Corridors
if OPT.corridor
    [constr_count] = loadCorridorConstraints(constr_count);
end