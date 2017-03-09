function SetUpSolver()
% CREATED 20150621 - BMorrell - To Auto load all required settings
% MODIFIED 20160426 - BMorrell - use global OPT for ASTRO base
global OPT
%% Set up solver
OPT.cfg         = initAstro();

%% Generate polynomial values
generateLegendrePoly(OPT.cfg.N,OPT.cfg.nSamp);