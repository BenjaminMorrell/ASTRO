function [store,timing] = InitialiseVariables()
% Function to initialise variables and pre-allocate the sizes

% Store variables
store.costHist = [];
store.polyHist = [];
store.gradHist = [];
store.state = [];

% Timing
timing = 0;

% Start timing counting
tic;
