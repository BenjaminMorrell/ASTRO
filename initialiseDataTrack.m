function DataTrack = initialiseDataTrack()
% initialises the data tracking data structure for ASTRO Solver
global OPT

DataTrack = struct('Poly_C', zeros(OPT.cfg.Nx*OPT.cfg.N,1), ... Polynomials
           'polystep',zeros(OPT.cfg.Nx*OPT.cfg.N,1),... polynomial step
           'totalCost',0,... total cost each iteration
           'maxViol',0,... maximum violation cost
           'gradient',zeros(OPT.cfg.Nx*OPT.cfg.N,1),... cost gradient 
           'coeff',1,... linesearch coefficient 
           'innerIter', 0,... iteration count for line-search
           'costDiff', 0 ... Cost change
           );