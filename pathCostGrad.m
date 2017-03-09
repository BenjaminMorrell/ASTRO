function [path_cost,path_costGrad] = pathCostGrad(CLegPoly,doGrad)
% pathCostGrad
% Runs the cost function for ASTRO, with the input polynomail coefficients
% and the legendre polynomial integrals stored in the global OPT structure.
% 
% Input
%       CLegPoly        The polynomial coefficients
%       OPT             The global options and settings structure
%
% Output
%       path_cost       The cost
%       path_costGrad   The cost gradient
%
%The cost function is calculated by calculating the total squared path
%length.  The function utilizes a pre-calculated integral stored in the cfg
%structure which represents the integral of P'k(t') from -1 to 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160429 - BMorrell - for ASTRO_base

global OPT

% Cost Function - multiply coefficeints by stored legendre polynomail integrals
path_cost = (repmat(OPT.cfg.P.intVelSq,1,OPT.cfg.Nx).*CLegPoly')*CLegPoly;

% Gradient
if doGrad
    %Gradient of the cost function
    path_costGrad = 2*repmat(OPT.cfg.P.intVelSq',OPT.cfg.Nx,1).*CLegPoly;
else
    % blank, zero gradient 
    path_costGrad = zeros(OPT.cfg.N*OPT.cfg.Nx,1);
end

