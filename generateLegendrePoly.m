function generateLegendrePoly(N,nsteps)
%> @file generateLegendrePoly.m
% ======================================================================
%> @brief Computes the Legendre polynomial coefficients for path planning.
%>
%> This script computes the matrices that need to be calculated only once,
%> when the problem dimension (N) is selected: the Legendre
%> Polynomial derivatives integral for the path length, and the Legendre
%> Polynomial values at each timestep. These are used for defining the
%> path, and evaluating the cost function for ASTRO. 
%> It is intended that this function will be used in the ASTRO path
%> planning algorithm.
%>
%> @param N The maximum order of the Legnedre polynomials.
%> @param nsteps The number of steps in the discretised trajectory that the
%> polynomials represent. 
%>
%> @param P A structure that contains the properties and paramters for the polynomials
%>
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODIFIED 20150426 - BMorrell - for ASTRO_Base
global OPT

% Create Legendre Polynomial matrix, of order N-1
PolLeg = legendreMatrix(N-1);   %Legendre polynomial coefficient matrix: PolLeg = [P_Legendre(1); P_Legendre(2); ...; P_Legendre(N)] 

% initialise the derivative and the integral of the legendre polynomials.
PolLegDot = zeros(N-1,N-2);  % Derivative of PolLeg
PolLegInt = zeros(N);        % Integral of PolLeg
PolLegInt(1,end) = 1;        % To give a constant term to the integral

% Loop through each Legendre Polynomial and compute the derivative and integral
for i = 2:N
    if i<N % can't do derivative for the last polynomial
        % Derivative of the polynomial (note the reduction in the number of terms
        PolLegDot(i,(N-1)-i+1:(N-1)-1) = polyder(PolLeg(i,:));
    end
    % Integral of the polynomial
    PolLegInt(i,:) = polyint(PolLeg(i-1,:),0); % leave constant of integration as zero
    
    % Set constant of integration, to make it equal to zero at x = +_ 1
    if i>2 % Special case for the x term, so only for i>2
        PolLegInt(i,end) = -sum(PolLegInt(i,1:(end-1)));
    end
end

% Add zero row to top of Velocity values for the first polynomial being = 0
PolLeg = [zeros(1,N-1);PolLeg];
PolLegDot = [zeros(1,N-2);PolLegDot];

% Populate Coefficient Vectors
OPT.cfg.P.PolLeg = PolLeg;
OPT.cfg.P.PolLegDot = PolLegDot;
OPT.cfg.P.PolLegInt = PolLegInt;

%% Unscaled state values
% Creates a matrix that can be multiplied by the legendre coefficients to
% get the state values at each of the timesteps being considered
% Create the scaled time vector (from -1 to 1)
tsteps = linspace(-1,1,nsteps);

% For each step, evaluate the unscaled (gets scaled depending on the final
% time) position, velocity and acceleration vectors.
for tIdx = 1:length(tsteps)
    for pIdx = 1:N
        OPT.cfg.P.posUnscaled(tIdx,pIdx) = polyval(PolLegInt(pIdx,:),tsteps(tIdx));
        OPT.cfg.P.velUnscaled(tIdx,pIdx) = polyval(PolLeg(pIdx,:),tsteps(tIdx));
        OPT.cfg.P.accUnscaled(tIdx,pIdx) = polyval(PolLegDot(pIdx,:),tsteps(tIdx));
    end
end
      
%% Integral path cost
% Integrate the polynomials involved in the square of the path length to
% have the constant part of the ASTRO Cost equation.

for pidx = 1:N % Loop through each polynomial
    pol = PolLeg(pidx,:);            % P'_k(t')
    intpol = polyint(conv(pol,pol));    % Squared and integrated
    intsqeval = polyval(intpol,[1 -1]); % Evaluate at boundaries
    OPT.cfg.P.intVelSq(pidx) = intsqeval(1)-intsqeval(2); % Evaluate the definite integral and store for future use
end

end