function P = legendreMatrix(N)
% LEGENDREMATRIX Generate a matrix of Legendre polynomial coefficients
% This funciton creates coefficients for the legendre polynomials with
% order 0 to N-1.  The resulting matrix is NxN

% initialise matriz
P = zeros(N,N);

% First Legendre Term
P(1,N) = 1;

% Loop through each term to define the Legendre Polynomial Coefficient
for i=2:N
    P(i,N-i+1:N) = legpoly(i-1);
end

end

function Pn = legpoly(N)
%LEGPOLY creates a legendre polynomial
% Polynomial coefficients are a vector in standard MATLAB form with
% decreasing order from left to right.

if (N==0) % P_0 = 1
    Pn=1;
elseif (N==1) % P_1 = x
    Pn=[1 0];
else %Generate according to Bonnet's Recursion
    Pn2=legpoly(N-2); %P_(N-2)
    Pn1=legpoly(N-1); %P_(N-1)
    
    %Formula is: P_N = (1/N) * [(2N-1)xP_(N-1) - (N-1)P_(N-2)]
    Pn=(1/N)*( (2*N-1)*conv([1 0],Pn1) - (N-1)*[0 0 Pn2]);
end;

end