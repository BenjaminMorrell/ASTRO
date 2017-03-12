function J = JacobGen(fun,X,eta)
%> @file JacobGen.m
% ======================================================================
%> @brief Function Calculates the Jacobian matrix for the input function and
%> variables (function should be of the one variable matrix X). 
%>
%> Takes in the given function a performes central differencing to
%> approcimate the jacobian of the function.
%>
%> @param fun A function handle for the function to be linearised for the
%> Jacobain computation. 
%> @param X The current state about which to linearise the function. A
%> column vector.
%> @param eta A vector containing the amount to perturb the state in the
%> central differencing scheme. If empty, a default eta of 1x10^-5 is used.
%>
%> @param J The output Jacobian matrix (square matrix the size of the input
%> X vector).
%>
% ======================================================================


% Difference
if isempty(eta)
    eta = ones(size(X)).*1e-5;  % weight quaternions and rotation rates equally
end
% eta = [1e-5,1e-3,1e-5,1e-5,1e-5,1e-3,1e-5,1e-5]';
% eta = [0.001,0.1,0.1,0.1,0.001,0.1,0.1,0.1]';

for i = 1:length(X);
    % Perturb Forwards
    X_test1 = X; X_test1(i) = X_test1(i) + eta(i);
    C1 = fun(X_test1);
    
    % Perturb backwards
    X_test2 = X; X_test2(i) = X_test2(i) - eta(i);
    C2 = fun(X_test2);
    
    % Central differencing to get column of G
    J(:,i) = (C1'-C2')./(2*eta(i));
end

    
    