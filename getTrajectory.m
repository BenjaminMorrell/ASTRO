function states = getTrajectory(CLegPoly)
%GETTRAJECTORY Generates a state trajectory based on time points in cfg
% Multiplies the pre-computed Legendre polynomials with the input
% coefficients to generate the trajectory. 
global OPT

pos=(OPT.cfg.P.posScaled*reshape(CLegPoly,OPT.cfg.N,OPT.cfg.Nx))';
vel=(OPT.cfg.P.velScaled*reshape(CLegPoly,OPT.cfg.N,OPT.cfg.Nx))';
acc=(OPT.cfg.P.accScaled*reshape(CLegPoly,OPT.cfg.N,OPT.cfg.Nx))';

states = [pos; vel; acc];

end

