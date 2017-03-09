function varargout = EllipsoidFromShapeMatrix(x_c,SM,FaceColor,EdgeColor,FaceAlpha,PlotOn)
%> @file EllipsoidFromShapeMatrix.m
% ======================================================================
%> @brief Plots an ellipsoid from a shape matrix input. 
%>
%> For plotting the results of a SLAM run where the features (ellipsoids)
%> have their size and orienataion represented by shape matrices. Also uses the
%> centroid to plot the ellipsoid. 
%> 
%> @pram x_c a 3x1 vector giving the 3D centroid of the ellipsoid
%> @param SM The 3x3 Shape Matrix for the ellipsoid
%> @param FaceColor the string denoting the color for the face of the
%> ellipsoid.
%> @param EdgeColor the string denoting the color for the edges of the
%> ellipsoid. The edges is the frame that is plotted to show the ellipsoid.
%> @param PlotOn - flag for plotting or not
%
%   Output (in varargout)
%       g               Handle for the plot
%       X,Y,Z           The plot outputs
%>
% ======================================================================
%> CREATED 20150816 - BMorrell - To use the shape matrix of an ellipsoid and
%> the centroid only to plot an ellipsoid. 


%% Take the SVD to extract information from the SM (note this is not fully unique)!
[u,s] = svd(SM);

% Assign magnitudes of axes
xr = sqrt(1/s(1,1));
yr = sqrt(1/s(2,2));
zr = sqrt(1/s(3,3));

% Get rotation Matrix from s
Rot_Mat_gb = u; % or s'


%% Create the Ellipsoid

[X,Y,Z]=ellipsoid(x_c(1),x_c(2),x_c(3),...
    xr,yr,zr);
    
% Take away centre
X1 = X - x_c(1);Y1 = Y - x_c(2);Z1 = Z - x_c(3);
Xr = X1.*Rot_Mat_gb(1,1)+Y1.*Rot_Mat_gb(1,2)+Z1.*Rot_Mat_gb(1,3);
Yr = X1.*Rot_Mat_gb(2,1)+Y1.*Rot_Mat_gb(2,2)+Z1.*Rot_Mat_gb(2,3);
Zr = X1.*Rot_Mat_gb(3,1)+Y1.*Rot_Mat_gb(3,2)+Z1.*Rot_Mat_gb(3,3);
X = Xr + x_c(1); Y = Yr + x_c(2); Z = Zr + x_c(3);

if PlotOn
    g=surf(X,Y,Z,'FaceAlpha',FaceAlpha,...
        'FaceColor',FaceColor,...
        'EdgeColor',EdgeColor);
else
    g = 0;
end

%% output
varargout{1} = g;
varargout{2} = X;
varargout{3} = Y;
varargout{4} = Z;