
function [ xy ] = sigmaEllipse2D( mu, Sigma, level, npoints )
%SIGMAELLIPSE2D generates x,y-points which lie on the ellipse describing
% a sigma level in the Gaussian density defined by mean and covariance.
%
%Input:
%   MU          [2 x 1] Mean of the Gaussian density
%   SIGMA       [2 x 2] Covariance matrix of the Gaussian density
%   LEVEL       Which sigma level curve to plot. Can take any positive value, 
%               but common choices are 1, 2 or 3. Default = 3.
%   NPOINTS     Number of points on the ellipse to generate. Default = 32.
%
%Output:
%   XY          [2 x npoints] matrix. First row holds x-coordinates, second
%               row holds the y-coordinates. First and last columns should 
%               be the same point, to create a closed curve.


%Setting default values, in case only mu and Sigma are specified.
if nargin < 3
    level = 3;
end
if nargin < 4
    npoints = 32;
end

%Your code here
% -------------------
angle = zeros(1,npoints);
angle1 = 0;
angle2 = 2*pi;
angle_step = angle2./npoints;
a = angle1;
for i = 1:npoints
    angle(i) = a;
    a = a + angle_step;
end
Cos_theta = cos(angle);
Sin_theta = sin(angle);
Cos_Sin = [Cos_theta ; Sin_theta];
Sigma_sqrt = sqrtm(Sigma);              %here sigma -> covarience
val = level.*(Sigma_sqrt*Cos_Sin);
xy = mu + val;
end
