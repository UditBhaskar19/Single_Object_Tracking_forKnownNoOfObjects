%Here we give parameters for a Gaussian density. The parameter mu is the mean, and P is the covariance matrix.
mu = [-2; 1];
P = [4, -2; -2 2];

%Call your function.
xy = sigmaEllipse2D(mu, P);

%Now plot the generated points. You should see an elongated ellipse stretching from the top left corner to the bottom right. 
figure(1);
Axy = [xy(:,:), xy(:,1)]; 
%h1 = plot(xy(1,:), xy(2,:));
h1 = plot(Axy(1,:), Axy(2,:));
%Set the scale of x and y axis to be the same. This should be done if the two variables are in the same domain, e.g. both are measured in meters.
axis equal
hold on
%Also plot a star where the mean is, and make it have the same color as the ellipse.
plot(mu(1), mu(2), '*', 'color', h1.Color);