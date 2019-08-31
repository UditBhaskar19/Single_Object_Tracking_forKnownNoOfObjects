
%dependensies:
% 1) modelgen.m
% 2) measmodel.m
% 3) motionmodel.m
% 4) objectdatagen.m

%Model structures need to be called:
    %objectdata: a structure specifies the object trajectories
    %           X: (K x 1) cell array, each cell stores object states
    %           of size (object state dimension) x (number of objects at
    %           corresponding time step)  
    %           N:  (K x 1) cell array, each cell stores the number of
    %           objects at corresponding time step 
    %sensormodel: a structure specifies the sensor parameters
    %           P_D: object detection probability --- scalar
    %           lambda_c: average number of clutter measurements per time
    %           scan, Poisson distributed --- scalar 
    %           range_c: range of surveillance area --- if 2D model: 2
    %           x 2 matrix of the form [xmin xmax;ymin ymax]; if 1D
    %           model: 1 x 2 vector of the form [xmin xmax] 
    %           pdf_c: clutter (Poisson) density --- scalar
    %           intensity_c: clutter (Poisson) intensity --- scalar
    %measmodel: a structure specifies the measurement model parameters
    %           d: measurement dimension --- scalar
    %           H: function handle return transition/Jacobian matrix
    %           h: function handle return the observation of the object
    %           state 
    %           R: measurement noise covariance matrix
function [measdata , n_clutter] = measdatagen(objectdata, sensormodel, measmodel)
%MEASDATAGEN generates object-generated measurements and clutter
%INPUT:     objectdata: a structure contains object data
%           sensormodel: a structure specifies sensor model parameters
%           measmodel: a structure specifies the measurement model
%           parameters 
%OUTPUT:    measdata: cell array of size (total tracking time, 1), each
%           cell stores measurements of size (measurement dimension) x
%           (number of measurements at corresponding time step) 

% We strongly recommend you to make use of the code snippet provided below,
% such that the random numbers generated in your code are reproducible. Otherwise,
% you may fail to pass the tests even if your implementation is correct.

%Initialize memory
K = length(objectdata.X);      %total tracking time
measdata = cell(K,1);          %initialize a vector of cells of tracking time
n_clutter = zeros(K,1);

%Generate measurements
for k = 1:K
    if objectdata.N(k) > 0          %if one or more than one object is present at a time k
        DATA = objectdata.X{k,1};   %object state data
        idx = (rand(objectdata.N(k),1) <= sensormodel.P_D);  %generate a vector of uniform random numbers
                                                             %size 'no of objects at time k'
        %generate a boolean vector flag (if rand(num) <= P_D --> idx = 1 , else 0)
        %Only generate object-originated observations for detected objects
        size_idx = size(idx,1);          %number of objects at time k
        for idx_detected = 1:size_idx    %loop over the number of objects at time k
            if(idx(idx_detected,1) == 1) %if the object is detected (probability of detetction <= P_D)
                obj_data = DATA(:,idx_detected);
                meas_data = measmodel.h(obj_data) + mvnrnd(zeros(measmodel.d,1) , measmodel.R)';
                measdata{k,1} = [measdata{k,1} , meas_data];
            end
        end
    end
    % ---------------------------------------------------------------------------
    %Number of clutter measurements
    N_c = poissrnd(sensormodel.lambda_c); % generate the number of clutter measurements as per poisson distribution
    %Generate clutter
    C = [];
    for idxx = 1:N_c
        % generate samples of the clutter as per the uniform spatial
        % distribution
        C_i = sensormodel.range_c(:,1) + ( sensormodel.range_c(:,2) - sensormodel.range_c(:,1) ).*rand(measmodel.d,1);
        C = [C , C_i];
    end
    n_clutter(k,1)= length(C);  %to store the number of clutters (for visualization)
    % -----------------------------------------------------------------
    %Total measurements are the union of object detections and clutter
    measdata{k} = [measdata{k} C]; %concatenate the object originated measurements and the clutter to get the list 
                                   %of measurements for every time k
end

end
