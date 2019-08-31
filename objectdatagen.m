
%dependensies:
% 1) modelgen.m
% 2) motionmodel.m

%Model structures need to be called:
    %groundtruth: a structure specifies the parameters to generate the ground truth
    %           nbirths: number of objects hypothesised to exist from
    %           time step 1 to time step K --- scalar 
    %           xstart: object initial state --- (object state
    %           dimension) x nbirths matrix 
    %           tbirth: object birth (appearing) time --- (total number
    %           of objects existed in the scene) x 1 vector  
    %           tdeath: the last time the object exists ---  (total number
    %           of objects existed in the scene) x 1 vector 
    %           K: total tracking time --- scalar
    %motionmodel: a structure specifies the motion model parameters
    %           d: object state dimension --- scalar
    %           F: function handle return transition/Jacobian matrix
    %           f: function handle return predicted object state
    %           Q: motion noise covariance matrix
function objectdata = objectdatagen(groundtruth,motionmodel,ifnoisy)
%TARGETDATA generates groundtruth object data
%INPUT:  groundtruth specifies the parameters used to generate groundtruth
%        motionmodel: a structure specifies the motion model parameters
%        ifnoisy: boolean value indicating whether to generate noisy object
%        state sequence or not 
%OUTPUT: objectdata.X:  (K x 1) cell array, each cell stores object states
%        of size (object state dimension) x (number of objects at
%        corresponding time step)  
%        objectdata.N:  (K x 1) vector, each element stores the number of
%        objects at corresponding time step 
%function obj = groundtruth(nbirths,xstart,tbirth,tdeath,K) <===== DELETE LATER
%n_obj = groundtruth.nbirths;  %scalar  number of objects
%Time = groundtruth.K;      %scalar total tracking time
%obj_birth_time = groundtruth.tbirth; % (n_obj x 1)
%obj_death_time = groundtruth.tdeath;  % (n_obj x 1)
%obj_Xstart = groundtruth.xstart;  %(dim x n_obj)
%curr_data = obj_Xstart;
%state_dim = motionmodel.d;   % 4
%process_noise_cov = motionmodel.Q % (4x4)
% ====================================================================================================%

%compute the number of objects in 1:K
n_objects = zeros(1,groundtruth.K);  %initialize a vector of zeros of size (1 x Tracking_time)
                                     %this will hold the number of objects
                                     %at each time
for t_idx = 1:groundtruth.K          %iterate over each time
    for t_birth_idx = 1:size(groundtruth.tbirth,1)     %iterate over each of the object birth time
        t_birth = groundtruth.tbirth(t_birth_idx,1);   %birth time of each object
        t_death = min(groundtruth.tdeath(t_birth_idx,1) , groundtruth.K);  %death time of each object
        if t_idx>=t_birth && t_idx<=t_death   % increment the number of objects accordingly         
            n_objects(1,t_idx) = n_objects(1,t_idx) + 1;
        end
    end
end


%maximum objects present at a time t
obj_max = max(n_objects);

%initialize the cell arrays to nulls
for t_idx = 1:groundtruth.K
    objectdata.X{t_idx,1} = [];   %object states at each time for each object
end

%generate the ground truth (motion model data sequence)
for n_birth_idx = 1:size(groundtruth.tbirth,1)    %pick each object
    t_birth = groundtruth.tbirth(n_birth_idx,1);  %time of birth of that object
    t_death = groundtruth.tdeath(n_birth_idx,1);  %time of death of that object
    %sim_time = (t_death - t_birth);
    
    prev_state = groundtruth.xstart(:,n_birth_idx); %previous state of that object
    
    for t = t_birth:min(t_death , groundtruth.K)
        if ifnoisy == 0                             %if noise is zero
            curr_state = motionmodel.f(prev_state); %current state
        end
        if ifnoisy == 1                             %if noise is valid
            curr_state = motionmodel.f(prev_state) + mvnrnd(zeros(motionmodel.d,1),motionmodel.Q)';
        end
        objectdata.X{t,1} = [objectdata.X{t,1} , curr_state];  
        prev_state = curr_state;
    end     
end

objectdata.N = n_objects';

end %end of the function
