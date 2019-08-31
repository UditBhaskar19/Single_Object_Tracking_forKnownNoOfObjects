
%Create motion model
T = 1;
sigma_q = 1;
motion_model = motionmodel.cvmodel(T,sigma_q);

%Create measurement model
sigma_r = 1;
meas_model = measmodel.cvmeasmodel(sigma_r);

%Create tracking model
P_D = 0.9;
lambda_c = 10;
range_c = [-100 100;-100 100];
sensor_model = modelgen.sensormodel(P_D,lambda_c,range_c);

nbirths = 3;
K = 10;

tbirth = zeros(nbirths,1);
tdeath = zeros(nbirths,1);
initial_state.x = zeros(motion_model.d,nbirths);
initial_state.x(:,1) = [0; 0; 0; -10];        tbirth(1) = 1;   tdeath(1) = 5;
initial_state.x(:,2) = [400; -600; -10; 5];   tbirth(2) = 2;   tdeath(2) = 8;      
initial_state.x(:,3) = [-800; -200; 20; -5];  tbirth(3) = 3;   tdeath(3) = K;
ground_truth = modelgen.groundtruth(nbirths,initial_state.x,tbirth,tdeath,K);

%Create object data
objectdata = objectdatagen(ground_truth,motion_model,0);

%Create measurement data
measdata = measdatagen(objectdata,sensor_model,meas_model);