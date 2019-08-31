%Create motion model
T = 1;
sigma_q = 1;
motion_model = motionmodel.cvmodel(T,sigma_q);

%Create tracking model
nbirths = 2;
K = 10;
tbirth = zeros(nbirths,1);
tdeath = zeros(nbirths,1);
initial_state.x = zeros(motion_model.d,nbirths);
initial_state.P = eye(motion_model.d);
initial_state.x(:,1) = [0; 0; 0; -10];        tbirth(1) = 1;   tdeath(1) = 5;
initial_state.x(:,2) = [400; -10; -600; 5];   tbirth(2) = 5;   tdeath(2) = K;    
ground_truth = modelgen.groundtruth(nbirths,initial_state.x,tbirth,tdeath,K);

%Create object data (ground truth)
ifnoisy = true;
objectdata = objectdatagen(ground_truth,motion_model,ifnoisy);

disp(cell2mat(objectdata.X'))
disp(objectdata.N)

array_obj_data = cell2mat(objectdata.X');
size(array_obj_data)
plot(array_obj_data(1,:),array_obj_data(2,:),'*');