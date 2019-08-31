%Choose object detection probability
P_D = 0.5;
%Choose clutter rate
lambda_c = 100;
%Create sensor model
range_c = [-1000 1000;-1000 1000];
sensor_model = modelgen.sensormodel(P_D,lambda_c,range_c);
        
%Create ground truth model
nbirths = 1;
K = 100;
initial_state.x = [0; 0; 10; 10];
initial_state.P = eye(4);
ground_truth = modelgen.groundtruth(nbirths,initial_state.x,1,K+1,K);
        
%Create linear motion model
T = 1;
sigma_q = 5;
motion_model = motionmodel.cvmodel(T,sigma_q);
        
%Create linear measurement model
sigma_r = 10;
meas_model = measmodel.cvmeasmodel(sigma_r);

%Generate true object data (noisy or noiseless) and measurement data
ifnoisy = 0;
objectdata = objectdatagen(ground_truth,motion_model,ifnoisy);
[measdata , n_clutter] = measdatagen(objectdata,sensor_model,meas_model);

%Single object tracker parameter setting
P_G = 0.999;            %gating size in percentage
w_min = 1e-3;           %hypothesis pruning threshold
merging_threshold = 2;  %hypothesis merging threshold
M = 100;                %maximum number of hypotheses kept in Gaussian sum filter
density_class_handle = feval(@GaussianDensity);    %density class handle
tracker = singleobjectracker();
tracker = tracker.initialize(density_class_handle,P_G,meas_model.d,w_min,merging_threshold,M);

%Nearest neighbour filter
nearestNeighborEstimates = nearestNeighbourFilter(tracker, initial_state, measdata, sensor_model, motion_model, meas_model);
%Probabilistic data association filter
probDataAssocEstimates = probDataAssocFilter(tracker, initial_state, measdata, sensor_model, motion_model, meas_model);
%Gaussian sum filter
GaussianSumEstimates = GaussianSumFilter(tracker, initial_state, measdata, sensor_model, motion_model, meas_model);


true_state = cell2mat(objectdata.X');
NN_estimated_state = cell2mat(nearestNeighborEstimates');
PDA_estimated_state = cell2mat(probDataAssocEstimates');
GS_estimated_state = cell2mat(GaussianSumEstimates');

% Visualize the result in the form of movie
% -----------------------------------------
filename_nnf = 'NNF.gif';
filename_pdaf = 'PDAF.gif';
filename_gsf = 'GSF.gif';
filename_meas = 'MEAS.gif';
filename_perf = 'COMPARISON.gif';
fID_MEAS = 1;
fID_NNF = 2;
fID_PDAF = 2;
fID_GSF = 2;
fID_filter = 1;
filename_filter = filename_perf;
%estimated_state = GS_estimated_state;
%figure
%hold on
%grid on

t = length(measdata);
samples_t = zeros(t,1);
measdata_array = [];  %measurememt array (clutter + obj originated meas)

for idx = 1:t
    array = cell2mat(measdata(idx));
    measdata_array = [measdata_array , array];
    samples = length(cell2mat(measdata(idx)));
    samples_t(idx,1) = samples;
    n_clutter_meas = n_clutter(idx,1);
    no_obj = samples - n_clutter_meas;
    
    X_obj = array(1,1:no_obj);
    Y_obj = array(2,1:no_obj);
    X_clutter = array(1, (no_obj + 1):end);
    Y_clutter = array(2, (no_obj + 1):end);
    X_groundTruth = true_state(1,idx);
    Y_groundTruth = true_state(2,idx);
    
    pause(0.1)
    
    figure(fID_filter);
    subplot(3,1,1)
    plot(NN_estimated_state(1,idx),NN_estimated_state(2,idx), '*','color','red');
    hold on;
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    %axis equal;
    title(strcat('NNF ' ,'(time = ',num2str(idx),')'));
    set(gca, 'XLim',[0 1200]);
    set(gca, 'YLim',[0 1200]);
    
    subplot(3,1,2)
    plot(PDA_estimated_state(1,idx), PDA_estimated_state(2,idx), '*','color','red');
    hold on;
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    %axis equal;
    title(strcat('PDAF ' ,'(time = ',num2str(idx),')'));
    set(gca, 'XLim',[0 1200]);
    set(gca, 'YLim',[0 1200]);
    
    subplot(3,1,3)
    plot(GS_estimated_state(1,idx), GS_estimated_state(2,idx), '*','color','red');
    hold on;
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    %axis equal;
    title(strcat('GSF ' ,'(time = ',num2str(idx),')'));
    set(gca, 'XLim',[0 1200]);
    set(gca, 'YLim',[0 1200]);
    
    drawnow
    
    % 'gcf' can handle if you zoom in to take a movie.
    frame_filter = getframe(fID_filter);
    image_filter = frame2im(frame_filter);
    [imind_filter,cm_filter] = rgb2ind(image_filter,256);
    % Write to the GIF File 
    if idx == 1 
        imwrite(imind_filter,cm_filter,filename_filter,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind_filter,cm_filter,filename_filter,'gif','WriteMode','append');
    end 
end

%plot(true_state(1,:), true_state(2,:), 'g','Linewidth', 2)
%plot(NN_estimated_state(1,:), NN_estimated_state(2,:), 'r-s' , 'Linewidth', 1)

%xlabel('x (m)')
%ylabel('y (m)')
%legend('Ground Truth','Nearest Neighbour', 'Location', 'best')
%legend('Ground Truth', 'Probalistic Data Association', 'Location', 'best')
%legend('Ground Truth','Gaussian Sum', 'Location', 'best')
%legend('Ground Truth','Nearest Neighbour', 'Probalistic Data Association', 'Gaussian Sum', 'Location', 'best')

%set(gca,'FontSize',12)