%Choose object detection probability
P_D = 0.9;
%Choose clutter rate
lambda_c = 10;
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

true_state = cell2mat(objectdata.X');
NN_estimated_state = cell2mat(nearestNeighborEstimates');

% Visualize the result in the form of movie
% -----------------------------------------
filename_nnf = 'NNF.gif';
filename_meas = 'MEAS.gif';
fID_NNF = 2;
fID_MEAS = 1;
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
    
    figure(fID_MEAS);
    %subplot(2,2,1)
    plot(X_obj,Y_obj, '*','color','red');
    hold on;
    plot(X_clutter,Y_clutter, '*', 'color', 'blue');
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    legend('Obj Originated meas','Clutter', 'Location', 'best')
    %axis equal;
    title(strcat('time = ',num2str(idx)));
    set(gca, 'XLim',[-1000 2000]);
    set(gca, 'YLim',[-1000 2000]);
    hold off;
    
    figure(fID_NNF);
    subplot(3,1,1)
    plot(X_obj,Y_obj, '.','color','red');
    hold on;
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    %axis equal;
    title(strcat('Object Detections ' ,'(time = ',num2str(idx),')'));
    set(gca, 'XLim',[0 1200]);
    set(gca, 'YLim',[0 1200]);
    
    subplot(3,1,2)
    plot(true_state(1,idx), true_state(2,idx), '.','color','black');
    hold on;
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    %axis equal;
    title(strcat('Ground Truth ' ,'(time = ',num2str(idx),')'));
    set(gca, 'XLim',[0 1200]);
    set(gca, 'YLim',[0 1200]);
    
    subplot(3,1,3)
    plot(NN_estimated_state(1,idx), NN_estimated_state(2,idx), '.','color','magenta');
    hold on;
    grid on
    xlabel('x(m)');
    ylabel('y(m)');
    %axis equal;
    title(strcat('NN Filter estimates ' ,'(time = ',num2str(idx),')'));
    set(gca, 'XLim',[0 1200]);
    set(gca, 'YLim',[0 1200]);
    
    drawnow
    
    frame_meas = getframe(fID_MEAS); % 'gcf' can handle if you zoom in to take a movie.
    frame_nnf = getframe(fID_NNF);
    image_meas = frame2im(frame_meas);
    image_nnf = frame2im(frame_nnf);
    [imind_meas,cm_meas] = rgb2ind(image_meas,256);
    [imind_nnf,cm_nnf] = rgb2ind(image_nnf,256);
    % Write to the GIF File 
    if idx == 1 
        imwrite(imind_meas,cm_meas,filename_meas,'gif', 'Loopcount',inf); 
        imwrite(imind_nnf,cm_nnf,filename_nnf,'gif', 'Loopcount',inf); 
    else 
        imwrite(imind_meas,cm_meas,filename_meas,'gif','WriteMode','append');
        imwrite(imind_nnf,cm_nnf,filename_nnf,'gif','WriteMode','append');
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