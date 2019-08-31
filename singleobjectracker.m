%dependensies
% 1) GaussianDensity.m
% 2) hypothesisReduction.m
% 3) normalizeLogWeights.m
% 4) log_mvnpdf.m
% 5) modelgen.m
% 6) measmodel.m
% 7) motionmodel.m

classdef singleobjectracker
    %SINGLEOBJECTRACKER is a class containing functions to track a single
    %object in clutter. 
    %Model structures need to be called:
    %sensormodel: a structure specifies the sensor parameters
    %           P_D: object detection probability --- scalar
    %           lambda_c: average number of clutter measurements per time
    %           scan, Poisson distributed --- scalar 
    %           pdf_c: clutter (Poisson) density --- scalar
    %           intensity_c: clutter (Poisson) intensity --- scalar
    %motionmodel: a structure specifies the motion model parameters
    %           d: object state dimension --- scalar
    %           F: function handle return transition/Jacobian matrix
    %           f: function handle return predicted object state
    %           Q: motion noise covariance matrix
    %measmodel: a structure specifies the measurement model parameters
    %           d: measurement dimension --- scalar
    %           H: function handle return transition/Jacobian matrix
    %           h: function handle return the observation of the object
    %           state 
    %           R: measurement noise covariance matrix
    
    properties
        gating      %specify gating parameter
        reduction   %specify hypothesis reduction parameter
        density     %density class handle
    end
    
    methods
        
        function obj = initialize(obj,density_class_handle,P_G,m_d,w_min,merging_threshold,M)
            %INITIATOR initializes singleobjectracker class
            %INPUT: density_class_handle: density class handle
            %       P_G: gating size in decimal --- scalar
            %       m_d: measurement dimension --- scalar
            %       wmin: allowed minimum hypothesis weight --- scalar
            %       merging_threshold: merging threshold --- scalar
            %       M: allowed maximum number of hypotheses --- scalar
            %OUTPUT:  obj.density: density class handle
            %         obj.gating.P_G: gating size in decimal --- scalar
            %         obj.gating.size: gating size --- scalar
            %         obj.reduction.w_min: allowed minimum hypothesis
            %         weight in logarithmic scale --- scalar 
            %         obj.reduction.merging_threshold: merging threshold
            %         --- scalar 
            %         obj.reduction.M: allowed maximum number of hypotheses
            %         --- scalar 
            
            obj.density = density_class_handle;
            obj.gating.P_G = P_G;
            obj.gating.size = chi2inv(obj.gating.P_G,m_d);
            obj.reduction.w_min = log(w_min);
            obj.reduction.merging_threshold = merging_threshold;
            obj.reduction.M = M;
        end
        
        function estimates = nearestNeighbourFilter(obj, state, Z, sensormodel, motionmodel, measmodel)
            %NEARESTNEIGHBOURFILTER tracks a single object using nearest
            %neighbor association 
            %INPUT: state: a structure with two fields:
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of  
            %            size (measurement dimension) x (number of
            %            measurements at corresponding time step) 
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x 1   
            
            % ------------------ STEPS -----------------
            % 1. gating
            % 2. calculate the predicted likelihood for each measurement in the gate;
            % 3. find the nearest neighbour measurement;
            % 4. compare the weight of the missed detection hypothesis 
            %    and the weight of the object detection hypothesis created using the nearest neighbour measurement;
            % 5. if the object detection hypothesis using the nearest neighbour measurement has the highest weight,
            %    perform Kalman update;
            % 6. extract object state estimate;
            % 7. prediction.
            % -------------------------------------------
            
            curr_state = state;                 %object initial state mean and covarience (x0, P0)
            tracking_time = size(Z,1);          %total tracking time
         
            estimates = cell(tracking_time,1);  %initialize memory for the estimated outputs of NN Filter
            
            for time_idx = 1:tracking_time
                 curr_meas = Z(time_idx,1);         %all available measurements at time t
                 curr_meas = cell2mat(curr_meas);   %converting cell to mat format
      
                 %perform gating
                 %--------------
                 [z_ingate, meas_in_gate] = GaussianDensity.ellipsoidalGating(curr_state, curr_meas, measmodel, obj.gating.size);
                 no_meas_in_gate = size(z_ingate,2);
                 
                 %predicted likelihood of each meas within the gate (in log scale for numerical statibility )
                 %------------------------------------------------------------------------------------------
                 predicted_likelihood = GaussianDensity.predictedLikelihood(curr_state,z_ingate,measmodel); %in log scale
                 
                 %find the nearest neighbour measurement (choose hypothesis corresponding to the maximum weight)
                 %----------------------------------------------------------------------------------------------
                 w0_unnormalized = log(1-sensormodel.P_D);  %weight corresponding to miss detectionn hypothesis
                 if(no_meas_in_gate ~= 0)
                     w_i_unnormalized = log(sensormodel.P_D/sensormodel.intensity_c) + predicted_likelihood;
                     max_w_i_unnormalized = max(w_i_unnormalized);
                 elseif (no_meas_in_gate == 0)
                     max_w_i_unnormalized = w0_unnormalized - 1;
                 end
                 
                 %if the maximum weight corresponds to a measurement within the gate , do state update with
                 %that measurement and state prediction to the next cycle, 
                 %if the maximum weight corresponds to  miss dectection hypothesis ,DO-NOT do state update 
                 %but perform only state prediction to the next cycle
                 %-----------------------------------------------------------------------------------------
                 if (w0_unnormalized <= max_w_i_unnormalized)
                     %perform state update
                     measurement_idx = find(max_w_i_unnormalized == w_i_unnormalized(:,1));
                     measurement_highest_weight = z_ingate(:,measurement_idx);
                     state_upd = GaussianDensity.update(curr_state , measurement_highest_weight, measmodel); 
                     estimates{time_idx,1} = state_upd.x;  % estimate of the NN Filter
                     state_pred = GaussianDensity.predict(state_upd , motionmodel);
                     curr_state = state_pred;
                     
                 elseif ( w0_unnormalized > max_w_i_unnormalized )
                     %perform state prediction only
                     estimates{time_idx,1} = curr_state.x;
                     state_pred = GaussianDensity.predict(curr_state , motionmodel);
                     curr_state = state_pred;
                 end
            end        
        end
        
        
        function estimates = probDataAssocFilter(obj, state, Z, sensormodel, motionmodel, measmodel)
            %PROBDATAASSOCFILTER tracks a single object using probalistic
            %data association 
            %INPUT: state: a structure with two fields:
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of size (measurement
            %       dimension) x (number of measurements at corresponding
            %       time step)  
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x 1  
            % ------ STEPS ------------------------------------------------
            % 1.gating;
            % 2.create missed detection hypothesis;
            % 3.create object detection hypotheses for each detection inside the gate;
            % 4.normalise hypothesis weights;
            % 5.prune hypotheses with small weights, and then re-normalise the weights.
            % 6.merge different hypotheses using Gaussian moment matching;
            % 7.extract object state estimate;
            % 8.prediction.
            % --------------------------------------------------------------
            curr_state = state;
            tracking_time = size(Z,1);
         
            %Initialize memory
            estimates = cell(tracking_time,1);
            
            for time_idx = 1:tracking_time
                 curr_meas = Z(time_idx,1);
                 curr_meas = cell2mat(curr_meas);
                 
                 %perform gating
                 %---------------
                 [z_ingate, meas_in_gate] = GaussianDensity.ellipsoidalGating(curr_state, curr_meas, measmodel, obj.gating.size);
                 no_meas_in_gate = size(z_ingate,2);
                 
                 %predicted likelihood of each meas within the gate , miss
                 %detection weight and updated state with each gated
                 %measurements
                 %--------------------------------------------------------
                 predicted_likelihood = GaussianDensity.predictedLikelihood(curr_state,z_ingate,measmodel); %in log scale
                 w0_unnormalized = log(1-sensormodel.P_D);
                 hypothesis_miss = curr_state;
                 hypothesis_miss_gated(1,1) = hypothesis_miss;
                 if(no_meas_in_gate ~= 0)
                     w_i_unnormalized = log(sensormodel.P_D/sensormodel.intensity_c) + predicted_likelihood;
                     Weights = [w0_unnormalized ; w_i_unnormalized];
                     for idx_gated = 1:no_meas_in_gate
                         meas_gated = z_ingate(:,idx_gated);
                         state_upd = GaussianDensity.update(curr_state , meas_gated, measmodel);
                         hypothesis_miss_gated(1+idx_gated,1) = state_upd;
                     end
                 elseif(no_meas_in_gate == 0)
                     Weights = w0_unnormalized;
                 end
                 Hypothesis_states = hypothesis_miss_gated;
                 
                 %normalize the weights
                 %---------------------
                 [Norm_Weights , Norm_Weights_sum] = normalizeLogWeights(Weights); %in log scale
                 
                 %prune the weights
                 %-----------------
%                   [hypothesesWeight111, multiHypotheses111] ...
%                      = hypothesisReduction.prune(Norm_Weights, ...
%                                                  Hypothesis_states, ...
%                                                  obj.reduction.w_min);
                 
                 prunned_measurement_idx = find(obj.reduction.w_min <= Weights(:,1));
                 prunned_hypothesis = Hypothesis_states(prunned_measurement_idx,1);
                 Prunned_Weights = Weights( (obj.reduction.w_min <= Weights(:,1) ) ,1 ); %in log scale
                 
                 %renormalize the weights
                 %-----------------------
                 Prunned_w_norm = normalizeLogWeights(Prunned_Weights);  %[w0, w1, w2,...]
                 
                 %merge the hypothesis
                 %--------------------
                 Merged_Hypothesis_state = GaussianDensity.momentMatching(Prunned_w_norm, prunned_hypothesis);
                
                 %estimate of the PDAF
                 %--------------------
                 estimates{time_idx,1} = Merged_Hypothesis_state(1,1).x;
              
                 %perform state prediction for the next cycle
                 %-------------------------------------------
                 state_pred = obj.density.predict(Merged_Hypothesis_state(1,1), motionmodel);
                 curr_state = state_pred;
             end
            
        end
        
        function estimates = GaussianSumFilter(obj, state, Z, sensormodel, motionmodel, measmodel)
            %GAUSSIANSUMFILTER tracks a single object using Gaussian sum
            %filtering
            %INPUT: state: a structure with two fields:
            %                x: object initial state mean --- (object state
            %                dimension) x 1 vector 
            %                P: object initial state covariance --- (object
            %                state dimension) x (object state dimension)
            %                matrix  
            %       Z: cell array of size (total tracking time, 1), each
            %       cell stores measurements of size (measurement
            %       dimension) x (number of measurements at corresponding
            %       time step)  
            %OUTPUT:estimates: cell array of size (total tracking time, 1),
            %       each cell stores estimated object state of size (object
            %       state dimension) x 1  
            
            % -----------------------------------------------------------------------
            % 1.for each hypothesis, create missed detection hypothesis;
            % 2.for each hypothesis, perform ellipsoidal gating and 
            %   only create object detection hypotheses for detections inside the gate;
            % 3.normalise hypothsis weights;
            % 4.prune hypotheses with small weights, and then re-normalise the weights;
            % 5.hypothesis merging (to achieve this, you just need to directly call function hypothesisReduction.merge.);
            % 6.cap the number of the hypotheses, and then re-normalise the weights;
            % 7.extract object state estimate using the most probably hypothesis estimation;
            % 8.for each hypothesis, perform prediction.
            % 9.For Gaussian sum filtering, the estimated single object state is extracted from the hypothesis with the highest weight.
            % 10.Hypothesis reduction is usually performed in the order of: 1). pruning; 2). merging; 3). capping.  
            % --------------------------------------------------------------------------
            
            %curr_state = state;
            tracking_time = size(Z,1);
         
            %Initialize memory
            estimates = cell(tracking_time,1);
            
            %initial no of hypothesis
            H0 = 1;
            Hk_1 = H0;  
            no_states = size(state,2);    %initial number of states 
            for hyp_state_idx = 1:no_states
                Hypothesised_States_prev.state(hyp_state_idx,1) = state(:,hyp_state_idx);
                Hypothesised_States_prev.weight_unnorm(hyp_state_idx,1) = log(1);
                Hypothesised_States_prev.weight(hyp_state_idx,1) = log(1);
            end
     
            for time_idx = 1:tracking_time
                 curr_meas = Z(time_idx,1);         %list of measurement at time k
                 curr_meas = cell2mat(curr_meas);   %cell to matrix (2 x n)
                 no_meas   = size(curr_meas,2);     %number of measurements
                 no_states = size(Hypothesised_States_prev.state,1);    %no of states
                 %disp('===============================');
                 count = 0;
                 count = 1;  %%%%%%%%%%%%%%%%%%%%
                 for hyp_state_idx = 1:no_states  %iterate over all the hypothesis
                     % state data : structure
                     % x: [4×1 double]
                     % P: [4×4 double]
                     state_data = Hypothesised_States_prev.state(hyp_state_idx,1);    %hypothesis
                     weight_prev = Hypothesised_States_prev.weight(hyp_state_idx,1);  %hypothesis weight from the previous cycle
                     
                     %create miss detection hypothesised state
                     %----------------------------------------
                     w0_unnormalized = weight_prev + log(1-sensormodel.P_D);
                     hypo_miss_detection = state_data;
                     miss_detection_idx = count;  %%%%%%%%%%%%%%%
                     %miss_detection_idx = hyp_state_idx + count; 
                     Hypothesised_States_curr.state(miss_detection_idx,1) = hypo_miss_detection;
                     Hypothesised_States_curr.weight(miss_detection_idx,1) = w0_unnormalized;
                     count = count + 1;
                     
                     %perform gating for the hypothesised state , predicted likelihood, log weight
                     %for the gated measurements do state update and copy the updated state
                     %----------------------------------------------------------------------------
                     [z_ingate, meas_in_gate] = GaussianDensity.ellipsoidalGating(state_data, curr_meas, measmodel, obj.gating.size);
                     predicted_likelihood = GaussianDensity.predictedLikelihood(state_data, z_ingate, measmodel); %in log scale
                     w_i_unnormalized = weight_prev + log(sensormodel.P_D/sensormodel.intensity_c) + predicted_likelihood;
                     %w_i_size = size(w_i_unnormalized) (n x 1)
                     %gg = size(z_ingate) (d x n)
                     no_gated_meas = size(z_ingate,2);
                     for idx_gated_meas = 1:no_gated_meas
                         meas = z_ingate(:,idx_gated_meas);
                         state_upd = GaussianDensity.update(state_data , meas, measmodel);
                         gated_detection_idx = count; %%%%%%%%%%%%%%%%%%%%%
                         Hypothesised_States_curr.state(gated_detection_idx,1) = state_upd;
                         Hypothesised_States_curr.weight(gated_detection_idx,1) = w_i_unnormalized(idx_gated_meas,1);
                         count = count + 1;
                     end
                 end
                 
           
                 
                 %normalize the weights
                 [Hypothesised_States_curr.weight , Norm_Weights_sum] = ...
                         normalizeLogWeights(Hypothesised_States_curr.weight); %in log scale  
                 
                 %prune hypothesis with small weights
                 [Hypothesised_States_curr.weight, Hypothesised_States_curr.state] ...
                     = hypothesisReduction.prune(Hypothesised_States_curr.weight, ...
                                                 Hypothesised_States_curr.state, ...
                                                 obj.reduction.w_min);
                     
                 %renormalize the weights
                 [Hypothesised_States_curr.weight , Norm_Weights_sum] = ...
                         normalizeLogWeights(Hypothesised_States_curr.weight); %in log scale
                
                     
                 %hypotheisi merge
                 [Hypothesised_States_curr.weight, Hypothesised_States_curr.state] ...
                       = hypothesisReduction.merge(Hypothesised_States_curr.weight, ...
                                                   Hypothesised_States_curr.state, ...
                                                   obj.reduction.merging_threshold, ...
                                                   GaussianDensity);
                
                 %hypotheisi cap
                 MM = min(obj.reduction.M, size(Hypothesised_States_curr.weight, 1));
                 [Hypothesised_States_curr.weight, Hypothesised_States_curr.state] ...
                     = hypothesisReduction.cap(Hypothesised_States_curr.weight, ...
                                               Hypothesised_States_curr.state, ...
                                               MM);                      
                 
                 %renormalize the weights
                 [Hypothesised_States_curr.weight , Norm_Weights_sum] = ...
                         normalizeLogWeights(Hypothesised_States_curr.weight); %in log scale
                 
                 max_weight = max(Hypothesised_States_curr.weight);
                 max_weight_idx = find(max_weight == Hypothesised_States_curr.weight);
                 max_hypo = Hypothesised_States_curr.state(max_weight_idx,1);
                 
                 estimates{time_idx,1} = max_hypo.x; 
                 
                 %do state prediction for all hypothesis for the next cycle
                 for idx_predict = 1:size(Hypothesised_States_curr.weight,1)
                      state_pred = GaussianDensity.predict(Hypothesised_States_curr.state(idx_predict,1), motionmodel);
                      Hypothesised_States_curr.state(idx_predict,1) = state_pred;
                 end
                 
                 Hk_1 = size(Hypothesised_States_curr.weight,1);
                 Hypothesised_States_prev = Hypothesised_States_curr;
            
            end
           
        end  % end of gaussian sum filter
        
    end
end

