%dependensies
% 1) normalizeLogWeights.m
% 2) GaussianDensity.m

classdef hypothesisReduction
    %HYPOTHESISREDUCTION is class containing different hypotheses reduction
    %method 
    %PRUNE: prune hypotheses with small weights.
    %CAP:   keep M hypotheses with the highest weights and discard the rest. 
    %MERGE: merge similar hypotheses in the sense of small Mahalanobis
    %       distance.
    
    methods (Static)
        function [hypothesesWeight, multiHypotheses] = ...
                prune(hypothesesWeight, multiHypotheses, threshold)
            %PRUNE prunes hypotheses with small weights
            %INPUT: hypothesesWeight: the weights of different hypotheses
            %       in logarithmic scale --- (number of hypotheses) x 1
            %       vector
            %       multiHypotheses: (number of hypotheses) x 1 structure
            %       threshold: hypotheses with weights smaller than this
            %       threshold will be discarded --- scalar in logarithmic scale
            %OUTPUT:hypothesesWeight: hypotheses weights after pruning in
            %       logarithmic scale --- (number of hypotheses after
            %       pruning) x 1 vector   
            %       multiHypotheses: (number of hypotheses after pruning) x
            %       1 structure  
            
            no_hypo = size(multiHypotheses,1);
            final_hypothesesWeight = [];
            final_hypothesis = [];
            final_no_hypo = 0;
            for idx = 1:no_hypo
                weight = hypothesesWeight(idx,1);
                hypo = multiHypotheses(idx,1);
                if (weight >= threshold)
                    final_hypothesesWeight = [final_hypothesesWeight ; weight];
                    final_hypothesis = [final_hypothesis ; hypo];
                    final_no_hypo = final_no_hypo + 1;
                end
            end
            hypothesesWeight =  final_hypothesesWeight;
            multiHypotheses = final_hypothesis;
        end
        
        function [hypothesesWeight, multiHypotheses] = ...
                cap(hypothesesWeight, multiHypotheses, M)
            %CAP keeps M hypotheses with the highest weights and discard
            %the rest 
            %INPUT: hypothesesWeight: the weights of different hypotheses
            %       in logarithmic scale --- (number of hypotheses) x 1
            %       vector  
            %       multiHypotheses: (number of hypotheses) x 1 structure
            %       M: only keep M hypotheses --- scalar
            %OUTPUT:hypothesesWeight: hypotheses weights after capping in
            %       logarithmic scale --- (number of hypotheses after
            %       capping) x 1 vector 
            %       multiHypotheses: (number of hypotheses after capping) x
            %       1 structure 
            
            % keep the first M highest weights and discard the rest
            capped_hypothesesWeight = [];
            capped_multiHypotheses = [];
            % arrange the hypothesis weights in descending order
            %num_hypo = size(hypothesesWeight,1);
            
            [sorted_hypothesesWeight, Weight_index] = sort(hypothesesWeight,'descend');
            
            for idx = 1:M
                weight_index = Weight_index(idx,1);
                weight = sorted_hypothesesWeight(idx,1);
                hypothesis = multiHypotheses(weight_index,1);
                capped_hypothesesWeight = [capped_hypothesesWeight ; weight];
                capped_multiHypotheses = [capped_multiHypotheses ; hypothesis];
            end 
     
            hypothesesWeight = capped_hypothesesWeight;
            multiHypotheses = capped_multiHypotheses;
        end
        
        function [hypothesesWeight,multiHypotheses] = ...
                merge(hypothesesWeight,multiHypotheses,threshold,density)
            %MERGE merges hypotheses within small Mahalanobis distance
            %INPUT: hypothesesWeight: the weights of different hypotheses
            %       in logarithmic scale --- (number of hypotheses) x 1
            %       vector  
            %       multiHypotheses: (number of hypotheses) x 1 structure
            %       threshold: merging threshold --- scalar
            %       density: a class handle
            %OUTPUT:hypothesesWeight: hypotheses weights after merging in
            %       logarithmic scale --- (number of hypotheses after
            %       merging) x 1 vector  
            %       multiHypotheses: (number of hypotheses after merging) x
            %       1 structure 
            
            [hypothesesWeight,multiHypotheses] = ...
                density.mixtureReduction(hypothesesWeight,multiHypotheses,threshold);
            
        end   
    end
end
