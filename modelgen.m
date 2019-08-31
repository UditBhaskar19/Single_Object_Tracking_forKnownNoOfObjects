classdef modelgen
    %MODELGEN is a class used to create the tracking model
    
    methods (Static)
        function obj = sensormodel(P_D,lambda_c,range_c)
            %SENSORMODEL creates the sensor model
            %INPUT:  P_D: object detection probability --- scalar
            %        lambda_c: average number of clutter measurements per
            %        time scan, Poisson distributed --- scalar 
            %        range_c: range of surveillance area --- if 2D model: 2
            %        x 2 matrix of the form [xmin xmax;ymin ymax]; if 1D
            %        model: 1 x 2 vector of the form [xmin xmax]  
            %OUTPUT: obj.pdf_c: uniform clutter density --- scalar
            %        obj.P_D: same as P_D 
            %        obj.lambda_c: same as lambda_c
            %        obj.range_c: same as range_c
            %        obj.intensity_c: uniform clutter intensity --- scalar
            
            obj.P_D = P_D;             %probability of detection of that object
            obj.lambda_c = lambda_c;   %clutter rate
            obj.range_c = range_c;     %FOV of the sensor
            
            % implement uniform clutter intensity
            [range_c_row, range_c_col] = size(range_c);
            volume_FOV = 1;
            for i = 1:range_c_row
                volume_FOV = volume_FOV * (range_c(i,2) - range_c(i,1));
            end
            obj.intensity_c = obj.lambda_c*(1/volume_FOV);
            
            %uniform clutter density
            obj.pdf_c = 1/volume_FOV;
            

        end
        
        function obj = groundtruth(nbirths,xstart,tbirth,tdeath,K)
            %GROUNDTRUTH specifies the parameters to generate groundtruth
            %INPUT:  nbirths: number of objects hypothesised to exist from
            %        time step 1 to time step K --- scalar 
            %        xstart: object initial state --- (object state
            %        dimension) x nbirths matrix 
            %        tbirth: object birth (appearing) time --- (total number
            %        of objects existed in the scene) x 1 vector  
            %        tdeath: the last time the object exists ---  (total number
            %        of objects existed in the scene) x 1 vector 
            %        K: total tracking time --- scalar
            
            obj.nbirths = nbirths;
            obj.xstart = xstart;
            obj.tbirth = tbirth;
            obj.tdeath = tdeath;
            obj.K = K;
        end
        
    end

end
