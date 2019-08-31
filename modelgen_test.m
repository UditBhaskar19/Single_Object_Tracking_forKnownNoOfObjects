P_D = 0.9;
lambda_c = 30;
%2D Cartisian coordinate
range_c = [-100 100;-100 100];

obj = modelgen.sensormodel(P_D,lambda_c,range_c);

disp(obj.pdf_c)
disp(obj.intensity_c)

%1D Cartisian coordinate
range_c = [-100 100];

obj = modelgen.sensormodel(P_D,lambda_c,range_c);

disp(obj.pdf_c)
disp(obj.intensity_c)