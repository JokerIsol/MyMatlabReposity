function [EFF_MAX,EFF_MIN,EFF_MEAN,EFF_BAD_SIZE]=test()
clear
clc
global POINT;
global POINT_INIT;
global POINT_INIT_URT;
EFF_BAD_SIZE=0;
for j=1:8
    init_point();
    for i=1:8
        R_T_PSO = pso(@testfunc,6);    
        R = ([R_T_PSO(6),R_T_PSO(5),R_T_PSO(4)]);
        T = [R_T_PSO(1),R_T_PSO(2),R_T_PSO(3)]';
        POINT_INIT_PSO=R*POINT_INIT+repmat(T,[1,size(POINT_INIT,2)]);
        F = POINT-POINT_INIT_PSO;
        F2 = sqrt(sum(F.*F,1));
        F_U = POINT-POINT_INIT_URT;
        F2_U = sqrt(sum(F_U.*F_U,1));
        EFF(i) = sum(F2_U-F2);
    end
    EFF_BAD_SIZE = EFF_BAD_SIZE+size(EFF(EFF<0),2);
    EFF = EFF(EFF>0);
    EFF_MAX(j) = max(EFF);
    EFF_MIN(j) = min(EFF);
    EFF_MEAN(j) = mean(EFF);
end
end

