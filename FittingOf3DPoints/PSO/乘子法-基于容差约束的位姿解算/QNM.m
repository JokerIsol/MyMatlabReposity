%%以SVD法解为初值
%%x0=[-0.0997611561407386;-0.109467433223369;-0.0414155321773056,;-990.180807105698;-424.101138987196;-585.289760886378];% 迭代初始值 
%%拟牛顿法[xx,fval,exitflag] = fminunc(@QNM,x0')
function f=QNM(x) 
load('P_measure.mat');
load('P_original.mat');
f=0;
R = [cos(x(1))*cos(x(2)) cos(x(1))*sin(x(2))*sin(x(3))-sin(x(1))*cos(x(3)) cos(x(1))*sin(x(2))*cos(x(3))+sin(x(1))*sin(x(3));...
     sin(x(1))*cos(x(2)) sin(x(1))*sin(x(2))*sin(x(3))+cos(x(1))*cos(x(3)) sin(x(1))*sin(x(2))*cos(x(3))-cos(x(1))*sin(x(3));...
     -sin(x(2))        cos(x(2))*sin(x(3))                         cos(x(2))*cos(x(3))];
 Diff_x = zeros(22,1);
 Diff_y = zeros(22,1);
 Diff_z = zeros(22,1);
 for i = 1:22
    PONE = P_measure(i,:); Pone = P_original(i,:);
    Diff_x(i) = (R(1,:)*PONE'+x(4)-Pone(1))^2;
    Diff_y(i) = (R(2,:)*PONE'+x(5)-Pone(2))^2;
    Diff_z(i) = (R(3,:)*PONE'+x(6)-Pone(3))^2;
    f = f + Diff_x(i)+ Diff_y(i)+Diff_z(i);
 end 
end