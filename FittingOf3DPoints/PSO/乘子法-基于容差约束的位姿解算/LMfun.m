% 利用（拉格朗日）乘子法求解含约束的最优化问题
% x0=[0.696000000000000,-1.57000000000000,-0.783000000000000,-99,-100,-100];
%  
% [x,fval]=fmincon(@LMfun,x0,[],[],[],[],[],[],@mycon,OPTIONS )

function [f]=LMfun(x)
% function [f,df]=LMfun(x) 
f=0;
df = 0;
load('P1.mat');
load('P2.mat');
R = [cos(x(1))*cos(x(2)) cos(x(1))*sin(x(2))*sin(x(3))-sin(x(1))*cos(x(3)) cos(x(1))*sin(x(2))*cos(x(3))+sin(x(1))*sin(x(3));...
     sin(x(1))*cos(x(2)) sin(x(1))*sin(x(2))*sin(x(3))+cos(x(1))*cos(x(3)) sin(x(1))*sin(x(2))*cos(x(3))-cos(x(1))*sin(x(3));...
     -sin(x(2))        cos(x(2))*sin(x(3))                         cos(x(2))*cos(x(3))];
 Diff_x = zeros(22,1);
 Diff_y = zeros(22,1);
 Diff_z = zeros(22,1);
 for i = 1:22
    PONE = P1(i,:); Pone = P2(i,:);
    Diff_x(i) = (R(1,:)*PONE'+x(4)-Pone(1))^2;
    Diff_y(i) = (R(2,:)*PONE'+x(5)-Pone(2))^2;
    Diff_z(i) = (R(3,:)*PONE'+x(6)-Pone(3))^2;
    f = f + Diff_x(i)+ Diff_y(i)+Diff_z(i);
 end 
 
%  df = [diff(f,x(1));diff(f,x(2));diff(f,x(3));diff(f,x(5));diff(f,x(6))]; 
%  df = [diff(f,x(1));diff(f,x(2));diff(f,x(3));diff(f,x(4));diff(f,x(5));diff(f,x(6))];
 
end