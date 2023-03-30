function [C,Ceq]=mycon(x)
% function [C,Ceq,DC,DCeq]=mycon(x)
%无等式约束
Ceq = [];
% DCeq = [];
C = zeros(33,1);
% DCeq = [];
load('P1.mat');
load('P2.mat');

R = [cos(x(1))*cos(x(2)) cos(x(1))*sin(x(2))*sin(x(3))-sin(x(1))*cos(x(3)) cos(x(1))*sin(x(2))*cos(x(3))+sin(x(1))*sin(x(3));...
     sin(x(1))*cos(x(2)) sin(x(1))*sin(x(2))*sin(x(3))+cos(x(1))*cos(x(3)) sin(x(1))*sin(x(2))*cos(x(3))-cos(x(1))*sin(x(3));...
     -sin(x(2))        cos(x(2))*sin(x(3))                         cos(x(2))*cos(x(3))];
 
%不等式约束


for i = 1:1:11
    PONE = P1(i,:); Pone = P2(i,:);
    C(3*i-2) = (R(1,:)*PONE'+x(4)-Pone(1))^2-0.5^2;   
    C(3*i-1) = (R(2,:)*PONE'+x(5)-Pone(2))^2-0.5^2; 
    C(3*i) = (R(3,:)*PONE'+x(6)-Pone(3))^2-0.5^2;
%    
%     DC(3*i-2) =  diff(C(3*i-2),x);
%     DC(3*i-1) =  diff(C(3*i-1),x);
%     DC(3*i) =  diff(C(3*i),x);
 end 
end