function [R,T]=init_point()
global POINT;
global POINT_INIT;
global POINT_INIT_URT;
global R_T;
global W;
load('R_T.mat');
load('W.mat');
load('POINT.mat');
R=eul2rotm([R_T(6),R_T(5),R_T(4)]);
T=[R_T(1),R_T(2),R_T(3)]';
POINT_INIT_URT=POINT+sqrt(3)/3*(2*rand(size(POINT))-ones(size(POINT)));
POINT_INIT=R*POINT_INIT_URT;  %ÓÐËæ»úÎó²î
%POINT_INIT=R*POINT;  %ÎÞÎó²î
POINT_INIT(1,:)=POINT_INIT(1,:)+T(1);
POINT_INIT(2,:)=POINT_INIT(2,:)+T(2);
POINT_INIT(3,:)=POINT_INIT(3,:)+T(3);