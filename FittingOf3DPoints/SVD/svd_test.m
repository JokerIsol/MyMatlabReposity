%% ���ô�Ȩֵ��SVD������������ĸ���ƥ������ Pm = R*Po + T 
function [R,T,err,dif] = svd_test(pm,po)

sizeOfA = size(po); 
countOfRow = sizeOfA(1);  %���ݱ������

%����װ��Xi��Yi��N*3����
Point_X = po;
Point_Y = pm;

%����Ȩֵ���������ȷ�Ϻ�����������
%W = [1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 ...
   %0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 ];

W=ones(countOfRow,1);
%W = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,100];
LenOfW =length(W);
SumOfW = 0;
for i=1:LenOfW 
    SumOfW = SumOfW + W(i);
end
Weight=diag(W);

%������������
MassCenter_X = zeros(1,3);
MassCenter_Y = zeros(1,3);
Point_Xi = Point_X; %����ԭʼ����
Point_Yi = Point_Y;

for j=1:3
    for i = 1:countOfRow        
        MassCenter_X(1,j) =  MassCenter_X(1,j) +  W(i)*Point_X(i,j);
        MassCenter_Y(1,j) =  MassCenter_Y(1,j) +  W(i)*Point_Y(i,j);
    end
end

MassCenter_X = MassCenter_X/(SumOfW);
MassCenter_Y = MassCenter_Y/(SumOfW);
%�ο����ף��ɻ�װ���д�ߴ�������Ľ������Ż�����
%����ȡ��������Xi��ģ���������Yiת��Ϊ��ȥ���ĵ����꣬�����������µ�����
for i = 1:countOfRow 
        Point_X(i,:) = Point_X(i,:) - MassCenter_X;  %����������A(i,:)��������A(:,i)
        Point_Y(i,:) = Point_Y(i,:) - MassCenter_Y;
end

%----------------------------------------------------------
%������������SVD����ֵ�ֽ��㷨���任����,����R��T
%----------------------------------------------------------
%������ֽ�H������Ϊ3*3����


% W = ones(countOfRow,countOfRow);
% W = W/sum(sum(W));
% H = Point_X'*W*W*Point_Y;

H = Point_X'*Weight*Weight*Point_Y;


[U , S , V] = svd(H);  %X = U*S*V'
X = V * U';
R = X;
UT = U';
if(det(X)<-0.5)
	UT(3,1)= - UT(3,1);
    UT(3,2)= - UT(3,2);
    UT(3,3)= - UT(3,3);
    R = V * UT;
end
T = MassCenter_Y' - R*MassCenter_X';
[err,dif]  = TestRT(R,T,Point_Xi,Point_Yi,po);

% if abs(det(X) - 1)< 1e-3 %���det(X)Լ����1
%     R = X;   %������ת����
% %     eul = tr2eul(R); %����robotics toolbox����ŷ����
% %     [theta,vec] = tr2angvec(R); %����robotics toolbox������ת�����ת��
%     T = MassCenter_Y' - R*MassCenter_X'; %����ƽ�ƾ���
% %     CombineRT(R,T);
%     [err,dif]  = TestRT(R,T,Point_Xi,Point_Yi,po);
% elseif abs(det(X) + 1)< 1e-10  %���det(X)Լ����-1
%     %[~,N] = eig(S);
%     WrongFlag = 0;  %��¼S������ֵ�Ƿ�ȫ��Ϊ��
%     for i = 1:3
%        if abs(S(i,i))<=0.1
%             WrongFlag = WrongFlag + 1;
%             for j = 1:3
%                 V(i,j) = -V(i,j);
%             end
%         end
%     end 
%     if WrongFlag >= 1
%         R = V' * U';   %������ת���� 
%         T = MassCenter_Y' - R*MassCenter_X'; %����ƽ�ƾ���
% %         CombineRT(R,T);
%         [err,dif]  = TestRT(R,T,Point_Xi,Point_Yi,po);
%     elseif  WrongFlag == 0
%         disp('���棡�����棡�������޷�ƥ�������');
%         err = Inf;
%         R = zeros(3,3);
%         T = zeros(1,3);
%         dif = zeros(LenOfW,3);
%     end
% end   
%     x = zeros(6,1);
%     x(1) = 
    
end

