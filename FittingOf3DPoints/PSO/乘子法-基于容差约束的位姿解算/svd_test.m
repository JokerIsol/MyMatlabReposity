%% ���ô�Ȩֵ��SVD������������ĸ���ƥ������ Y = RT * X
function [R,T,err,dif] = svd_test(data)
%----------------------------------------------------------
% ��һ������excel�ļ��ж�ȡ����
%----------------------------------------------------------
A = data;

%----------------------------------------------------------
%�ڶ���������mat������Ԥ����,��ȡ��������Xi��ģ���������Yi����������ݽṹ�йأ�
%----------------------------------------------------------
sizeOfA = size(A); 
countOfRow = sizeOfA(1);  %���ݱ������

%����װ��Xi��Yi��N*3����
Point_X = zeros(countOfRow,3);
Point_Y = zeros(countOfRow,3);

%����Ȩֵ���������ȷ�Ϻ�����������
W = [1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 ...
   0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 ];

%W=ones(countOfRow,1);
LenOfW =length(W);
SumOfW = 0;
for i=1:LenOfW 
    SumOfW = SumOfW + W(i);
end
Weight=diag(W);

%������������
MassCenter_X = zeros(1,3);
MassCenter_Y = zeros(1,3);
for i = 1:countOfRow 
    for j = 1:3
        Point_X(i,j) = A(i,j);
        Point_Y(i,j) = A(i,j+4);
    end
end
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

%H = Point_X' * Point_Y;

H = Point_X'*Weight*Weight*Point_Y;


[U , S , V] = svd(H);  %X = U*S*V'
X = V * U';
if abs(det(X) - 1)< 1e-3 %���det(X)Լ����1
    R = X;   %������ת����
%     eul = tr2eul(R); %����robotics toolbox����ŷ����
%     [theta,vec] = tr2angvec(R); %����robotics toolbox������ת�����ת��
    T = MassCenter_Y' - R*MassCenter_X'; %����ƽ�ƾ���
%     CombineRT(R,T);
    [err,dif] = TestRT(R,T,Point_Xi,Point_Yi,A);
elseif abs(det(X) + 1)< 1e-3  %���det(X)Լ����-1
    [~,N] = eig(S);
    WrongFlag = 0;  %��¼S������ֵ�Ƿ�ȫ��Ϊ��
    for i = 1:3
        if N(i,i)==0
            WrongFlag = WrongFlag + 1;
            for j = 1:3
                S(i,j) = -S(i,j);
            end
        end
    end 
    if WrongFlag >= 1
        R = V' * U';   %������ת���� 
        T = MassCenter_Y' - R*MassCenter_X'; %����ƽ�ƾ���
%         CombineRT(R,T);
        [err,dif]  = TestRT(R,T,Point_Xi,Point_Yi,A);
    elseif  WrongFlag == 0
        disp('���棡�����棡�������޷�ƥ�������');
        err = Inf;
        R = 0;
        T = 0;
        dif = inf;
    end
    
%     x = zeros(6,1);
%     x(1) = 
    
end

