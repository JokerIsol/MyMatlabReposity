%% 利用带权值的SVD法计算少量点的刚性匹配问题 Y = RT * X
function [R,T,err,dif] = svd_test(data)
%----------------------------------------------------------
% 第一步：从excel文件中读取数据
%----------------------------------------------------------
A = data;

%----------------------------------------------------------
%第二步：基于mat的数据预处理,提取理论坐标Xi和模拟测量坐标Yi（跟表格数据结构有关）
%----------------------------------------------------------
sizeOfA = size(A); 
countOfRow = sizeOfA(1);  %数据表的行数

%定义装载Xi和Yi的N*3矩阵
Point_X = zeros(countOfRow,3);
Point_Y = zeros(countOfRow,3);

%定义权值矩阵并输出，确认后程序继续运行
W = [1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 1.5/22 ...
   0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 0.5/22 ];

%W=ones(countOfRow,1);
LenOfW =length(W);
SumOfW = 0;
for i=1:LenOfW 
    SumOfW = SumOfW + W(i);
end
Weight=diag(W);

%定义质心坐标
MassCenter_X = zeros(1,3);
MassCenter_Y = zeros(1,3);
for i = 1:countOfRow 
    for j = 1:3
        Point_X(i,j) = A(i,j);
        Point_Y(i,j) = A(i,j+4);
    end
end
Point_Xi = Point_X; %储存原始数据
Point_Yi = Point_Y;

for j=1:3
    for i = 1:countOfRow        
        MassCenter_X(1,j) =  MassCenter_X(1,j) +  W(i)*Point_X(i,j);
        MassCenter_Y(1,j) =  MassCenter_Y(1,j) +  W(i)*Point_Y(i,j);
    end
end
MassCenter_X = MassCenter_X/(SumOfW);
MassCenter_Y = MassCenter_Y/(SumOfW);
%参考文献：飞机装配中大尺寸测量场的建立与优化技术
%将提取理论坐标Xi和模拟测量坐标Yi转化为减去质心的坐标，即定义两组新的坐标
for i = 1:countOfRow 
        Point_X(i,:) = Point_X(i,:) - MassCenter_X;  %矩阵行引用A(i,:)；列引用A(:,i)
        Point_Y(i,:) = Point_Y(i,:) - MassCenter_Y;
end

%----------------------------------------------------------
%第三步：基于SVD奇异值分解算法求解变换矩阵,计算R和T
%----------------------------------------------------------
%计算待分解H矩阵，其为3*3矩阵

%H = Point_X' * Point_Y;

H = Point_X'*Weight*Weight*Point_Y;


[U , S , V] = svd(H);  %X = U*S*V'
X = V * U';
if abs(det(X) - 1)< 1e-3 %如果det(X)约等于1
    R = X;   %计算旋转矩阵
%     eul = tr2eul(R); %利用robotics toolbox计算欧拉角
%     [theta,vec] = tr2angvec(R); %利用robotics toolbox计算旋转轴和旋转角
    T = MassCenter_Y' - R*MassCenter_X'; %计算平移矩阵
%     CombineRT(R,T);
    [err,dif] = TestRT(R,T,Point_Xi,Point_Yi,A);
elseif abs(det(X) + 1)< 1e-3  %如果det(X)约等于-1
    [~,N] = eig(S);
    WrongFlag = 0;  %纪录S的特征值是否全部为零
    for i = 1:3
        if N(i,i)==0
            WrongFlag = WrongFlag + 1;
            for j = 1:3
                S(i,j) = -S(i,j);
            end
        end
    end 
    if WrongFlag >= 1
        R = V' * U';   %计算旋转矩阵 
        T = MassCenter_Y' - R*MassCenter_X'; %计算平移矩阵
%         CombineRT(R,T);
        [err,dif]  = TestRT(R,T,Point_Xi,Point_Yi,A);
    elseif  WrongFlag == 0
        disp('警告！！警告！！出现无法匹配情况！');
        err = Inf;
        R = 0;
        T = 0;
        dif = inf;
    end
    
%     x = zeros(6,1);
%     x(1) = 
    
end

