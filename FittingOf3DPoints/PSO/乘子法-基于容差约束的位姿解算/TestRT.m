%检测R和T的精度
function [m,dif] = TestRT(R,T,Point_X,Point_Y,M)

sizeOfA = size(M); 
countOfRow = sizeOfA(1);  %数据表的行数

Point_Y_Caculate = zeros (countOfRow,3);

for i = 1:countOfRow
    Point_Y_Caculate(i,:) = ( R * Point_X(i,:)' + T )'; % 计算时注意行向量与列向量的转换
end
Point_Y_CaculateDif =  Point_Y_Caculate - Point_Y;
m = sum(sqrt(sum(Point_Y_CaculateDif.*Point_Y_CaculateDif,2)))/countOfRow;
dif = Point_Y_CaculateDif;
end
