function dif = MonteCarlo()
%������������
load('P2');

[N,~] = size(P2);

%%����������
Err_Made = 1 * ones(N,3)+ 0.1 * rand(N,3);    %����������������
Err_Coor = 2.5 * ones(N,3)+ 0.5 * rand(N,3);     %Э��������������
Err = zeros(N,3);

for i = 1:N
     if i <= N/2
        Err(i,:) = Err_Made(i,:);
     else 
        Err(i,:) = Err_Coor(i,:);
     end
end


P_original = P2 + Err;      %�ɻ�����������װ����ʵ�ʲ���������۵�
P_measure = zeros(N,3);

eul_angle = [pi/30,pi/30,pi/60];
R = eul2rotm(eul_angle);
T = [1000;500;500];

for i = 1:N
    P_measure(i,:) = (R * P_original(i,:)' + T)'+ 0.05 * rand(1,3);  %����������
end

P_original = P2 ;    %��������У���Ȼ���������ݼ���

data = zeros(N,7);
for i =1:N
    for j = 1:7
       if j <= 3
          data(i,j) = P_measure(i,j);
       elseif j >= 5
          data(i,j) = P_original(i,j-4);
       end
     end
end
[R,T,err,dif1] = svd_test(data);
dif = dif1(:,1);
end

   
       
