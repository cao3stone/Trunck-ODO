function [t,acce_f,gyro_f,magn_f,acce_b,gyro_b,magn_b]=read_data_magn_odo(dt)


% str1="C:\Users\17816\Desktop\头部双磁\data\20240111-头盔\1\返\MT_012103B8-001-000_";
% % str1="C:\Users\17816\Desktop\头部双磁\data\20240626\直线\MT_012103B8-000-000_";
% % str1="C:\Users\17816\Desktop\头部双磁\data\20240626\折线\MT_012103B8-001-000_";
% % 
% str21="00B46150.txt";
% str22="00B431A0.txt";

% path=strcat(str1,str21);

% path="C:\Users\17816\Desktop\头部双磁\data\20240626\直线\MT_012103B8-000-000_00B431A0.txt";
path="C:\Users\17816\Desktop\躯干双磁论文\data\line\MT_012103B8-000-000_00B43298.txt";
data=readmatrix(path);
C=[0 0 1;0 1 0;-1 0 0];
acce_f=C*data(:,3:5)';
gyro_f=C*data(:,6:8)';
magn_f=C*data(:,9:11)';

% path=strcat(str1,str22);
% path="C:\Users\17816\Desktop\头部双磁\data\20240626\直线\MT_012103B8-000-000_00B431A0.txt";
path="C:\Users\17816\Desktop\躯干双磁论文\data\line\MT_012103B8-000-000_00B46152.txt";
data=readmatrix(path);
C=[0 0 -1;0 -1 0;-1 0 0];
acce_b=C*data(:,3:5)';
gyro_b=C*data(:,6:8)';
magn_b=C*data(:,9:11)';
t=(0:length(data)-1)*dt;

% path="C:\Users\17816\Desktop\躯干双磁\data\双磁测速实验\MT_012103B8-005-000_00B46150.txt";
% data=readmatrix(path);
% % t1=data(:,1);
% C=[0 0 1;0 1 0;-1 0 0];
% acce_f=C*data(11:end,2:4)';
% gyro_f=C*data(11:end,5:7)';
% magn_f=C*data(11:end,8:10)';
% 
% % path=strcat(str1,str22);
% path="C:\Users\17816\Desktop\躯干双磁\data\双磁测速实验\MT_012103B8-005-000_00B46149.txt";
% data=readmatrix(path);
% % t2=data(:,1);
% C=[0 0 -1;0 -1 0;-1 0 0];
% acce_b=C*data(:,2:4)';
% gyro_b=C*data(:,5:7)';
% magn_b=C*data(:,8:10)';

% acce_f1=[];
% gyro_f1=acce_f1;
% magn_f1=acce_f1;
% for k=1:length(t2)
%     num=find(t1==t2(k));
%     acce_f1(:,k)=acce_f(:,num);
%     gyro_f1(:,k)=gyro_f(:,num);
%     magn_f1(:,k)=magn_f(:,num);
% end
% acce_f=acce_f1;
% gyro_f=gyro_f1;
% magn_f=magn_f1;

t=(0:length(data)-1)*dt;

% plot(t(1:200),magn_f(1,1:200))
% hold on
% plot(t,magn_b(1,:))
% legend('front','back')
% 
% plot(t,magn_f(1,:))
% hold on
% plot(t,magn_b(1,:))

% plot(t(1000:5000),acce(1:2,1000:5000))
% legend('x','y')
% xlabel('Sampe Time(s)')
% ylabel('Acceleration(m/s^2)')
% tam=vecnorm(acce_f,2);
% plot(movmean(tam,20),'o')
end 
