function [v_p,step_No]=Step_Detection_Length(Acce,Gyro,Time,dt)

N=length(Acce);

a_x=mean(Acce(1,1:100));
a_y=mean(Acce(2,1:100));
a_z=mean(Acce(3,1:100));

% 
% plot(Time,Acce)
% legend('x','y','z')

roll=atan2(-a_y,-a_z);
pitch=atan2(a_x,sqrt(a_y*a_y+a_z*a_z));
yaw=0;
% yaw=69.187*pi/180;
atti=[roll;pitch;yaw];

% 标定
g0=gravity(40,40);
% g=[0 0 g0]';
Cn2b=euler2dcmR2b(atti);
% l_b=Ch2b*l_h;
% ba=[a_x;a_y;a_z]+Cn2b*g;
% legend('x','y','z')
Acce=Cn2b'*Acce;
% Acce=Acce-ba;
% 
% plot(Time,Acce)

%%  脚步检测

Acce_norm=vecnorm(Acce);
Acce_norm=movmean(Acce_norm,20);
if_step=zeros(1,N);
step_cycle=if_step;
if_highest=if_step;
for i =2:N-1
    if (Acce_norm(i)>Acce_norm(i-1) && Acce_norm(i)>Acce_norm(i+1) && Acce_norm(i)>11)
        step_cycle(i)=1;
    end
end
for k=1:N
    if step_cycle(k)==1
        for m=k-50:k+50
            if (Acce_norm(m)<g0 && Acce_norm(m+1)>g0)
                if_step(m)=1;
            end
        end
    end
end

step_No=find(if_step==1);
for k=1:length(step_No)-1
    for i=step_No(k):step_No(k+1)
        if Acce_norm(i)>g0 && Acce_norm(i+1)<g0
            if_highest(i)=1;
            if_highest(i+1)=1;
        end
    end
end

% plot(Acce(3,:))
% hold on
% Acce(3,:)=lowpass(Acce(3,:),2,100);

% plot(Acce(3,:))
% Acce_n=Cn2b'*Acce;

% figure
% plot(Acce_norm)
% hold on
% A=Acce_norm.*if_step;
% plot(A,'ro')
% 
% plot(Gyro(2,:))
% % % plot(Acce(1,:))
% 
% A1=Gyro(2,:).*if_highest;
% plot(A1,'bo')
%% 步长估计+计算前向速度
n=length(step_No); Step_length=zeros(1,n-1); v_p=zeros(1,length(Acce));
for k=1:(n-1)
    % Acce_max=max(Acce_norm(step_No(k):step_No(k+1))); %
    % Acce_min=min(Acce_norm(step_No(k):step_No(k+1)));
    % Step_length(k)=0.38*(Acce_max-Acce_min)^(1/4);
    Acce_max=max(Acce(3,step_No(k):step_No(k+1)));
    Acce_min=min(Acce(3,step_No(k):step_No(k+1))); %
    Step_length(k)=0.4*(Acce_max-Acce_min)^(1/4);
    t_start=step_No(k);
    t_end=step_No(k+1); 
    t=(t_end-t_start)*dt;
    if t<5
        v_p(t_start:t_end)=Step_length(k)/t;
    end
end

% h=1.77;
% aphi=0.2;
% beta=-3;
% v_p=zeros(1,length(Acce));
% for k=2:n
%     t_start=step_No(k-1);
%     t_end=step_No(k);
%     t=(t_end-t_start)*dt;
%     Step_length(k-1)=h*(1/t+aphi)+beta;
%     v_p(t_start:t_end)=Step_length(k-1)/t;
% end
% L=sum(Step_length)-400
% plot(Step_length)

%% 计算前向速度

% plot(No_step,'*')
% for k=2:n
%     t_start=step_No(k-1);
%     t_end=step_No(k);
%     t=(t_end-t_start)*dt;
%     v_p(t_start:t_end)=Step_length(k-1)/t;
% end

end

function g=gravity(lambda,h)
lambda=lambda*pi/180;
gamma=9.780327*(1+0.0053024*sin(lambda)^2-0.0000058*sin(2*lambda)^2);
g=gamma-((3.0877e-6)-(0.004e-6)*sin(lambda)^2)*h+(0.072e-12)*h^2;
end