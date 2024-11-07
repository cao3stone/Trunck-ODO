addpath E:\研究生涯\6-code\自写\Tool_NED_FRD
%% 基本参数
fs=100;
dt=1/fs;
% gt=[0 54;0 0]*0.8;
% gt=[0 26 26 57;0 0 -11 -11]*0.8;
gt=[0 68;0 0]*0.8;
%% 读取数据
[t,a_f,w_f,m_f,a_b,w_b,m_b]=read_data_magn_odo(dt);

%% 脚步检测
[v_p_f,step_No_f]=Step_Detection_Length(a_f,w_f,t,dt);
[v_p_b,step_No_b]=Step_Detection_Length(a_b,w_b,t,dt);
if_static=~(v_p_b==0);
% plot(v_p_b)
%% 惯性解算（单独、无速度）
[X_f,~]=EKF_21_truck_wl(dt,a_f,w_f,v_p_f,step_No_f);
[X_b,~]=EKF_21_truck_wl(dt,a_b,w_b,v_p_b,step_No_b);

%% 初始姿态估计
C=euler2dcmR2b(X_f(7:9,1))';
m_f_t_0=C*m_f;

C=euler2dcmR2b(X_b(7:9,1))';
m_b_t_0=C*m_b;

%% 梯度分析
% cal_gradient(m_f(1:3,:),m_b(1:3,:),t,t);

%% 磁场数据分析
mean_number=50;
% m_f=movmean(m_f,mean_number,2);
% m_b=movmean(m_b,mean_number,2);
m_f_t_0=movmean(m_f_t_0,mean_number,2);
m_b_t_0=movmean(m_b_t_0,mean_number,2);

%% 速度对齐
v=gradient2(m_f_t_0.*if_static,m_b_t_0.*if_static,3);

d_static=diff(if_static);
static_start=find(d_static==1);
static_end=find(d_static==-1);
v_p=zeros(1,length(a_f));
v_p(static_start+1:static_end)=v;
plot(v)

%% 惯性解算（单独、有速度）
[X_f,~]=EKF_21_truck_wl(dt,a_f,w_f,v_p,step_No_f);
[X_b,~]=EKF_21_truck_wl(dt,a_b,w_b,v_p,step_No_b);
figure
plot(gt(2,:),gt(1,:),'k')
hold on
plot(X_f(2,:),X_f(1,:),'r')
plot(X_b(2,:),X_b(1,:),'b')
% plot(X_o(2,:),X_o(1,:),'g')
axis equal
legend('truth','front','back')

%% 惯性解算（联合）
[X_f,X_b,cov_f,cov_b,X_o]=EKF_42_truck(dt,a_f,w_f,a_b,w_b,v_p,step_No_f);
% plot(t,X_f(4:6,:))
%% 绘图
figure
plot(gt(2,:),gt(1,:),'k')
hold on
plot(X_f(2,:),X_f(1,:),'r')
plot(X_b(2,:),X_b(1,:),'b')
plot(X_o(2,:),X_o(1,:),'g')
axis equal
% legend('truth','front','back')
legend('truth','front','back','O')

% plot(t,cov_f(4:6,:))
% legend('x','y','z')
% plot(t,cov_b(4:6,:))
% legend('x','y','z')
% plot(t,X_b(7:9,:)*180/pi)
% legend('roll','pitch','yaw')

% dyaw=atan2(X4(2,10),X4(1,10));
% C=[cos(dyaw) sin(dyaw);-sin(dyaw) cos(dyaw)];
% X4(1:2,:)=C*X4(1:2,:);
% [X,cov]=EKF_15_truck_wl(dt,a,w,v_p,step_No,if_step,if_highest);

% dyaw=zeros(1,length(X));
% for k=2:length(X)
%     dyaw(k)= abs(X(9,k)-X(9,k-1));
%     if abs(dyaw(k)-2*pi)<10*pi/180
%         dyaw(k)=2*pi-dyaw(k);
%     end
% end
% plot(dyaw*180/pi)

% figure
% plot(gt(2,:),gt(1,:),'k','LineWidth',2)
% hold on
% % plot(p1(2,:),p1(1,:),'r')
% % plot(p2(2,:),p2(1,:),'b')
% plot(X(2,:),X(1,:),'r','LineWidth',2)
% % plot(X_h(2,:),X_h(1,:),'b','LineWidth',2)
% % plot(X(2,step_No),X(1,step_No),'b*')
% % plot(X2(2,:),X2(1,:),'b','LineWidth',2)
% % plot(X_spine(2,:),X_spine(1,:),'r')
% axis equal
% legend('Ground truth','Track')
% % legend('Ground truth','Track-b','Track-h')
% % legend('Ground truth','Track','Track_l')
% % text(gt(1,1),gt(2,1),['(',num2str(gt(1,1)),',',num2str(gt(2,1)),')'])
% % text(gt(2,2),gt(1,2),['(',num2str(gt(2,2)),',',num2str(gt(1,2)),')'])
% % text(X(2,end),X(1,end),['(',num2str(X(2,end)),',',num2str(X(1,end)),')'])
% % text(X2(2,end),X2(1,end),['(',num2str(X2(2,end)),',',num2str(X2(1,end)),')'])
% % legend('Ground truth','Track','Track-filt')

% x0=zeros(2,length(X));
% for k=2:length(X)
%     x0(:,k)=x0(:,k-1)+[X(4,k)*dt;X(5,k)*dt];
% end

% plot(x0(2,:),x0(1,:))
% plot3(X(2,:),X(1,:),X(3,:))
% axis equal
% plot3(X_spine(2,:),X_spine(1,:),X_spine(3,:))
% axis equal
% % 
% plot(gt(2,:),gt(1,:),'k')
% hold on
% plot(X2(2,:),X2(1,:),'r')
% plot(X_spine(2,:),X_spine(1,:),'b')
% axis equal
% legend('Ground truth','Track-head','Track-spine')
% 
% 
% figure
% plot(t,X_spine(4:6,:))
% legend('x','y','z')
% 
% figure
% plot(X(4,:))
% hold on
% plot(X1(4,:))
% figure
% plot(t,X_trunk_wl(4:6,:))
% legend('x','y','z')
% figure
% plot(t,X(7:9,:)*180/pi)
% legend('roll','pitch','yaw')
% figure
% plot(t,X(10:12,:))
% legend('x','y','z')
% figure
% plot(t,X(13:15,:))
% legend('x','y','z')
% figure
% plot(t,X(16:18,:))
% legend('x','y','z')

% figure
% plot(t,cov(1:3,:))
% legend('x','y','z')
% figure
% plot(t,cov(4:6,:))
% legend('x','y','z')
% figure
% plot(t,cov(7:9,:))
% legend('x','y','z')
% figure
% plot(t,cov(10:12,:))
% legend('x','y','z')
% figure
% plot(t,cov(13:15,:))
% legend('x','y','z')
% figure
% plot(t,cov(16:18,:))
% legend('x','y','z')

% plot(t,X1(4:6,:))
% legend('x','y','z')
% plot(t,X2(10:12,:))
% legend('x','y','z')
% plot(t,X2(16:18,:))
% legend('x','y','z')
% 
% plot(gt(2,:),gt(1,:),'k')
% hold on
% plot(X(2,:),X(1,:),'r')
% plot(X1(2,:),X1(1,:),'b')
% legend('Ground truth','Track-raw','Track-filt')
% axis equal

% figure(2)
% clf
% number=length(X);
% title('运动轨迹图')
% axis([min(X(2,:))-1,max(X(2,:))+1,min(X(1,:))-1,max(X(1,:))+1])
% xlabel('x [m]')
% ylabel('y [m]')
% axis equal
% h = animatedline;
% h0 = animatedline('Marker','o','Color','r');
% addpoints(h0,X(2,1),X(1,1))
% a=tic; %start timer
% for k =1:number
%     addpoints(h,X(2,k),X(1,k));
%     b=toc(a); %check timer
%     if b>(1/3000)
%         drawnow  % update screen
%         a=tic;   % reset timer after updating
%     end
% end
% drawnow  %draw final frame
% h1 = animatedline('Marker','*','Color','b');
% addpoints(h1,X(2,number),X(1,number))
% legend('运动轨迹','起点','终点')
% grid on
% box on