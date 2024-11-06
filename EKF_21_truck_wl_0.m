function [X,cov]=EKF_21_truck_wl_0(dt,a,w,v_p_f,step_no)

n=15;   %状态量个数
N=length(a); %数据长度
Ta=1800;
Tg=1800;
%% 获得误差矩阵
[P,cov,Q,R_zupt,R_measure]=Error_init(n,N,Ta,Tg);

%% 获得初始状态
[X,a,w]=Nav_init(n,N,a,w);
yaw0=0;
Cb2h=euler2dcmR2b([X(7,1) X(8,1) yaw0])';
% Cb2h=eye(3);

%% 滤波
update_time=1;   %更新频率
steps=10;
tag=0;
k=2;
while k<N+1
    X(:,k)=IMU_update(X(:,k-1),a(:,k-1),a(:,k),w(:,k-1),w(:,k),dt);    %IMU状态递推
    [PHI,G]=get_matrix(X(:,k),dt,a(:,k),Ta,Tg);
    P=PHI*P*PHI'+G*Q*G'/dt;
    cov(:,k)=diag(P);
    if k>step_no(steps) && tag==0
        dyaw=atan2(X(2,k-1),X(1,k-1));
        Cb2h=euler2dcmR2b([X(7,1) X(8,1) dyaw])';
        [P,cov,Q,R_zupt,R_measure]=Error_init(n,N,Ta,Tg);
        tag=1;
        k=2;
        continue
    end
    if v_p_f(k)==0 && abs(norm(a(:,k))-gravity(40,40))<0.2
        [H,dz]=get_measure_zupt(X(:,k-1),X(:,k));
        [X(:,k),P,cov(:,k)]=KF_update(P,H,dz,R_zupt,X(:,k));
    % elseif if_step(k)==1
    elseif mod(k,update_time)==0 %&& abs(X(9,k)-X(9,k-1))<0.3*pi/180
        [H,dz]=get_measure(X(:,k),v_p_f(:,k),Cb2h,w(:,k));
        [X(:,k),P,cov(:,k)]=KF_update(P,H,dz,R_measure,X(:,k));
    end
    k=k+1;
end
% eulerw=dcm2euler(Cb2h)*180/pi;
% t=(1:N)*dt;
% w_h=Cb2h*w;
% % dw=w_h-w;
% % plot(t,dw(3,:))
% plot(t,w(1,:))
% hold on
% plot(t,w_h(1,:))
% legend('b','h')
end

%% 获得误差矩阵
function [P,cov,Q,R_zupt,R_measure]=Error_init(n,N,Ta,Tg)

%状态量的协方差矩阵
% sigma_p_intial=1e-5*[1 1 1]';
% sigma_v_intial=1e-5*[1 1 1]';
% sigma_a_intial=0.1*pi/180*[1 1 1]';
% sigma_ba=0.1*[1 1 1]';
% sigma_bg=0.1*pi/180*[1 1 1]';
sigma_p_intial=1e-5*[1 1 1]';
sigma_v_intial=1e-5*[1 1 1]';
sigma_a_intial=0.1*pi/180*[1 1 1]';
sigma_ba=0.04*[1 1 1]';
sigma_bg=0.02*pi/180*[1 1 1]';
P=diag([sigma_p_intial;sigma_v_intial;sigma_a_intial;sigma_ba;sigma_bg].^2);
cov=zeros(n,N);
cov(:,1)=diag(P);

%状态噪声矩阵
% sigma_acce_noise=0.06*[1 1 1]';
% sigma_gyro_noise=0.05*pi/180*[1 1 1]';
% sigma_ba_driving_noise=0.01*[1 1 1]';
% sigma_bg_driving_noise=0.01*pi/180*[1 1 1]';
sigma_acce_noise=0.08*[1 1 1]';
sigma_gyro_noise=0.05*pi/180*[1 1 1]';
sigma_ba_driving_noise=0.01*[1 1 1]';
sigma_bg_driving_noise=0.01*pi/180*[1 1 1]';

Q=diag([sigma_acce_noise;sigma_gyro_noise;sigma_ba_driving_noise;sigma_bg_driving_noise].^2);
Q(7:9,7:9)=2*Q(7:9,7:9)/Ta;
Q(10:12,10:12)=2*Q(10:12,10:12)/Tg;
%量测噪声矩阵
% sigma_zupt_p=0.01*[1 1 1]';
% sigma_zupt=0.01*[1 1 1]';
% sigma_zaru=0.01*pi/180*[1 1 1]';
% sigma_v=0.1*[1 1 1]';
sigma_zupt_p=0.01*[1 1 1]';
sigma_zupt=0.01*[1 1 1]';
sigma_zaru=0.01*pi/180*[1 1 1]';
sigma_v=0.2*[1 1 1]';

R_zupt=diag([sigma_zupt_p;sigma_zupt;sigma_zaru].^2);
R_measure=diag(sigma_v.^2);

end

%% 导航状态初始化
function [X,a,w]=Nav_init(n,N,a,w)

X=zeros(n,N);

a_x=mean(a(1,1:100));
a_y=mean(a(2,1:100));
a_z=mean(a(3,1:100));
roll=atan2(-a_y,-a_z);
pitch=atan2(a_x,sqrt(a_y*a_y+a_z*a_z));
yaw=0;
euler0=[roll pitch yaw]';
X(1:3,1)=[0 0 0]';
X(4:6,1)=[0 0 0]';
X(7:9,1)=euler0;
end

%% 获得转移矩阵矩阵
function [PHI,G]=get_matrix(X,dt,ak,Ta,Tg)

Cb2n=euler2dcmR2b(X(7:9))';

fnk=Cb2n*ak;
fnk_vec=get_vec(fnk);
F=zeros(length(X));
F(1:3,4:6)=eye(3);
F(4:6,7:9)=fnk_vec;
F(4:6,10:12)=Cb2n;
F(7:9,13:15)=-Cb2n;
F(10:12,10:12)=-1/Ta*eye(3);
F(13:15,13:15)=-1/Tg*eye(3);
% PHI=[O eye(3)     O           O         O;
%          O    O    fnk_vec    Cb2n      O   ;
%          O    O         O           O     -Cb2n;
%          O    O         O        -1/Ta      O;
%          O    O         O           O      -1/Tg];
PHI=eye(length(X))+F*dt;

Gc=zeros(length(X),12);
Gc(4:6,1:3)=Cb2n;
Gc(7:9,4:6)=-Cb2n;
Gc(10:12,7:9)=eye(3);
Gc(13:15,10:12)=eye(3);
% Gc=[O      O     O O; 
%     Cb2n    O     O O; 
%         O -Cb2n   O O; 
%         O     O       I  O; 
%         O     O      O  I];
G=dt*Gc;

end
%% 获得观测
function [H,dz]=get_measure(X,v_p_f,Cb2h,wk)

H=zeros(3,length(X));

Cn2b=euler2dcmR2b(X(7:9));

Cn2h=Cb2h*Cn2b;
H(1:3,4:6)=Cn2h;
H(1:3,7:9)=-Cn2h*get_vec(X(4:6));
% tilde-观测 
% hat-预测
wk=Cb2h*wk;
l=1.77;
% v_p_tilde=[v_p_f 0 0]';  
v_p_tilde=[v_p_f wk(1)*l 0]';  
v_p_hat=Cn2h*X(4:6);  
dz=v_p_tilde-v_p_hat;
dz(1)=0;
end
%% zupt
function [H,dz]=get_measure_zupt(Xk_1,Xk)

H=zeros(9,length(Xk));
H(1:3,1:3)=eye(3);
dz_p=Xk_1(1:3)-Xk(1:3);

H(4:6,4:6)=eye(3);
dz_v=-Xk(4:6);

Cb2n=euler2dcmR2b(Xk(7:9))';
C11=Cb2n(1,1);
C21=Cb2n(2,1);
C31=Cb2n(3,1);
H(9,7:9)=[C11*C31/(C11*C11+C21*C21) C21*C31/(C11*C11+C21*C21) -1];
dz_atti=[0 0 Xk_1(9)-Xk(9)]';

dz=[dz_p;dz_v;dz_atti];

end

function [X,P,cov]=KF_update(P,H,dz,R,X)

    K=(P*H')/(H*P*H'+R);
    dx=K*dz;
    
    % d_atti=get_vec(dx(7:9));
    % C=(eye(3)-d_atti)*euler2dcmR2b(X(7:9,k))';
    
    Cn=euler2dcmR2b(dx(7:9));
    Cb2n=euler2dcmR2b(X(7:9))';       
    C=Cn*Cb2n;

    X=X+dx;

    X(7:9)=dcmb2R2euler(C);
    
    P=(eye(length(P))-K*H)*P*(eye(length(P))-K*H)'+K*R*K';
    
    cov=diag(P);

end

%% 惯性递推
function [Xk]=IMU_update(Xk_1,fk_1,fk,wk_1,wk,dt)
Xk=zeros(15,1);
fk=fk+Xk_1(10:12);
wk=wk+Xk_1(13:15);

g0=gravity(40,40);
g=[0 0 g0]';
Cb2nk_1=euler2dcmR2b(Xk_1(7:9))';
d_thetak_1=wk_1*dt;
d_thetak=wk*dt;

phi=d_thetak+cal_vec(d_thetak_1,d_thetak)/12;
Cb=rot2dcm(phi);
Cb2nk=Cb2nk_1*Cb; %Cb2nk=Cn*Cb2nk-1*Cb  认为Cn=I

Xk(7:9)=dcmb2R2euler(Cb2nk);

d_vk_1=fk_1*dt;
d_vk=fk*dt;

vfk=d_vk+cal_vec(d_thetak,d_vk)/2+(cal_vec(d_thetak_1,d_vk)+cal_vec(d_vk_1,d_thetak))/12;
vcor=g*dt;
Xk(4:6)=Xk_1(4:6)+vcor+Cb2nk_1*vfk;

Xk(1:3)=Xk_1(1:3)+Xk(4:6)*dt;

Xk(10:15)=Xk_1(10:15);

end

