function [X_f,X_b,cov_f,cov_b,X_o]=EKF_42_truck(dt,a_f,w_f,a_b,w_b,v_p,step_no)

n=21;   %状态量个数
N=length(a_f); %数据长度
Tba=1800;
Tbg=1800;
Tsa=1800;
Tsg=1800;
%% 获得误差矩阵
[P_f,P_b,cov_f,cov_b,Q,R_zupt,R_measure]=Error_init(n,N,Tba,Tbg,Tsa,Tsg);

%% 获得初始状态
X_f=Nav_init(n,N,a_f);
X_b=Nav_init(n,N,a_b);
% r1=[0.13;0;-0.025];
% r1=[0.13;0;0];
r1=[0.26 0.045 0.03]'/2;
r2=-r1;
X_f(1:3,1)=r1;
X_b(1:3,1)=r2;
yaw0=0;
Cb2h_f=euler2dcmR2b([X_f(7,1) X_f(8,1) yaw0])';
Cb2h_b=euler2dcmR2b([X_b(7,1) X_b(8,1) yaw0])';

%% 滤波
update_time=1;   %更新频率
steps=10;
tag=0;
k=2;
while k<N+1
    %% IMU1
    X_f(:,k)=IMU_update(X_f(:,k-1),a_f(:,k-1),a_f(:,k),w_f(:,k-1),w_f(:,k),dt);    %IMU状态递推
    [PHI_f,G_f]=get_matrix(X_f(:,k),dt,a_f(:,k),w_f(:,k),Tba,Tbg,Tsa,Tsg);
    P_f=PHI_f*P_f*PHI_f'+G_f*Q*G_f'/dt;
    cov_f(:,k)=diag(P_f);
    %% IMU2
    X_b(:,k)=IMU_update(X_b(:,k-1),a_b(:,k-1),a_b(:,k),w_f(:,k-1),w_b(:,k),dt);    %IMU状态递推
    [PHI_b,G_b]=get_matrix(X_b(:,k),dt,a_b(:,k),w_b(:,k),Tba,Tbg,Tsa,Tsg);
    P_b=PHI_b*P_b*PHI_b'+G_b*Q*G_b'/dt;
    cov_b(:,k)=diag(P_b);
    if k>step_no(steps) && tag==0
        dyaw_f=atan2((X_f(2,k-1)-X_f(2,1)),(X_f(1,k-1)-X_f(1,1)));
        Cb2h_f=euler2dcmR2b([X_f(7,1) X_f(8,1) dyaw_f])';
        dyaw_b=atan2((X_b(2,k-1)-X_b(2,1)),(X_b(1,k-1)-X_b(1,1)));
        Cb2h_b=euler2dcmR2b([X_b(7,1) X_b(8,1) dyaw_b])';
        [P_f,P_b,cov_f,cov_b,Q,R_zupt,R_measure]=Error_init(n,N,Tba,Tbg,Tsa,Tsg);
        tag=1;
        k=2;
        continue
    end
    if v_p(k)==0 && abs(norm(a_f(:,k))-gravity(40,40))<0.2 && abs(norm(a_b(:,k))-gravity(40,40))<0.2
        [H,dz]=get_measure_zupt(X_f(:,k-1),X_f(:,k));
        [X_f(:,k),P_f,cov_f(:,k)]=KF_update(P_f,H,dz,R_zupt,X_f(:,k));
        [H,dz]=get_measure_zupt(X_b(:,k-1),X_b(:,k));
        [X_b(:,k),P_b,cov_b(:,k)]=KF_update(P_b,H,dz,R_zupt,X_b(:,k));
    elseif mod(k,update_time)==0 %&& abs(X(9,k)-X(9,k-1))<0.3*pi/180
    % elseif v_p(1,k)==1
        [H_f,dz_f]=get_measure(X_f(:,k),X_b(:,k),v_p(:,k),Cb2h_f,w_f(:,k),r1);
        [H_b,dz_b]=get_measure(X_b(:,k),X_f(:,k),v_p(:,k),Cb2h_b,w_b(:,k),r2);
  
       dz_f_0=dz_f(1)/sqrt(cov_f(4,k));
       dz_b_0=dz_b(1)/sqrt(cov_b(4,k));
       if abs(dz_f_0)<50 && abs(dz_b_0)<50
            [X_f(:,k),P_f,cov_f(:,k)]=KF_update(P_f,H_f,dz_f,R_measure,X_f(:,k));
            [X_b(:,k),P_b,cov_b(:,k)]=KF_update(P_b,H_b,dz_b,R_measure,X_b(:,k));
        end
    end
    k=k+1;
end
X_o=0.5*(X_f(1:3,:)+X_b(1:3,:));
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
function [P1,P2,cov1,cov2,Q,R_zupt,R_measure]=Error_init(n,N,Tba,Tbg,Tsa,Tsg)

%状态量的协方差矩阵
sigma_p_intial=1e-5*[1 1 1]';
sigma_v_intial=1e-5*[1 1 1]';
sigma_a_intial=0.1*pi/180*[1 1 1]';
sigma_ba=0.04*[1 1 1]';
sigma_bg=0.02*pi/180*[1 1 1]';
sigma_sa=0.04*[1 1 1]';
sigma_sg=0.02*pi/180*[1 1 1]';
P1=diag([sigma_p_intial;sigma_v_intial;sigma_a_intial;sigma_ba;sigma_bg;sigma_sa;sigma_sg].^2);cov1=zeros(n,N);
cov1(:,1)=diag(P1);
P2=P1;
cov2=cov1;

%状态噪声矩阵
sigma_acce_noise=0.025*[1 1 1]';
sigma_gyro_noise=0.01*pi/180*[1 1 1]';
sigma_ba_driving_noise=0.001*[1 1 1]';
sigma_bg_driving_noise=0.001*pi/180*[1 1 1]';
sigma_sa_driving_noise=0.01*[1 1 1]';
sigma_sg_driving_noise=0.01*pi/180*[1 1 1]';

Q=diag([sigma_acce_noise;sigma_gyro_noise;sigma_ba_driving_noise;sigma_bg_driving_noise;sigma_sa_driving_noise;sigma_sg_driving_noise].^2);
Q(7:9,7:9)=2*Q(7:9,7:9)/Tba;
Q(10:12,10:12)=2*Q(10:12,10:12)/Tbg;
Q(13:15,13:15)=2*Q(7:9,7:9)/Tsa;
Q(16:18,16:18)=2*Q(10:12,10:12)/Tsg;
%量测噪声矩阵
sigma_zupt_p=0.01*[1 1 1]';
sigma_zupt=0.01*[1 1 1]';
sigma_zaru=0.01*pi/180*[1 1 1]';
sigma_v=0.01*[1 1 1]';
sigma_r=0.01*[1 1 1]';

R_zupt=diag([sigma_zupt_p;sigma_zupt;sigma_zaru].^2);
R_measure=diag([sigma_v;sigma_r].^2);

end

%% 导航状态初始化
function X=Nav_init(n,N,a)

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
function [PHI,G]=get_matrix(X,dt,ak,wk,Tba,Tbg,Tsa,Tsg)

Cb2n=euler2dcmR2b(X(7:9))';

fnk=Cb2n*ak;
fnk_vec=get_vec(fnk);
F=zeros(length(X));
F(1:3,4:6)=eye(3);
F(4:6,7:9)=fnk_vec;
F(4:6,10:12)=Cb2n;
F(4:6,16:18)=Cb2n*diag(ak);
F(7:9,13:15)=-Cb2n;
F(7:9,19:21)=-Cb2n*diag(wk);
F(10:12,10:12)=-1/Tba*eye(3);
F(13:15,13:15)=-1/Tbg*eye(3);
F(16:18,16:18)=-1/Tsa*eye(3);
F(19:21,19:21)=-1/Tsg*eye(3);
% PHI=[O    eye(3)       O           O         O               O                      O;
%          O       O      fnk_vec    Cb2n       O      Cb2n*diag(f_b)         O;
%          O       O           O           O      -Cb2n           O            -Cb2n*diag(w_b);
%          O       O           O        -1/Ta       O               O                      O;
%          O       O           O           O       -1/Tg            O                      O;
%          O       O           O           O          O            -1/Tg                   O;
%          O       O           O           O          O               O]                  -1/Tg;
PHI=eye(length(X))+F*dt;

Gc=zeros(length(X),18);
Gc(4:6,1:3)=Cb2n;
Gc(7:9,4:6)=-Cb2n;
Gc(10:12,7:9)=eye(3);
Gc(13:15,10:12)=eye(3);
Gc(16:18,13:15)=eye(3);
Gc(19:21,16:18)=eye(3);
% Gc=[O      O     O     O     O    O; 
%     Cb2n    O     O     O     O    O; 
%         O -Cb2n   O     O     O    O; 
%         O     O       I      O     O    O; 
%         O     O      O      I      O    O;
%         O     O      O     O      I     O;
%         O     O      O     O      O    I;
G=dt*Gc;

end
%% 获得观测
function [H,dz]=get_measure(X1,X2,v_p,Cb2h,wk,r)

H=zeros(6,length(X1));

Cn2b=euler2dcmR2b(X1(7:9));
Cn2h=Cb2h*Cn2b;
wk=Cb2h*wk;
% tilde-观测 
% hat-预测
H(1:3,4:6)=Cn2h;
H(1:3,7:9)=-Cn2h*get_vec(X1(4:6));
l=1.77;
v_p_tilde=[v_p wk(1)*l 0]';  
v_p_hat=Cn2h*X1(4:6);  
dz_v=v_p_tilde-v_p_hat;
% H(1:3,4:6)=Cn2h;
% H(1:3,7:9)=-Cn2h*get_vec(X1(4:6));
% H(1:3,13:15)=get_vec(r)*Cb2h;
% l=1.77;
% v_p_tilde=[v_p wk(1)*l 0]';  
% v_p_hat=Cn2h*X1(4:6)+get_vec(r)*wk;
% dz_v=v_p_tilde-v_p_hat;

H(4:6,1:3)=eye(3);
H(4:6,7:9)=get_vec(Cn2h'*r);

r_tilde=0.5*(X1(1:3)+X2(1:3));
r_hat=X1(1:3)-Cn2h'*r;

dz_r=r_tilde-r_hat;

dz=[dz_v;dz_r];

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
Xk=zeros(21,1);
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

Xk(10:21)=Xk_1(10:21);

end

