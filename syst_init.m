% Initialization code for Simulink file - initialize the observer matrices

% 
%
% [1] - Tucci,Meng,Guerrero,Ferrari-Trecate (2016), 'A consensus-based 
%       secondary voltage control layer for stable current sharing and 
%       voltage balancing in DC microgrids', Technical Report. URL:
%       http://arxiv.org/abs/1603.03624.
clear

%% Simulation parameters

% Total simulation time [s]
tTot = 20;
% Sample time [s]
tSamp = 0.0001;
tSampData = 1e-3;
% Simulation times [s]
tSim = 0:tSamp:tTot;
% Number of samples per sec
nSamp = 1/tSamp;
% Total number of samples
nTot = numel(tSim);

% Source Voltage
Vs = 60;

% Number of DGUs
N = 6;
% Number of states per DGU
n = 3;

% Time of connection of DGUs
Tc = 2; 
Ta = 8;
% Ts = tSim(end);
% Trep= 3;
% % Tc = 15;
% 
% atkMag = .01;
% 
% % Index at which connection occurs
tc = find(tSim==Tc);

% load('rbar_slowLuen_w0p05.mat')
load('rbar_slowLuen_w0p05_6by6UIO.mat')
% load('rbar_slowLuen.mat')
% load('rbar.mat')

%% Define DGU parameters
% Electrical components - Local parameters
DGU(1).Rt = 2.0e-1;
DGU(1).Ct = 2.2e-3;
DGU(1).Lt = 1.8e-3;

DGU(2).Rt = 3.0e-1;
DGU(2).Ct = 1.9e-3;
DGU(2).Lt = 2.0e-3;

DGU(3).Rt = 1.0e-1;
DGU(3).Ct = 1.7e-3;
DGU(3).Lt = 2.2e-3;

DGU(4).Rt = 5.0e-1;
DGU(4).Ct = 2.5e-3;
DGU(4).Lt = 3.0e-3;

% DGU(4).Rt = 3.0e-1;
% DGU(4).Ct = 1.9e-3;
% DGU(4).Lt = 2.0e-3;

DGU(5).Rt = 4.0e-1;
DGU(5).Ct = 2.0e-3;
DGU(5).Lt = 1.3e-3;

DGU(6).Rt = 6.0e-1;
DGU(6).Ct = 3.0e-3;
DGU(6).Lt = 2.5e-3;

% Interconnections - compare with Stage 2 in [1]
DGU(1).Ni = [2,3,6];
DGU(2).Ni = [1,4];
DGU(3).Ni = [1,4];
DGU(4).Ni = [2,3,5];
DGU(5).Ni = [4,6];
DGU(6).Ni = [1,5];

% Line parameters
% Line 1 : connects DGUs (1,2)
% Line 2 : connects DGUs (1,3)
% Line 3 : connects DGUs (1,6)
% Line 4 : connects DGUs (2,4)
% Line 5 : connects DGUs (3,4)
% Line 6 : connects DGUs (4,5)
% Line 7 : connects DGUs (5,6)

% --1----------- 2 ------ 3 ------ 6 -----
DGU(1).Rij = [ 5.0e-2 ; 7.0e-2 ; 1.0e-1 ];
DGU(1).Lij = [ 2.1e-6 ; 1.8e-6 ; 2.5e-6 ];

% --2----------- 1 ------ 4 -----
DGU(2).Rij = [ 5.0e-2 ; 4.0e-2 ];
DGU(2).Lij = [ 2.1e-6 ; 2.3e-6 ];

% --3----------- 1 ------ 4 -----
DGU(3).Rij = [ 7.0e-2 ; 6.0e-2 ];
DGU(3).Lij = [ 1.8e-6 ; 1.0e-6 ];

% --4----------- 2 ------ 3 ------ 5 -----
DGU(4).Rij = [ 4.0e-2 ; 6.0e-2 ; 8.0e-2 ];
DGU(4).Lij = [ 2.3e-6 ; 1.0e-6 ; 1.8e-6 ];

% --5----------- 4 ------ 6 -----
DGU(5).Rij = [ 8.0e-2 ; 8.0e-2 ];
DGU(5).Lij = [ 1.8e-6 ; 3.0e-6 ];

% --6----------- 1 ------ 5 -----
DGU(6).Rij = [ 1.0e-1 ; 8.0e-2 ];
DGU(6).Lij = [ 2.5e-6 ; 3.0e-6 ];


% Primary controllers
DGU(1).K  = [-2.134,-0.163,13.553];
DGU(2).K  = [-0.869,-0.050,48.285];
DGU(3).K  = [-0.480,-0.108,30.673];
DGU(4).K  = [-6.990,-0.175,102.960];
% DGU(4).K  = [-0.869,-0.050,48.285];
DGU(5).K  = [-0.101,-0.010,16.393];
DGU(6).K  = [-2.134,-0.163,13.553];

% DGU(1).K  = [-2.134,-0.163];
% DGU(1).K(3) = (DGU(1).K(1)-1)*(DGU(1).K(2)-DGU(1).Rt)/DGU(1).Lt/4;
% DGU(2).K  = [-0.869,-0.050];
% DGU(2).K(3) = (DGU(2).K(1)-1)*(DGU(2).K(2)-DGU(2).Rt)/DGU(2).Lt/4;
% DGU(3).K  = [-0.480,-0.108];
% DGU(3).K(3) = (DGU(3).K(1)-1)*(DGU(3).K(2)-DGU(3).Rt)/DGU(3).Lt/4;
% DGU(4).K  = [-6.990,-0.175];
% DGU(4).K(3) = (DGU(4).K(1)-1)*(DGU(4).K(2)-DGU(4).Rt)/DGU(4).Lt/4;
% DGU(5).K  = [-0.101,-0.010];
% DGU(5).K(3) = (DGU(5).K(1)-1)*(DGU(5).K(2)-DGU(5).Rt)/DGU(5).Lt/4;
% DGU(6).K  = [-2.134,-0.163];
% DGU(6).K(3) = (DGU(6).K(1)-1)*(DGU(6).K(2)-DGU(6).Rt)/DGU(6).Lt/4;

% Secondary controller weights - aij = 1 , if j \in Ni
DGU(1).kI = .5;
DGU(2).kI = .5;
DGU(3).kI = .5;
DGU(4).kI = .5;
DGU(5).kI = .5;
DGU(6).kI = .5;
% Secondary controller rated currents
DGU(1).Ir = 1;
DGU(2).Ir = 1;
DGU(3).Ir = 1;
DGU(4).Ir = 1;
DGU(5).Ir = 1;
DGU(6).Ir = 1;

% Linear dynamics of DGUi
DGU = stateDyn(DGU,1);
DGU = stateDyn(DGU,2);
DGU = stateDyn(DGU,3);
DGU = stateDyn(DGU,4);
DGU = stateDyn(DGU,5);
DGU = stateDyn(DGU,6);

Aconn = zeros(n*N);
for i = 1:N
    for j = 1:N
        if size(find(DGU(i).Ni == j),2) ~= 0
            jj = find(DGU(i).Ni == j);
            Aconn(n*(i-1)+1:n*i,n*(j-1)+1:n*j) = DGU(i).Aij(:,n*(jj-1)+1:n*jj);
        elseif i == j 
            Aconn(3*(i-1)+1:3*i,3*(i-1)+1:3*i) = DGU(i).A;
        end
    end
end

% Adisc = zeros(n*N);
% for i = 1:N
%     tmpA = DGU(i).A;
%     %       Set Aii(1,1) = 0;
%     tmpA(1) = 0;
%     Adisc((i-1)*n+1:i*n,(i-1)*n+1:i*n) = tmpA;
% end

Adisc = blkdiag(DGU.Aii);

Aii = blkdiag(DGU.A);
Aij = Aconn-Aii;
B = blkdiag(DGU.B);
K = blkdiag(DGU.K);
M = blkdiag(DGU.M);
Ir = [DGU.Ir]';

%% Define local estimator parameters Li
%   There are multiple methods to do this... Choose one.
%       place( ) :  you can choose where the eigenvalues are - resulting AL
%                   is diagonal
%       lqr  ( ) :  Li designed using optimal linear-quadratic technique
for i = 1:N
%     DGU(i).Li = lqr((DGU(i).A+DGU(i).B*DGU(i).K)',DGU(i).C',eye(3),eye(3))';
    DGU(i).Li = place((DGU(i).A)',DGU(i).C',[-1e2,-5e1,-5e2])';
    DGU(i).LiDisc = place((Adisc(1:3,1:3))',DGU(i).C',[-1e2,-5e1,-5e2])';
    DGU(i).AL = DGU(i).A + DGU(i).B*DGU(i).K - DGU(i).Li*DGU(i).C;
    DGU(i).ALi = DGU(i).A-DGU(i).Li*DGU(i).C;
end

L = blkdiag(DGU.Li);
AL = blkdiag(DGU.ALi);


%% NEW UIO FORMULATION MATRICES

% Abar11 = [0                         1/DGU(1).Ct                       0                    ;
%         (DGU(1).K(1)-1)/DGU(1).Lt (DGU(1).K(2)-DGU(1).Rt)/DGU(1).Lt DGU(1).K(3)/DGU(1).Lt;
%         -1                        0                                 0                    ];
% 
% Abar_xavg_OL = [0                 1/DGU(1).Ct               0                    ;
%                 -1/DGU(1).Lt      -DGU(1).Rt/DGU(1).Lt      0;
%                 -1                0                         0                    ];
% 
% Abar12 = DGU(1).B*DGU(1).K;
% 
% Cbar = [eye(3) zeros(3,3); eye(3) eye(3)];
% 
% S11 = eye(3);
% S21 = eye(3);
% 
% S = [S11 zeros(3,3);
%      S21 zeros(3,3)];
%  
% H21 = -S21-eye(3);
% H11 = eye(3)-S11;
% 
% Hi = [H11 zeros(3,3);
%      H21 eye(3)];
% 
% SA = [S11*Abar11 S11*Abar12;
%       S21*Abar11 S21*Abar12];
%   
% Ktildei = place(SA',Cbar',[-1 -1.5 -2 -2.5 -3 -3.5])';
% Fi = SA - Ktildei*Cbar;
% Kbari = Fi*Hi;
% Khati = Ktildei + Kbari;

A11 = Adisc(1:3,1:3);
A11pr = DGU(1).Ak;

S = [zeros(6,1) [1;1;0;0;0;0] zeros(6,1) [0;0;1;1;0;0] zeros(6,1) [0;0;0;0;1;1]];
% S = rand(6,6);
% S(:,[1 3 5]) = 0;

Cbar = [eye(3) zeros(3,3); eye(3) eye(3)];

Hi = (eye(6)-S)*inv(Cbar);

SA = S*[A11pr zeros(3,3); -DGU(1).B*DGU(1).K A11];

Ktildei = place(SA',Cbar',[-1 -1.5 -2 -2.5 -3 -3.5])';
Fi = SA - Ktildei*Cbar;
Kbari = Fi*Hi;
Khati = Ktildei + Kbari;

% Noise bounds
nMax = .05;
nMaxMeas = .01;

rhoMax = nMaxMeas*repmat([ones(n-1,1);0],N,1);
wMax = nMax*repmat([ones(n-1,1);0],N,1)+nMaxMeas*repmat([zeros(n-1,1);1],N,1);

rhoNoise = [2*nMaxMeas*rand(n*N,tTot/tSamp/1+1)-nMaxMeas];
rhoNoise(3:n:end,:) = 0;
rhoNoise_ts = timeseries(rhoNoise,0:tSamp*1:tTot);

%% UIO Matrices for all DGUs 
for j = 1:N
    DGU(j).Ebar = [1/DGU(j).Ct 0; 0 0; 0 1];
    DGU(j).Hj = [1 0.1 0; 0 0.9 0; 0 0.1 1];
    DGU(j).Tj = eye(n) - DGU(j).Hj*DGU(j).C;
%     DGU(j).Ktilda = lqr((DGU(j).Tj*DGU(j).Ak)',DGU(j).C',eye(3),eye(3))';
    DGU(j).Ktilda = place((DGU(j).Tj*DGU(j).A)',DGU(j).C',[-1 -1.5 -2])';
    DGU(j).Fj = DGU(j).Tj*DGU(j).A- DGU(j).Ktilda*DGU(j).C;
    DGU(j).Kbar = DGU(j).Fj*DGU(j).Hj;
    DGU(j).Khat = DGU(j).Ktilda + DGU(j).Kbar;
    
    Ajj = DGU(j).Aii;
    BjKj = DGU(j).B*DGU(j).K;
    DGU(j).bA = [   Ajj+BjKj   ,     BjKj   ;
                     -BjKj     ,   Ajj-BjKj ];
    DGU(j).bEbar = [ 1 , 0 , 0 ;
                     0 , 0 , 0 ;
                     0 , 1 , 0 ;
                     0 , 0 , 0 ;
                     0 , 0 , 1 ;
                     0 , 0 , 0 ];
    DGU(j).bW = [ zeros(3,3) ,  BjKj ;
                    eye(3)   , -BjKj ];
    DGU(j).bC = [   eye(3)  ,  zeros(3,3) ;
                    eye(3)  ,    eye(3)   ];
    DGU(j).bRho = [ zeros(3,3) ;
                     eye(3)  ];
    DGU(j).bT = zeros(6,6);
    DGU(j).bT(1:2,2) = 1;
    DGU(j).bT(3:4,4) = 1;
    DGU(j).bT(5:6,6) = 1;
    
    DGU(j).bwMax = abs(DGU(j).bW) * [wMax((j-1)*n+1:j*n);rhoMax((j-1)*n+1:j*n)];
    DGU(j).brhoMax = abs(DGU(j).bRho) * rhoMax((j-1)*n+1:j*n);
    
    DGU(j).bH = (eye(6)-DGU(j).bT)*inv(DGU(j).bC);
    
    DGU(j).bKtilde = place((DGU(j).bT*DGU(j).bA)',DGU(j).bC',[-1 -1.5 -2 -2.5 -3 -3.5])';
    DGU(j).bF = DGU(j).bT*DGU(j).bA - DGU(j).bKtilde*DGU(j).bC;
    
    DGU(j).bKbar = DGU(j).bF*DGU(j).bH;
    DGU(j).bKhat = DGU(j).bKtilde + DGU(j).bKbar;
    DGU(j).bZ = eye(6)-DGU(j).bC*DGU(j).bH;
    
%     Ajjsm = DGU(j).Aii(1:2,1:2);
%     BjKjsm = DGU(j).B(1:2)*DGU(j).K(1:2);
%     DGU(j).bAsm = [   Ajjsm+BjKjsm   ,     BjKjsm   ;
%                      -BjKjsm         ,   Ajjsm-BjKjsm ];
%     DGU(j).bEbarsm = [ 1 , 0 , 0 ;
%                        0 , 0 , 0 ;
%                        0 , 0 , 0 ;
%                        0 , 0 , 1 ];
%     DGU(j).bWsm = [ zeros(2,2) ,  BjKjsm ;
%                       eye(2)   , -BjKjsm ];
%     DGU(j).bCsm = [   eye(2)  ,  zeros(2,2) ;
%                       eye(2)  ,    eye(2)   ];
%     DGU(j).bRhosm = [ zeros(2,2) ;
%                         eye(2)   ];
%     DGU(j).bTsm = zeros(4,4);
%     DGU(j).bTsm(1:2,2) = 1;
%     DGU(j).bTsm(2:4,4) = 1;
%     
%     DGU(j).bHsm = (eye(4)-DGU(j).bTsm)*inv(DGU(j).bCsm);
%     
%     DGU(j).bKtildesm = place((DGU(j).bTsm*DGU(j).bAsm)',DGU(j).bCsm',[-1 -1.5 -2 -2.5])';
%     DGU(j).bFsm = DGU(j).bTsm*DGU(j).bAsm - DGU(j).bKtildesm*DGU(j).bCsm;
%     
%     DGU(j).bKbarsm = DGU(j).bFsm*DGU(j).bHsm;
%     DGU(j).bKhatsm = DGU(j).bKtildesm + DGU(j).bKbarsm;
%     DGU(j).bZsm = eye(4)-DGU(j).bCsm*DGU(j).bHsm;
    
    DGU(j).bM = [DGU(j).M; zeros(3,2)];
    DGU(j).bInet = [-1/DGU(j).Ct; zeros(5,1)];
    DGU(j).bB = [zeros(3,1); DGU(j).B];
end

H = blkdiag(DGU.Hj);
T = blkdiag(DGU.Tj);
F = blkdiag(DGU.Fj);
Ktilda = blkdiag(DGU.Ktilda);
Kbar = blkdiag(DGU.Kbar);
Khat = blkdiag(DGU.Khat);
% bW = blkdiag(DGU.bW);
% bRho = blkdiag(DGU.bRho);



bH = blkdiag(DGU.bH);
bT = blkdiag(DGU.bT);
bF = blkdiag(DGU.bF);
bKtilda = blkdiag(DGU.bKtilde);
bKbar = blkdiag(DGU.bKbar);
bKhat = blkdiag(DGU.bKhat);
bC = blkdiag(DGU.bC);

bigCyComm = [zeros(3,3)  eye(3) zeros(3,30);
             zeros(3,9)  eye(3) zeros(3,24);
             zeros(3,15) eye(3) zeros(3,18);
             zeros(3,21) eye(3) zeros(3,12);
             zeros(3,27) eye(3) zeros(3,6) ;
             zeros(3,33) eye(3)            ];



         
atk24 = [zeros(3,1); 0.1*rand(3,1)-0.05];
atk34 = [zeros(3,1); 0.1*rand(3,1)-0.05];

% atk24 = 0.04*rand(6,1)-0.02;
% atk34 = 0.04*rand(6,1)-0.02;

atk34(4) = -atk24(4)*DGU(4).Aij(1,1)/DGU(4).Aij(1,4);
         

% wNoise = [2*nMax*rand(n*N,tTot*10/tSamp+1)-nMax];
% wNoise(3:n:end,:) = 2*nMaxMeas*rand(N,tTot*10/tSamp+1)-nMaxMeas;
% wNoise_ts = timeseries(wNoise,0:tSamp/10:tTot);


% %% Generate Resiual Thresholds
% 
% % Initial local estimation error 
% epsilon0bar = repmat(rhoMax,1);%[50;5;15;50;2;15;50;4;15;50;3;15;50;10;15;50;3;15];
% 
% rLoc_bar = zeros(n*N,nTot);
% r_bar = zeros(n*N,nTot);
% 
% for t = 1:nTot    
% 
%     if t >= tc
%         
%         gamma = (@(Tau) (abs(expm(AL*((t-1)*tSamp-Tau)))));
%         integrall = integral(gamma,Tc,(t-1)*tSamp,'ArrayValued',true);
%         rLoc_bar(:,t) = abs(expm(AL*((t-1)*tSamp-Tc)))*epsilon0bar + ...
%             integrall*(abs(Aij)*rhoMax+wMax+abs(L)*rhoMax) + rhoMax;
%     
%         Gamma = (@(Tau) (abs(expm(F*((t-1)*tSamp-Tau)))));
%         Integrall = integral(Gamma,Tc,(t-1)*tSamp,'ArrayValued',true);
%         r_bar(:,t) = abs(expm(F*((t-1)*tSamp-Tc)))*(epsilon0bar + abs(H)*rhoMax) ...
%                    + (abs(T))*rhoMax ...
%                    + Integrall*((abs(T)*wMax) + abs(Khat)*rhoMax);
%     end
%     t
% end
% 
% rLoc_bar_ts = timeseries(rLoc_bar,tSim);
% r_bar_ts = timeseries(r_bar,tSim);
% 
% save('rbar_slowLuen_w0p05','rLoc_bar','rLoc_bar_ts','r_bar','r_bar_ts')

%%

%% Generate Resiual Thresholds for 6x6 UIO
% 
% % brhoMax = [zeros(n*N);
% %             eye(n*N) ] * rhoMax;
% %         
% % bwMax = abs([ zeros(n*N) ,  B*K ;
% %                eye(n*N)  , -B*K ]) * [  wMax  ;
% %                                        rhoMax ];
% bwMax = reshape([DGU.bwMax],2*n*N,1);
% brhoMax = reshape([DGU.brhoMax],2*n*N,1);
% 
% % Initial local estimation error 
% bepsilon0bar = brhoMax;%[50;5;15;50;2;15;50;4;15;50;3;15;50;10;15;50;3;15];
% epsilon0bar = rhoMax;
% 
% rLoc_bar = zeros(n*N,nTot);
% r_bar = zeros(2*n*N,nTot);
% 
% for t = 1:nTot    
% 
%     if t >= tc
%         
%         gamma = (@(Tau) (abs(expm(AL*((t-1)*tSamp-Tau)))));
%         integrall = integral(gamma,Tc,(t-1)*tSamp,'ArrayValued',true);
%         rLoc_bar(:,t) = abs(expm(AL*((t-1)*tSamp-Tc)))*epsilon0bar + ...
%             integrall*(abs(Aij)*rhoMax+wMax+abs(L)*rhoMax) + rhoMax;
%     
%         bGamma = (@(Tau) (abs(expm(bF*((t-1)*tSamp-Tau)))));
%         bIntegrall = integral(bGamma,Tc,(t-1)*tSamp,'ArrayValued',true);
%         r_bar(:,t) = abs(bC*expm(bF*((t-1)*tSamp-Tc)))*(bepsilon0bar + abs(bH)*brhoMax) ...
%                    + (abs(bC*(eye(2*n*N)-bC*bH)))*brhoMax ...
%                    + abs(bC)*bIntegrall*((abs(bT)*bwMax) + abs(bKhat)*brhoMax);
%     end
%     t
% end
% 
% % rLoc_bar_ts = timeseries(rLoc_bar,tSim);
% r_bar_ts = timeseries(r_bar,tSim);
% 
% % save('rbar_slowLuen_w0p05_6by6UIO','r_bar','r_bar_ts')
% save('rbar_slowLuen_w0p05_6by6UIO','r_bar','r_bar_ts','rLoc_bar','rLoc_bar_ts')

%%

% %% Stealthy attack simulation test
% 
% DGUj = DGU(1);
% 
% Hj = DGUj.Hj
% Sj = eye(3)-Hj
% Akj = DGUj.Ak
% Ktildej = DGUj.Ktilda
% Fj = DGUj.Fj
% Khatj = DGUj.Khat
% 
% H1 = Hj(1,2);
% H2 = Hj(2,2);
% H3 = Hj(3,2);
% 
% % Elements for reducing constraint A to constraint A'
% a = Khatj(1,1)/H1+Khatj(2,1)/(1-H2)
% b = Khatj(1,3)/H1+Khatj(2,3)/(1-H2)
% d = Khatj(3,3)/H3+Khatj(2,3)/(1-H2)
% c = Khatj(3,1)/H3+Khatj(2,1)/(1-H2)
% 
% % Matrices of the dynamics \tilde{\Sigma}
% Atilde = [Fj -Khatj(:,2); Fj(1,:)/H1 -Khatj(1,2)/H1]
% Btilde = [-Khatj(:,1) -Khatj(:,3); -Khatj(1,1)/H1 -Khatj(1,3)/H1]
% 
% % Btilde is rank deficient
% rank(Btilde)
% 
% % Dynamics \tilde{\Sigma} is not controllable
% rank([Btilde Atilde*Btilde Atilde*Atilde*Btilde Atilde*Atilde*Atilde*Btilde])
% 
% % The controllable subspace of \tilde{\Sigma}
% Mc = [Btilde Atilde*Btilde Atilde*Atilde*Btilde Atilde*Atilde*Atilde*Btilde];
% mc = orth(Mc)
% 
% % Matrix for constraint A'
% Aprime = [Fj(2,:)/(1-H2)+Fj(1,:)/H1 -Khatj(2,2)/(1-H2)-Khatj(1,2)/H1; 
%                    Fj(2,:)/(1-H2)+Fj(3,:)/H3 -Khatj(2,2)/(1-H2)-Khatj(3,2)/H3]
%                
% % Constraint A' is full row rank
% rank(Aprime)
% 
% % Controllable subspace of the system satisfies the constraint 
% Aprime*mc
% 
% 
% % Gamma matrix 
% Gamma = Aprime*Btilde

 