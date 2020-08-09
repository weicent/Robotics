function [xEKF, pEKF]=EKF_Algorithm(x, u, p, z, H, R, Q, JF, JH, dt)
%预测
xPred=PredictNewStatus(x,u, dt);
pPred=JF*p*JF.'+Q;%JF.'等效于transpose(JF)
%更新
zPred=H*xPred;
y=z-zPred;
S=JH*pPred*JH.'+R;
K=pPred*JH.'*S^(-1);%S^(-1)等效于inv(S)
xEKF=xPred+K*y;
pEKF=(eye(4)-K*JH)*pPred;
end