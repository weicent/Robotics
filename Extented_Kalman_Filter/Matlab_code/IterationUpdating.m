function [xActual, zObserve, uNoise, xPredict]=...
    IterationUpdating(GPS_Noise, Sensor_Noise, H, u_Exact, x_Exact, xPred, dt)
xActual=PredictNewStatus(x_Exact, u_Exact, dt);%x��u�Ǵ�����û������������
zObserve=H*x_Exact+GPS_Noise*randn(2,1);
uNoise=u_Exact+Sensor_Noise*randn(2,1);
xPredict=PredictNewStatus(xPred, uNoise, dt);
end