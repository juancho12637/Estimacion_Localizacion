% Guarda las poses y parámetros del ensayo en el directorio de ejecución.
assert(exist('k','var')==1 && exist('xTrueHist','var')==1,'No hay una simulación ejecutada.');
save('historial_localizacion.mat','xTrueHist','xHistEKF','xHistUKF','xHistMCL','k','dt', ...
    'Q_gps','Q_imu','Q','R_odom','goal','tol','latEKF','latUKF','latMCL');
fprintf('Distancia real final a meta: %.4f m\n',norm(xTrueHist(1:2,k)-goal(1:2)'));
