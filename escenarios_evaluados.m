function salida = escenarios_evaluados()
% Ejecutar E0-E4 y guardar los resultados en directorios independientes.
% MATLAB R2021a+. Ejecutar desde Command Window, no desde actividad.mlx.
raiz = fileparts(mfilename('fullpath'));
ref = raiz;
original = fileread(fullfile(ref,'actividad.m'));
gps = 'Q_gps = diag([0.9, 0.9]).^2;';
imu = 'Q_imu = deg2rad(20)^2;';
assert(numel(strfind(original,gps))==1 && numel(strfind(original,imu))==1,'Definiciones de ruido inesperadas.');
requeridos = {'metricas.csv','historial_localizacion.mat','map_trajectory.png', ...
    'comparativa_x_y_theta.png','truth.txt','ekf_est.txt','ukf_est.txt','mcl_est.txt'};
for f={'actividad.mlx','exampleMaps.mat','guardar_historial.m','evaluar_trayectorias.m'}
    assert(isfile(fullfile(ref,f{1})),['Falta dependencia: ' f{1}]);
end
assert(isfolder(fullfile(ref,'auxiliar')),'Faltan funciones auxiliares.');
contenedor=fullfile(raiz,'resultados','escenarios_evaluados');
if ~isfolder(contenedor), mkdir(contenedor); end
marca=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
salida=fullfile(contenedor,marca);
assert(~isfolder(salida),'Ya existe el directorio de ejecución.');
mkdir(salida);
log=fullfile(salida,'verificacion_bloques.txt');
ids={'E0','E1','E2','E3','E4'};
viejos={gps,gps,gps,imu,imu};
nuevos={gps,'Q_gps = diag([0.45, 0.45]).^2;', ...
        'Q_gps = diag([1.8, 1.8]).^2;', ...
        'Q_imu = deg2rad(10)^2;', 'Q_imu = deg2rad(40)^2;'};
sigGPS=[.9 .45 1.8 .9 .9]; sigIMU=[20 20 20 10 40];
previo=pwd; rutaPrevia=path;
limpieza=onCleanup(@() restaurar(previo,rutaPrevia)); %#ok<NASGU>
% Preparar y comprobar TODAS las copias antes de simular.
for i=1:5
    destino=fullfile(salida,ids{i}); mkdir(destino);
    for f={'exampleMaps.mat','guardar_historial.m','auxiliar'}
        copyfile(fullfile(ref,f{1}),fullfile(destino,f{1}));
    end
        texto=strrep(original,viejos{i},nuevos{i});
    assert(numel(strfind(texto,nuevos{i}))==1 && ...
        strcmp(strrep(texto,nuevos{i},viejos{i}),original), ...
        'Cambio inesperado en el codigo.');
    archivo=fullfile(destino,'actividad.m');
    fid=fopen(archivo,'w','n','UTF-8'); assert(fid>=0,'No se pudo crear actividad.m');
    fwrite(fid,unicode2native(texto,'UTF-8'),'uint8'); fclose(fid);
    assert(strcmp(fileread(archivo),texto),'La copia escrita no coincide.');
    verificar_aux(ref,destino);
    if i==1
        escribir(log,'E0: código de referencia sin cambios.');
    else
        escribir(log,sprintf('%s: unica sustitucion verificada: %s -> %s',ids{i},viejos{i},nuevos{i}));
    end
end
% actividad.m contiene clear all: se ejecuta en el workspace base, no en el
% workspace de esta funcion. No se cambia el codigo de inicializacion.
for i=1:5
    destino=fullfile(salida,ids{i});
    cd(destino); path(rutaPrevia); addpath(fullfile(destino,'auxiliar'));
    fprintf('\nEjecutando %s en %s\n',ids{i},destino);
    try
        evalin('base','run(''actividad.m'');');
        evalin('base','run(''guardar_historial.m'');');
        for j=1:numel(requeridos)
            assert(isfile(fullfile(destino,requeridos{j})),['Falta salida: ' requeridos{j}]);
        end
        assert(strcmp(fileread(fullfile(destino,'actividad.m')),strrep(original,viejos{i},nuevos{i})), ...
            'El codigo cambio durante la ejecucion.');
        verificar_aux(ref,destino);
        escribir(log,[ids{i} ': ejecucion finalizada; codigo y auxiliares verificados.']);
    catch ME
        escribir(log,[ids{i} ': ERROR: ' ME.message]);
        fprintf(2,'Resultados parciales conservados en %s\n',salida);
        rethrow(ME);
    end
end
cd(raiz); path(rutaPrevia);
assert(strcmp(fileread(fullfile(ref,'actividad.m')),original),'E0 fuente cambiado.');
% Comparativa de las cinco ejecuciones existentes.
T=table(); nav=table();
for i=1:5
    id=sprintf('E%d',i-1); carpeta=fullfile(salida,id);
    M=readtable(fullfile(carpeta,'metricas.csv'),'TextType','string');
    M=addvars(M,repmat(string(id),height(M),1),repmat(sigGPS(i),height(M),1), ...
        repmat(sigIMU(i),height(M),1),'Before',1,'NewVariableNames',{'Escenario','Sigma_GPS_m','Sigma_IMU_deg'});
    T=[T;M]; %#ok<AGROW>
    D=load(fullfile(carpeta,'historial_localizacion.mat'));
    n=D.k; meta=D.goal(1:2)';
    assert(norm(D.Q_gps-eye(2)*sigGPS(i)^2,'fro')<1e-12 && abs(D.Q_imu-deg2rad(sigIMU(i))^2)<1e-12,'Covarianzas inesperadas.');
    d=[norm(D.xTrueHist(1:2,n)-meta),norm(D.xHistEKF(1:2,n)-meta), ...
       norm(D.xHistUKF(1:2,n)-meta),norm(D.xHistMCL(1:2,n)-meta)];
    N=table(string(id),n,n*D.dt,d(1),d(2),d(3),d(4),all(d(2:4)<D.tol), ...
        'VariableNames',{'Escenario','Muestras','Tiempo_s','Real_meta_m','EKF_meta_m','UKF_meta_m','MCL_meta_m','Criterio_original_cumplido'});
    nav=[nav;N]; %#ok<AGROW>
end
writetable(T,fullfile(salida,'comparativa_escenarios.csv'));
writetable(nav,fullfile(salida,'navegacion.csv'));
vars={'RMSE_pos_m','RMSE_theta_deg','Error_final_m','Latencia_ms_llamadas'};
labels={'RMSE posicion (m)','RMSE orientacion (grados)','Error final (m)','Latencia por llamada (ms)'};
for j=1:numel(vars)
    A=nan(5,3); filtros={'EKF','UKF','MCL'};
    for i=1:5
        for f=1:3
            q=T.Escenario==string(sprintf('E%d',i-1)) & T.Filtro==string(filtros{f});
            assert(nnz(q)==1,'Tabla de metricas inesperada.'); A(i,f)=T.(vars{j})(q);
        end
    end
    fig=figure('Visible','off','Color','w'); bar(A); xticklabels({'E0','E1','E2','E3','E4'});
    ylabel(labels{j}); legend(filtros,'Location','best'); grid on;
    exportgraphics(fig,fullfile(salida,[vars{j} '.png']),'Resolution',180); close(fig);
end
% Evaluacion matematica MATLAB independiente, no EVO Toolkit.
evaluar_trayectorias(salida);
fprintf('\nResultados: %s\n',salida);
end

function verificar_aux(ref,destino)
arch=dir(fullfile(ref,'auxiliar','**','*.m'));
for j=1:numel(arch)
    a=fullfile(arch(j).folder,arch(j).name);
    relativo=a(numel(ref)+2:end);
    assert(strcmp(fileread(a),fileread(fullfile(destino,relativo))),['Auxiliar distinto: ' relativo]);
end
for f={'guardar_historial.m','exampleMaps.mat'}
    assert(isequal(bytes(fullfile(ref,f{1})),bytes(fullfile(destino,f{1}))),['Archivo distinto: ' f{1}]);
end
end
function b=bytes(p)
f=fopen(p,'rb'); assert(f>=0,'No se pudo leer archivo'); b=fread(f,Inf,'*uint8'); fclose(f);
end
function escribir(p,s)
f=fopen(p,'a','n','UTF-8'); assert(f>=0,'No se pudo escribir registro'); fprintf(f,'%s\n',s); fclose(f);
end
function restaurar(p,r)
cd(p); path(r);
end
