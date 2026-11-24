function resumen = evaluar_trayectorias(raiz)
% Evaluacion independiente de poses SE(2), sin alineacion ni escala.
% MATLAB R2021a+. No modifica los archivos de simulacion.
if nargin == 0, raiz = pwd; end
raiz = char(raiz);
if ~isfolder(raiz), error('La carpeta indicada no existe.'); end
esc = {'E0','E1','E2','E3','E4'};
filtros = {'EKF','UKF','MCL'};
campos = {'xHistEKF','xHistUKF','xHistMCL'};
% Encontrar una unica carpeta de experimentos completa.
if all(cellfun(@(s)isfile(fullfile(raiz,s,'historial_localizacion.mat')),esc))
    arch = dir(fullfile(raiz,'E0','historial_localizacion.mat'));
else
    arch = dir(fullfile(raiz,'**','historial_localizacion.mat'));
end
candidatos = {};
for a = 1:numel(arch)
    [padre,nombre] = fileparts(arch(a).folder);
    if strcmp(nombre,'E0') && all(cellfun(@(s)isfile(fullfile(padre,s,'historial_localizacion.mat')),esc))
        candidatos{end+1} = padre; %#ok<AGROW>
    end
end
candidatos = unique(candidatos);
if numel(candidatos) ~= 1
    error('Se requiere una unica carpeta con los cinco escenarios E0-E4. Indique su ruta.');
end
base = candidatos{1};
salida = fullfile(base,'evaluacion');
if isfolder(salida), error('La carpeta evaluacion ya existe. Conserve o renombre sus resultados antes de repetir.'); end
mkdir(salida);
filas = cell(0,13); 
for e = 1:5
    D = load(fullfile(base,esc{e},'historial_localizacion.mat'));
    n = double(D.k); dt = double(D.dt);
    assert(isscalar(n)&&isfinite(n)&&n==floor(n)&&n>10&&isscalar(dt)&&isfinite(dt)&&dt>0,'Historial invalido');
    T = validar(D.xTrueHist,n); t = (1:n)'*dt;
    carpeta = fullfile(salida,esc{e}); mkdir(carpeta);
    fig = figure('Visible','off','Color','w','Position',[100 100 1100 850]);
    tl = tiledlayout(fig,3,2,'TileSpacing','compact');
    ax = gobjects(6,1);
    nombres = {'APE posicion (m)','APE orientacion (deg)','CDF APE posicion','RPE traslacion, 1 paso (m)','RPE traslacion, 10 pasos (m)','RPE rotacion, 10 pasos (deg)'};
    for j=1:6, ax(j)=nexttile(tl); hold(ax(j),'on'); grid(ax(j),'on'); title(ax(j),nombres{j}); xlabel(ax(j),'Tiempo (s)'); end
    xlabel(ax(3),'APE posicion (m)'); ylabel(ax(3),'Probabilidad acumulada');
    apebox = zeros(n,3);
    for f = 1:3
        X = validar(D.(campos{f}),n);
        ape = hypot(X(1,:)-T(1,:),X(2,:)-T(2,:))';
        ang = abs(atan2(sin(X(3,:)-T(3,:)),cos(X(3,:)-T(3,:))))'*180/pi;
        [r1,a1] = relativo(T,X,1); [r10,a10] = relativo(T,X,10);
        series = {ape,ang,r1,a1,r10,a10};
        etiquetas = {'APE_pos','APE_ang','RPE_pos_1','RPE_ang_1','RPE_pos_10','RPE_ang_10'};
        unidades = {'m','deg','m','deg','m','deg'}; pasos=[0 0 1 1 10 10];
        for j=1:6
            v=series{j}; q=sort(v); p95=interp1(linspace(0,1,numel(q)),q,0.95);
            filas(end+1,:)={esc{e},filtros{f},etiquetas{j},unidades{j},pasos(j),pasos(j)*dt,numel(v),sqrt(mean(v.^2)),mean(v),median(v),min(v),max(v),std(v,1)}; %#ok<AGROW>
            if j==1, p95pos=p95; end
        end
        writetable(table(t,ape,ang,'VariableNames',{'Tiempo_s','APE_pos_m','APE_ang_deg'}),fullfile(carpeta,[filtros{f} '_APE.csv']));
        writetable(table(t(2:end),r1,a1,'VariableNames',{'Tiempo_s','RPE_pos_m','RPE_ang_deg'}),fullfile(carpeta,[filtros{f} '_RPE_1.csv']));
        writetable(table(t(11:end),r10,a10,'VariableNames',{'Tiempo_s','RPE_pos_m','RPE_ang_deg'}),fullfile(carpeta,[filtros{f} '_RPE_10.csv']));
        writetable(table(p95pos,'VariableNames',{'APE_pos_P95_m'}),fullfile(carpeta,[filtros{f} '_P95.csv']));
        plot(ax(1),t,ape); plot(ax(2),t,ang); plot(ax(3),sort(ape),(1:n)'/n);
        plot(ax(4),t(2:end),r1); plot(ax(5),t(11:end),r10); plot(ax(6),t(11:end),a10); apebox(:,f)=ape;
    end
    for j=1:6, legend(ax(j),filtros,'Location','best'); end
    title(tl,['Evaluacion SE(2) - ' esc{e}]); exportgraphics(fig,fullfile(carpeta,'evaluacion.png'),'Resolution',180); close(fig);
    fig=figure('Visible','off','Color','w'); boxchart(reshape(repmat(1:3,n,1),[],1),apebox(:)); xticks(1:3); xticklabels(filtros); ylabel('APE posicion (m)'); grid on; title(esc{e}); exportgraphics(fig,fullfile(carpeta,'cajas_APE.png'),'Resolution',180); close(fig);
end
resumen=cell2table(filas,'VariableNames',{'Escenario','Filtro','Metrica','Unidad','Delta_pasos','Delta_s','N','RMSE','Media','Mediana','Min','Max','STD_poblacional'});
writetable(resumen,fullfile(salida,'resumen_SE2.csv'));
fprintf('Resultados guardados en: %s\n',salida);
end
function X=validar(X,n)
assert(size(X,1)==3 && size(X,2)>=n,'Pose debe ser 3 x N'); X=double(X(:,1:n)); assert(all(isfinite(X(:))),'Pose no finita');
end
function [r,a]=relativo(T,X,d)
% Traslaciones relativas expresadas en el marco local de la pose inicial.
% Norma de la traslacion de inv(DeltaT_real)*DeltaT_est: invariante a rotacion.
i=1:size(T,2)-d; j=i+d;
p=local(T,i,j); q=local(X,i,j); r=hypot(q(1,:)-p(1,:),q(2,:)-p(2,:))';
w=(X(3,j)-X(3,i))-(T(3,j)-T(3,i)); a=abs(atan2(sin(w),cos(w)))'*180/pi;
end
function p=local(X,i,j)
d=X(1:2,j)-X(1:2,i); c=cos(X(3,i)); s=sin(X(3,i)); p=[c.*d(1,:)+s.*d(2,:);-s.*d(1,:)+c.*d(2,:)];
end
