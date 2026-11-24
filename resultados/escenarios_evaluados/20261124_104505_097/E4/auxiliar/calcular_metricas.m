function results = calcular_metricas(xTrue, estStruct, covStruct, Q, latencyStruct)
% Firma original. NEES/NIS temporales no disponibles con la plantilla protegida.
filters = fieldnames(estStruct); A = nan(numel(filters),12);
for i=1:numel(filters)
    n=filters{i}; X=estStruct.(n); e=X-xTrue; e(3,:)=wrapToPi(e(3,:));
    d=vecnorm(e(1:2,:),2,1); inc=diff(e(1:2,:),1,2);
    lat=latencyStruct.(n); valid=lat>0 & isfinite(lat);
    A(i,:)=[mean(d),sqrt(mean(d.^2)),sqrt(mean(e(3,:).^2))*180/pi, ...
        sqrt(mean(sum(inc.^2,1))),norm(e(1:2,end)-e(1:2,1)),d(end),std(d), ...
        mean(lat(valid))*1000,nnz(valid)/(size(X,2)*0.1),NaN,NaN,NaN];
end
results=array2table(A,'VariableNames',{'ATE_media_m','RMSE_pos_m','RMSE_theta_deg','RPE_incremento_m','Drift_neto_m','Error_final_m','STD_m','Latencia_ms_llamadas','Frecuencia_llamadas_Hz','NEES','NIS_1D','NIS_3D'});
results=addvars(results,filters,'Before',1,'NewVariableNames','Filtro');
disp(results); writetable(results,'metricas.csv');
end
