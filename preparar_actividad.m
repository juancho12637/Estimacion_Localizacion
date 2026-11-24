function carpeta = preparar_actividad()
% Prepara una ejecución individual del Live Script sin alterar el código fuente.
raiz = fileparts(mfilename('fullpath'));
contenedor = fullfile(raiz,'resultados','actividad');
if ~isfolder(contenedor), mkdir(contenedor); end
carpeta = fullfile(contenedor,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
assert(~isfolder(carpeta),'El directorio de ejecución ya existe.');
mkdir(carpeta);
for f = {'actividad.mlx','actividad.m','exampleMaps.mat','auxiliar','guardar_historial.m'}
    copyfile(fullfile(raiz,f{1}),fullfile(carpeta,f{1}));
end
cd(carpeta);
open('actividad.mlx');
fprintf('Directorio de ejecución: %s\n',carpeta);
fprintf('Ejecute Run All, guarde el Live Script y ejecute guardar_historial.\n');
end
