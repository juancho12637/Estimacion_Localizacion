function graficar_evolucion_estados(xTrueHist, xHistEKF, xHistUKF, xHistMCL, k, nombreArchivo, dt)
    if nargin < 7, dt = 0.1; end
    t = (1:k)*dt;
    fig = figure;

    % ----- Subplot X -----
    subplot(3,1,1);
    hold on;
    plot(t, xTrueHist(1,1:k), 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.5);
    plot(t, xHistEKF(1,1:k),  'r-', 'LineWidth', 5);
    plot(t, xHistUKF(1,1:k),  'g-', 'LineWidth', 3.5);
    plot(t, xHistMCL(1,1:k),  'b-', 'LineWidth', 2);
    title('x'); ylabel('x (m)'); xlabel('Tiempo (s)'); legend('Real','EKF','UKF','MCL');
    grid on;

    % ----- Subplot Y -----
    subplot(3,1,2);
    hold on;
    plot(t, xTrueHist(2,1:k), 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.5);
    plot(t, xHistEKF(2,1:k),  'r-', 'LineWidth', 5);
    plot(t, xHistUKF(2,1:k),  'g-', 'LineWidth', 3.5);
    plot(t, xHistMCL(2,1:k),  'b-', 'LineWidth', 2);
    title('y'); ylabel('y (m)'); xlabel('Tiempo (s)'); legend('Real','EKF','UKF','MCL');
    grid on;

    % ----- Subplot Theta -----
    subplot(3,1,3);
    hold on;
    plot(t, xTrueHist(3,1:k), 'Color', [0.5 0.5 0.5], 'LineStyle', ':', 'LineWidth', 1.5);
    plot(t, xHistEKF(3,1:k),  'r-', 'LineWidth', 5);
    plot(t, xHistUKF(3,1:k),  'g-', 'LineWidth', 3.5);
    plot(t, xHistMCL(3,1:k),  'b-', 'LineWidth', 2);
    title('\theta'); ylabel('Orientacion (rad)'); xlabel('Tiempo (s)'); legend('Real','EKF','UKF','MCL');
    grid on;
    ylim([-pi, pi]);
    yticks(-pi : pi/2 : pi);
    yticklabels({'-\pi','-\pi/2','0','\pi/2','\pi'});

    % Exportar imagen
    if nargin < 6
        nombreArchivo = 'comparativa_x_y_theta.png';
    end
    exportgraphics(fig, nombreArchivo, 'Resolution', 800);
end