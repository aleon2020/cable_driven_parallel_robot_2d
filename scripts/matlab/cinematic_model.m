% PARAMETERS
largo_plano = 100;
alto_plano = 100;
largo_efector = 10;
alto_efector = 10;
radio_rueda = 5;

% REQUEST FOR THE COORDINATES OF THE END-EFFECTOR
fprintf('\nCOORDENADAS DEL EFECTOR FINAL\n');
x_efector = input('Coordenada x del efector final: ');
y_efector = input('Coordenada y del efector final: ');
fprintf('\n');

% LIMIT VERIFICATION
error = false;
if x_efector >= (largo_plano - (largo_efector / 2))
    fprintf("FUERA DEL LÍMITE en x = %.2f\n", x_efector);
    error = true;
elseif x_efector <= (largo_efector / 2)
    fprintf("FUERA DEL LÍMITE en x = %.2f\n", x_efector);
    error = true;
end
if y_efector >= (alto_plano - (alto_efector / 2))
    fprintf("FUERA DEL LÍMITE en y = %.2f\n", y_efector);
    error = true;
elseif y_efector <= (alto_efector / 2)
    fprintf("FUERA DEL LÍMITE en y = %.2f\n", y_efector);
    error = true;
end
if error
    return;
end

% FIGURE
figure;
hold on;
axis equal;
xlim([-largo_plano * 0.25, largo_plano * 1.25]);
ylim([-alto_plano * 0.5, alto_plano * 1.25]);
xlabel('Eje X (centímetros)');
ylabel('Eje Y (centímetros)');
title('Robot por cables para el control de un efector final');

% STRUCTURE
plot([0, 0], [alto_plano, -(alto_plano / 2)], 'k', 'LineWidth', 5);
plot([largo_plano, largo_plano], [alto_plano, -(alto_plano / 2)], 'k', 'LineWidth', 5);
plot([0, largo_plano], [alto_plano, alto_plano], 'k', 'LineWidth', 5);
plot([0, largo_plano], [0, 0], 'k', 'LineWidth', 5);
plot([-5, 5], [-(alto_plano / 2), -(alto_plano / 2)], 'k', 'LineWidth', 5);
plot([largo_plano - 5, largo_plano + 5], [-(alto_plano / 2), -(alto_plano / 2)], 'k', 'LineWidth', 5);

% END-EFFECTOR
% Top left corner (x1, y1)
x1 = x_efector - (largo_efector / 2);
y1 = y_efector + (alto_efector / 2);
% Top right corner (x2, y2)
x2 = x_efector + (largo_efector / 2);
y2 = y_efector + (alto_efector / 2);
% Bottom left corner (x3, y3)
x3 = x_efector - (largo_efector / 2);
y3 = y_efector - (alto_efector / 2);
% Bottom right corner (x4, y4)
x4 = x_efector + (largo_efector / 2);
y4 = y_efector - (alto_efector / 2);

% CABLES
% Cable top left corner M1 = (M1x, M1y)
M1x = 0;
M1y = alto_plano;
plot([M1x, x1], [M1y, y1], 'r', 'LineWidth', 2);
% Cable top right corner M2 = (M2x, M2y)
M2x = largo_plano;
M2y = alto_plano;
plot([M2x, x2], [M2y, y2], 'r', 'LineWidth', 2);

% END-EFFECTOR REPRESENTATION
xe = [x3, x4, x2, x1, x3];
ye = [y3, y4, y2, y1, y3];
plot(xe, ye, 'black', 'LineWidth', 2);
plot(x_efector, y_efector, 'ko', 'MarkerSize', 2, 'MarkerFaceColor', 'black');

% PULLEYS REPRESENTATION
plot(0, alto_plano, 'ko', 'MarkerSize', radio_rueda, 'MarkerFaceColor', 'black');
plot(0, alto_plano, 'wo', 'MarkerSize', radio_rueda / 2, 'MarkerFaceColor', 'white');
plot(largo_plano, alto_plano, 'ko', 'MarkerSize', radio_rueda, 'MarkerFaceColor', 'black');
plot(largo_plano, alto_plano, 'wo', 'MarkerSize', radio_rueda / 2, 'MarkerFaceColor', 'white');

% CABLE LENGTHS
L1 = sqrt((x1 - M1x)^2 + (y1 - M1y)^2);
L2 = sqrt((x2 - M2x)^2 + (y2 - M2y)^2);
fprintf("Longitud del cable L1 = %.2f cm\n", L1);
fprintf("Longitud del cable L2 = %.2f cm\n", L2);

% ANGLES
q1 = -rad2deg(atan((x1 - M1x) / (y1 - M1y)));
q2 = rad2deg(atan((x2 - M2x) / (y2 - M2y)));
fprintf("Ángulo del cable L1 (q1) = %.2f °\n", q1);
fprintf("Ángulo del cable L2 (q2) = %.2f °\n", q2);

hold off;