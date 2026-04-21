clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Duración total para dar una vuelta completa
tf = 20;             
ts = 0.005;            
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 

% IMPORTANTE: Iniciamos el robot sobre la trayectoria para evitar saltos.
% El punto de inicio en t=0 será x=4, y=0.
x1(1) = 4;              
y1(1) = 0;   
% Lo orientamos a 90 grados (pi/2) para que arranque tangencial a la curva
phi(1) = pi/2;             

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);  
hy = zeros(1,N+1);  
hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); 
w = zeros(1,N); 

% Parámetros de la circunferencia
R = 4;                     % Radio (sacado de x^2 + y^2 = 16)
W_ang = (2*pi) / tf;       % Velocidad angular de la trayectoria para cerrar en tf

for k = 1:N
    tk = t(k);
    
    % -------- TRAYECTORIA PARAMÉTRICA (CÍRCULO) --------
    
    % Primera derivada (Velocidades en ejes X e Y)
    dx = -R * W_ang * sin(W_ang * tk);
    dy = R * W_ang * cos(W_ang * tk);
    
    % Segunda derivada (Aceleraciones en ejes X e Y)
    ddx = -R * W_ang^2 * cos(W_ang * tk);
    ddy = -R * W_ang^2 * sin(W_ang * tk);
    
    % Cálculo de velocidad lineal (u) y angular (w) usando la fórmula general
    u(k) = sqrt(dx^2 + dy^2);
    w(k) = (dx*ddy - dy*ddx) / (dx^2 + dy^2); 
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % Cinemática del robot diferencial
    phi(k+1) = phi(k) + w(k)*ts;
    
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    
    x1(k+1) = x1(k) + xp1*ts; 
    y1(k+1) = y1(k) + yp1*ts; 
    
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure;
set(scene,'Color','white');
set(gca,'FontWeight','bold');
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
camlight('headlight');
axis equal;
grid on;
box on;
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
view([25 25]);

% Ejes ajustados para abarcar el radio de 4 desde el centro (0,0)
axis([-5 5 -5 5 0 2]); 
scale = 0.5; 

MobileRobot_5;
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2 = plot3(hx(1),hy(1),0,'r','lineWidth',2);
step = 1;

for k=1:step:N
    delete(H1);    
    delete(H2);
    
    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2 = plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
graph = figure;
set(graph,'position',sizeScreen);
subplot(211)
plot(t,u,'b','LineWidth',2), grid on
xlabel('Tiempo [s]'), ylabel('m/s'), legend('u (Velocidad Lineal)')
title('Perfil de Velocidad Lineal')
axis([0 tf 0 2]) % Fijamos el eje Y para ver que la velocidad es constante

subplot(212)
plot(t,w,'r','LineWidth',2), grid on
xlabel('Tiempo [s]'), ylabel('rad/s'), legend('w (Velocidad Angular)')
title('Perfil de Velocidad Angular')
axis([0 tf 0 1]) % Fijamos el eje Y para ver que la velocidad es constante