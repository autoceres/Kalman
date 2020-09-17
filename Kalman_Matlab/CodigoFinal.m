addpath('quaternion_library');      % include quaternion library
close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal
format long;
%Define os vetores para calculo da variancia a partir de uma medicao feita
%parada, gerando 16100 valores de:
%   Latitude
%   Longitude
%   Aceleracao em X
%   Aceleracao em Y
%   Gravidade em X
%   Gravidade em Y
define_vetores_variancia;

%Define os vetores de dados coletados de um experimento
define_vetores_teste_2;

AHRS = MadgwickAHRS('SamplePeriod', 1/50, 'Beta', 0.1);
dcm = DCM_IMU; 

%Gera fator de conversão linear entre graus e metros a partir
%da posicao atual 
[m2gX, m2gY] = geraFatoresConversaoMetros2Graus(lat(1),lon(1)); 

sizeVar = size(accXVariancia);
numAmostras = sizeVar(1);

quaternion = zeros(numAmostras, 4);

%Calcula as variancias
for i=1:numAmostras(1)
   
    angVar(i) = (-1)*(gyroZVariancia(i))*dt/2;

end
 
[varLat, varLon, varAccX, varAccY, varAng] = calculaVar(latVariancia, lonVariancia, accXVariancia, accYVariancia, gravXVariancia, gravYVariancia, angVar);


%Define os vetores de dados coletados de um experimento
define_vetores_teste_3;

amostras = size(accX);

numAmostras = amostras(1); %quantidade de iterações (definido pela frequencia de aquisicao do app)


t=0; %tempo (segundos)

cont = 1; %contador para chamdas do GPS

% Matriz de comportamento do sistema
A = [1,     0,      dt,         0,        (dt^2)/2,             0
     0,     1,      0,          dt,         0,               (dt^2)/2
     0,     0,      1,          0,          dt,                 0
     0,     0,      0,          1,          0,                  dt
     0,     0,      0,          0,          1,                  0
     0,     0,      0,          0,          0,                  1];
 
%Seta valor inicial para a matriz P 
P = eye(6);

%Matriz de covariancia do processo
Q = eye(6)*0.05;

%Matriz Identidade
I = eye(6);

%Vetor de estados inicial 
%Pega primeira leitura do GPS e da IMU e considera que o corpo estava em
%repouso
x_ant = [lon(1) 
         lat(1)
         0
         0
         0
         0]; 
     
     
     
%VETOR PARA ARMAZENAR OS ESTADOS     
estados = zeros(7,numAmostras);   
estimado = zeros(7,numAmostras);

posLatAnt = 0;
posLonAnt = 0;
accXFinal = 0;
accYFinal = 0;

j=0;    

gyro = [gyroX(1), gyroY(1), gyroZ(1)];
acc = [accX(1), accY(1), accZ(1)];
mag = [magX(1), magY(1), magZ(1)];
AHRS.Update(gyro, acc, mag);	% gyroscope units must be radians
quaternion(1, :) = AHRS.Quaternion;

euler = quatern2euler(quaternConj(quaternion));

roll = euler(1,1);
pitch = euler(1,2);
yaw = euler(1,3);


[matrizRotacao] = geraMatrizRotacao(accX(1),accY(1),accZ(1), magX(1), magY(1), magZ(1), gyroX(1), gyroY(1), gyroZ(1));

vecOrientacao = pegaOrientacao(matrizRotacao);
oriInicial = vecOrientacao(1);

roll_filtrado = 0;
pitch_filtrado = 0;
yaw_filtrado = 0;

gyro_filtrado = [0, 0, 0];
acc_filtrado = [0, 0, 0];
mag_filtrado = [0, 0, 0]; 

grav_filtrado = [0, 0, 0];

for i=1 : numAmostras
       
    
    %Ajusta os valores de acelerecao
    gyro = [gyroX(i), gyroY(i), gyroZ(i)];
    acc = [accX(i), accY(i), accZ(i)];
    mag = [magX(i), magY(i), magZ(i)];
    grav = [gravX(i), gravY(i), gravZ(i)];
    
    T = 0.2;
    %FILTRO-----------------------------------------------------------
    % T             -> constante a ajustar
    % dt            -> tempo de discretizacao
    % ac_fitlrado   -> sinal filtrado
    % wc            -> Valor a filtrar
    %acx_filtrado=(1-(dt/T))*acx_filtrado+wcx(k)*(dt/T);
    %acy_filtrado=(1-(dt/T))*acy_filtrado+wcy(k)*(dt/T);
    %acz_filtrado=(1-(dt/T))*acz_filtrado+wcz(k)*(dt/T);
    %-----------------------------------------------------------------
    gyro_filtrado(1) = (1-(dt/T))*gyro_filtrado(1)+gyro(1)*(dt/T);
    gyro_filtrado(2) = (1-(dt/T))*gyro_filtrado(2)+gyro(2)*(dt/T);
    gyro_filtrado(3) = (1-(dt/T))*gyro_filtrado(3)+gyro(3)*(dt/T);
    
    acc_filtrado(1) = (1-(dt/T))*acc_filtrado(1)+acc(1)*(dt/T);
    acc_filtrado(2) = (1-(dt/T))*acc_filtrado(2)+acc(2)*(dt/T);
    acc_filtrado(3) = (1-(dt/T))*acc_filtrado(3)+acc(3)*(dt/T);
    
    mag_filtrado(1) = (1-(dt/T))*mag_filtrado(1)+mag(1)*(dt/T);
    mag_filtrado(2) = (1-(dt/T))*mag_filtrado(2)+mag(2)*(dt/T);
    mag_filtrado(3) = (1-(dt/T))*mag_filtrado(3)+mag(3)*(dt/T);
    
    grav_filtrado(1) = (1-(dt/T))*grav_filtrado(1)+grav(1)*(dt/T);
    grav_filtrado(2) = (1-(dt/T))*grav_filtrado(2)+grav(2)*(dt/T);
    grav_filtrado(3) = (1-(dt/T))*grav_filtrado(3)+grav(3)*(dt/T);
    
    
    %--------------  Angulos de Euler  --------------------------------%
    %                  Metodo DCM
    UpdateIMU(dcm,gyro_filtrado,acc_filtrado,dt);
    
    roll_dcm=dcm.roll;
    
    pitch_dcm=dcm.pitch;
     
    yaw_dcm=-dcm.yaw;%+ oriInicial;
    
    quaternion_dcm = eul2quat([yaw_dcm, pitch_dcm, roll_dcm]);
    %------------------------------------------------------------------%
    
    %--------------- Angulos de Euler ---------------------------------%
    %                      AHRS
    AHRS.Update(gyro_filtrado, acc_filtrado, mag_filtrado);	% gyroscope units must be radians
    quaternion(i, :) = AHRS.Quaternion;

    euler = quatern2euler(quaternConj(quaternion));
    
    roll = euler(i,1);
    pitch = euler(i,2);
    yaw = -euler(i,3);%+ oriInicial;
    %------------------------------------------------------------------%
    
    
    %--------------- Angulos de Euler ---------------------------------%
    %                Metodos Android  
    [matrizRotacao] = geraMatrizRotacao(acc_filtrado(1),acc_filtrado(2),acc_filtrado(3), mag_filtrado(1), mag_filtrado(2), mag_filtrado(3), gyro_filtrado(1), gyro_filtrado(2), gyro_filtrado(3));
    vecOrientacao = pegaOrientacao(matrizRotacao);
    yaw_a = vecOrientacao(1);
    pitch_a = vecOrientacao(2);
    roll_a = vecOrientacao(3);
    quaternion_a = eul2quat([yaw_a, pitch_a, roll_a]);
    %------------------------------------------------------------------%
    
        
    roll_filtrado = (1-(dt/T))*roll_filtrado+roll*(dt/T);
    pitch_filtrado = (1-(dt/T))*pitch_filtrado+pitch*(dt/T);
    yaw_filtrado = (1-(dt/T))*yaw_filtrado+yaw*(dt/T);
    
    %----- Matriz de rotacao do sistema ----------------------------------
    matrizRotacaoQuat = quaternion2MatrizRotacao(quaternion(i,1),quaternion(i,2),quaternion(i,3),quaternion(i,4));
    
    matrizVelAng = geraMatrizVelocidadeAngular(gyro_filtrado(1), gyro_filtrado(2), gyro_filtrado(3));

    matrizRotacao = transpose(matrizRotacaoQuat) * matrizVelAng;
    %--------------------------------------------------------------------%
    
        
%     accXRot = matrizRotacao(1)*(accX(i)-gravX(i)) + matrizRotacao(2)*(accY(i)-gravY(i)) + matrizRotacao(3)*(accZ(i)-gravZ(i));
%     accYRot = matrizRotacao(4)*(accX(i)-gravX(i)) + matrizRotacao(5)*(accY(i)-gravY(i)) + matrizRotacao(6)*(accZ(i)-gravZ(i));   
%     accZRot = matrizRotacao(7)*(accX(i)-gravX(i)) + matrizRotacao(8)*(accY(i)-gravY(i)) + matrizRotacao(9)*(accZ(i)-gravZ(i));   
%     
    accXRot = matrizRotacao(1)*(acc_filtrado(1)-grav_filtrado(1)) + matrizRotacao(2)*(acc_filtrado(2)-grav_filtrado(2)) + matrizRotacao(3)*(acc_filtrado(3)-grav_filtrado(3));
    accYRot = matrizRotacao(4)*(acc_filtrado(1)-grav_filtrado(1)) + matrizRotacao(5)*(acc_filtrado(2)-grav_filtrado(2)) + matrizRotacao(6)*(acc_filtrado(3)-grav_filtrado(3));   
    accZRot = matrizRotacao(7)*(acc_filtrado(1)-grav_filtrado(1)) + matrizRotacao(8)*(acc_filtrado(2)-grav_filtrado(2)) + matrizRotacao(9)*(acc_filtrado(3)-grav_filtrado(3));   
    
    
    gyroXRot = matrizRotacao(1)*(gyroX(i)) + matrizRotacao(2)*(gyroY(i)) + matrizRotacao(3)*(gyroZ(i));
    gyroYRot = matrizRotacao(4)*(gyroX(i)) + matrizRotacao(5)*(gyroY(i)) + matrizRotacao(6)*(gyroZ(i));
    gyroZRot = matrizRotacao(7)*(gyroX(i)) + matrizRotacao(8)*(gyroY(i)) + matrizRotacao(9)*(gyroZ(i));
    
    magXRot = matrizRotacao(1)*(magX(i)) + matrizRotacao(2)*(magY(i)) + matrizRotacao(3)*(magZ(i));
    magYRot = matrizRotacao(4)*(magX(i)) + matrizRotacao(5)*(magY(i)) + matrizRotacao(6)*(magZ(i));
    magZRot = matrizRotacao(7)*(magX(i)) + matrizRotacao(8)*(magY(i)) + matrizRotacao(9)*(magZ(i));
    
    
    %angFinal = yaw_filtrado ;
    
    %accYFinal = accYRot*cos(angFinal) + accXRot*sin(angFinal);
    %accXFinal = -accYRot*sin(angFinal) + accXRot*cos(angFinal);
    
    accXFinal = accXRot;
    accYFinal = accYRot;
    
    accYFinal = accYFinal * m2gY;
    accXFinal = accXFinal * m2gX;
    

    % -------------------------------------------------------------------
    
    if (cont == 49)
        
        %Pega coordenadas UTM
        j = j + 1;

        %-----------------
        if ((lat(j) ~= posLatAnt)&&(lon(j) ~= posLonAnt)) 
            
            %Vetor de leituras
            z = [lon(j)
                 lat(j)
                 accXFinal
                 accYFinal ];

            H = [1, 0, 0, 0, 0, 0
                 0, 1, 0, 0, 0, 0             
                 0, 0, 0, 0, 1, 0
                 0, 0, 0, 0, 0, 1];

            R = [varLon,    0,      0,          0
                    0,      varLat, 0,          0
                    0,      0,      varAccX,    0
                    0,      0,      0,          varAccY];

            
            
            posLatAnt = lat(j);
            posLonAnt = lon(j);
        else
            
            %Vetor de leituras
            z = [accXFinal
                 accYFinal];

            H = [0, 0,  0,  0,  1,  0
                 0, 0,  0,  0,  0,  1];

            R = [varAccX, 0; 0, varAccY];
            
            cont = 0;
        end
        
            
        %Vetor de leituras
        z = [lon(j)
             lat(j)
             accXFinal
             accYFinal ];
        
        H = [1, 0, 0, 0, 0, 0
             0, 1, 0, 0, 0, 0             
             0, 0, 0, 0, 1, 0
             0, 0, 0, 0, 0, 1];
        
        R = [varLon,    0,      0,          0
                0,      varLat, 0,          0
                0,      0,      varAccX,    0
                0,      0,      0,          varAccY];
        
        cont = 0;
            
    else
          
        %Vetor de leituras
        z = [accXFinal
             accYFinal];

        H = [0, 0,  0,  0,  1,  0
             0, 0,  0,  0,  0,  1];

        R = [varAccX, 0; 0, varAccY];
    end
    
    
    %Estima o novo estado a partir da matriz A e do estado anterior
    x_est = A*x_ant;
    
    %Atualiza matriz P
    P = A*P*transpose(A) + Q;
    
    
    % Calcula o ganho de Kalman
    %  K =  P * transpose(H)
    %     --------------------
    %    H*P*transpose(H) + R
    %
    aux = (H*P*transpose(H)) + R;
    K = P*transpose(H)*inv(aux);
    
    
    %Observa os valores de aceleracao da IMU
    y = z-(H*x_est);
    
    %Atualizo o estado atual
    aux2 = K*y;
    x_atual = x_est + K*y;
    x_ant = x_atual;
    %DEBUG
    estados(1,i) = d
    estimado(1,i) = x_est(1);
    estimado(2,i) = x_est(2);
    estimado(3,i) = x_est(3);
    estimado(4,i) = x_est(4);
    estimado(5,i) = x_est(5);
    estimado(6,i) = x_est(6);
    estimado(7,i) = yaw * 180/pi;
    
    angulos(1,i) = yaw*180/pi;
    angulos(2,i) = yaw_a*180/pi;
    angulos(3,i) = yaw_dcm*180/pi;
    angulos(4,i) = yaw_filtrado*180/pi;
    
    angulos(5,i) = roll*180/pi;
    angulos(6,i) = roll_a*180/pi;
    angulos(7,i) = roll_dcm*180/pi;
    angulos(8,i) = roll_filtrado*180/pi;
    
    angulos(9,i) = pitch*180/pi;
    angulos(10,i) = pitch_a*180/pi;
    angulos(11,i) = pitch_dcm*180/pi;
    angulos(12,i) = pitch_filtrado*180/pi;
    
      
    %Atualiza matriz P
    P = (I-K*H)*P;
    
    %desenha_elipse(3^2*P,[estimado(1,i) estimado(2,i)],'k');
    
    %Incrementa contador chamada GPS
    cont = cont + 1;
    

       
end



figure(1)
plot(estados(1,:),estados(2,:),'r+')
hold on
plot(lon,lat,'b*')


step = 1 : numAmostras;
figure(2)
%plot(step,vec,'r')
%hold on
%plot(step,angulo,'b+')
%hold on
plot(step,angulos(1,:),'r')
hold on
plot(step,angulos(2,:),'b')
hold on
plot(step,angulos(3,:),'g*')
hold on
plot(step,angulos(4,:),'K')
title('Yaw');
%plot(stepGPS,gps,'K+')

figure(3)
%plot(step,vec,'r')
%hold on
%plot(step,angulo,'b+')
%hold on
plot(step,angulos(5,:),'r')
hold on
plot(step,angulos(6,:),'b')
hold on
plot(step,angulos(7,:),'g*')
hold on
plot(step,angulos(8,:),'K')
title('Roll');

figure(4)
%plot(step,vec,'r')
%hold on
%plot(step,angulo,'b+')
%hold on
plot(step,angulos(9,:),'r')
hold on
plot(step,angulos(10,:),'b')
hold on
plot(step,angulos(11,:),'g*')
hold on
plot(step,angulos(12,:),'K')
title('Pitch');


