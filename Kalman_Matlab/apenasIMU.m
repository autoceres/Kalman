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
Q = eye(6)*0.03;

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


[matrizRotacao, roll, pitch, yaw] = geraMatrizRotacao(accX(1),accY(1),accZ(1), magX(1), magY(1), magZ(1), gyroX(1), gyroY(1), gyroZ(1));

vecOrientacao = pegaOrientacao(matrizRotacao);
angAnt = vecOrientacao(1);

for i=1 : numAmostras
    
    %Ajusta os valores de acelerecao
    gyro = [gyroX(i), gyroY(i), gyroZ(i)];
    acc = [accX(i), accY(i), accZ(i)];
    mag = [magX(i), magY(i), magZ(i)];
    AHRS.Update(gyro, acc, mag);	% gyroscope units must be radians
    quaternion(i, :) = AHRS.Quaternion;

    euler = quatern2euler(quaternConj(quaternion));
    
    roll = euler(i,1);
    pitch = euler(i,2);
    yaw = euler(i,3);
    
    %matrizCos = geraMatrizCossenosDiretores(roll, pitch, yaw);

    %matrizVelAng = geraMatrizVelocidadeAngular(magX(1), magY(1), magZ(1));

    %matrizRotacao = matrizCos;
        
    %accXRot = matrizRotacao(1)*(accX(i)-gravX(i)) + matrizRotacao(2)*(accY(i)-gravY(i)) + matrizRotacao(3)*(accZ(i)-gravZ(i));
    %accYRot = matrizRotacao(4)*(accX(i)-gravX(i)) + matrizRotacao(5)*(accY(i)-gravY(i)) + matrizRotacao(6)*(accZ(i)-gravZ(i));   
    %accZRot = matrizRotacao(7)*(accX(i)-gravX(i)) + matrizRotacao(8)*(accY(i)-gravY(i)) + matrizRotacao(9)*(accZ(i)-gravZ(i));   
    
    %gyroXRot = matrizRotacao(1)*(gyroX(i)) + matrizRotacao(2)*(gyroY(i)) + matrizRotacao(3)*(gyroZ(i));
    %gyroYRot = matrizRotacao(4)*(gyroX(i)) + matrizRotacao(5)*(gyroY(i)) + matrizRotacao(6)*(gyroZ(i));
    %gyroZRot = matrizRotacao(7)*(gyroX(i)) + matrizRotacao(8)*(gyroY(i)) + matrizRotacao(9)*(gyroZ(i));
    
    %magXRot = matrizRotacao(1)*(magX(i)) + matrizRotacao(2)*(magY(i)) + matrizRotacao(3)*(magZ(i));
    %magYRot = matrizRotacao(4)*(magX(i)) + matrizRotacao(5)*(magY(i)) + matrizRotacao(6)*(magZ(i));
    %magZRot = matrizRotacao(7)*(magX(i)) + matrizRotacao(8)*(magY(i)) + matrizRotacao(9)*(magZ(i));
    
    accXFinal = accX(i)*sin(yaw);
    accYFinal = accY(i)*cos(yaw);
        
    accYFinal = accYFinal * m2gY;
    accXFinal = accXFinal * m2gX;
    
    
     
    
    % -------------------------------------------------------------------
    
%     if (cont == 48)
%         
%         %Pega coordenadas UTM
%         j = j + 1;
% 
%         %-----------------
%         if ((lat(j) ~= posLatAnt)&&(lon(j) ~= posLonAnt)) 
%             
%             %Vetor de leituras
%             z = [lon(j)
%                  lat(j)
%                  accXFinal
%                  accYFinal ];
% 
%             H = [1, 0, 0, 0, 0, 0
%                  0, 1, 0, 0, 0, 0             
%                  0, 0, 0, 0, 1, 0
%                  0, 0, 0, 0, 0, 1];
% 
%             R = [varLon,    0,      0,          0
%                     0,      varLat, 0,          0
%                     0,      0,      varAccX,    0
%                     0,      0,      0,          varAccY];
% 
%             
%             
%             posLatAnt = lat(j);
%             posLonAnt = lon(j);
%         else
%             
%             %Vetor de leituras
%             z = [accXFinal
%                  accYFinal];
% 
%             H = [0, 0,  0,  0,  1,  0
%                  0, 0,  0,  0,  0,  1];
% 
%             R = [varAccX, 0; 0, varAccY];
%             
%             cont = 0;
%         end
%         
%             
%         %Vetor de leituras
%         z = [lon(j)
%              lat(j)
%              accXFinal
%              accYFinal ];
%         
%         H = [1, 0, 0, 0, 0, 0
%              0, 1, 0, 0, 0, 0             
%              0, 0, 0, 0, 1, 0
%              0, 0, 0, 0, 0, 1];
%         
%         R = [varLon,    0,      0,          0
%                 0,      varLat, 0,          0
%                 0,      0,      varAccX,    0
%                 0,      0,      0,          varAccY];
%         
%         cont = 0;
%             
%     else
          
        %Vetor de leituras
        z = [accXFinal
             accYFinal];

        H = [0, 0,  0,  0,  1,  0
             0, 0,  0,  0,  0,  1];

        R = [varAccX, 0; 0, varAccY];
   % end
    
    
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
    estados(1,i) = x_atual(1);
    estados(2,i) = x_atual(2);
    estados(3,i) = x_atual(3);
    estados(4,i) = x_atual(4);
    estados(5,i) = x_atual(5);
    estados(6,i) = x_atual(6);
    estimado(1,i) = x_est(1);
    estimado(2,i) = x_est(2);
    estimado(3,i) = x_est(3);
    estimado(4,i) = x_est(4);
    estimado(5,i) = x_est(5);
    estimado(6,i) = x_est(6);
    estimado(7,i) = 1;%;angFinal * 180/pi;
      
    %Atualiza matriz P
    P = (I-K*H)*P;
    
    %desenha_elipse(3^2*P,[estimado(1,i) estimado(2,i)],'k');
    
    %Incrementa contador chamada GPS
    cont = cont + 1;
    

       
end



figure(1)
%plot(lon,lat,'K*')
%hold on
plot(estados(1,:),estados(2,:),'g')

step = 1 : numAmostras;
%figure(2)
%plot(step,vec,'r')
%hold on
%plot(step,angulo,'b+')
%hold on
%plot(step,estimado(7,:),'r')
%plot(stepGPS,gps,'K+')

%figure(3)
%plot(estados(5,:),estados(6,:),step)


