function [matrizRotacao] = geraMatrizRotacao(accX,accY,accZ,magX,magY,magZ,gyroX,gyroY,gyroZ)
        
        Ax = accX;
        Ay = accY;
        Az = accZ;
        normsqA = (Ax * Ax + Ay * Ay + Az * Az);
        g = 9.81;
        freeFallGravitySquared = 0.01 * g * g;
        matrizRotacao = zeros(1,9);
        
        
        if (normsqA < freeFallGravitySquared) 
                       
        else

            Ex = magX;
            Ey = magY;
            Ez = magZ;
            Hx = Ey * Az - Ez * Ay;
            Hy = Ez * Ax - Ex * Az;
            Hz = Ex * Ay - Ey * Ax;
            normH = sqrt(Hx * Hx + Hy * Hy + Hz * Hz);
            if (normH < 0.1) 
             
            else
                invH = 1.0 / normH;
                Hx = Hx * invH;
                Hy = Hy * invH;
                Hz = Hz * invH;
                invA = 1.0 / sqrt(Ax * Ax + Ay * Ay + Az * Az);
                Ax = Ax * invA;
                Ay = Ay * invA;
                Az = Az * invA;
                Mx = Ay * Hz - Az * Hy;
                My = Az * Hx - Ax * Hz;
                Mz = Ax * Hy - Ay * Hx;

                R(1) = Hx;     R(2) = Hy;     R(3) = Hz;
                R(4) = Mx;     R(5) = My;     R(6) = Mz;
                R(7) = Ax;     R(8) = Ay;     R(9) = Az;
                matrizRotacao = R;

            end
            
            
        end




% 
%     %Usando Madgwick
%     beta = 0.1; % 2x ganho proporcional
%     sampleFreqDef = 50.0; % Frequencia de amostragem em HZ (Experimentar mudar para 50Hz)
%     q0 = 1.0;
%     q1 = 0.0;
%     q2 = 0.0;
%     q3 = 0.0;
%     invSampleFreq = 1.0/sampleFreqDef;
%     
%     ax = accX;
%     ay = accY;
%     az = accZ;
%     mx = magX;
%     my = magY;
%     mz = magZ;
%     gx = gyroX;
%     gy = gyroY;
%     gz = gyroZ;
%     
%     
%     qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
% 	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
% 	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
% 	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
%     
%     
%     %Normalise accelerometer measurement
%     recipNorm = 1/(sqrt((ax * ax + ay * ay + az * az)));
%     ax = ax * recipNorm;
%     ay = ay * recipNorm;
%     az = az * recipNorm;
%     
%     %Normalise magnetometer measurement
%     recipNorm = 1/(sqrt((mx * mx + my * my + mz * mz)));
%     mx = mx * recipNorm;
%     my = my * recipNorm;
%     mz = mz * recipNorm;
%     
%     
%     aux2q0mx = 2.0 * q0 * mx;
%     aux2q0my = 2.0 * q0 * my;
%     aux2q0mz = 2.0 * q0 * mz;
%     aux2q1mx = 2.0 * q1 * mx;
%     aux2q0 = 2.0 * q0;
%     aux2q1 = 2.0 * q1;
%     aux2q2 = 2.0 * q2;
%     aux2q3 = 2.0 * q3;
%     aux2q0q2 = 2.0 * q0 * q2;
%     aux2q2q3 = 2.0 * q2 * q3;
%     q0q0 = q0 * q0;
%     q0q1 = q0 * q1;
%     q0q2 = q0 * q2;
%     q0q3 = q0 * q3;
%     q1q1 = q1 * q1;
%     q1q2 = q1 * q2;
%     q1q3 = q1 * q3;
%     q2q2 = q2 * q2;
%     q2q3 = q2 * q3;
%     q3q3 = q3 * q3;
%     
%     
%     
%     %Reference direction of Earth's magnetic field
%     hx = mx * q0q0 - aux2q0my * q3 + aux2q0mz * q2 + mx * q1q1 + aux2q1 * my * q2 + aux2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
%     hy = aux2q0mx * q3 + my * q0q0 - aux2q0mz * q1 + aux2q1mx * q2 - my * q1q1 + my * q2q2 + aux2q2 * mz * q3 - my * q3q3;
%     aux2bx = sqrt(hx * hx + hy * hy);
%     aux2bz = -aux2q0mx * q2 + aux2q0my * q1 + mz * q0q0 + aux2q1mx * q3 - mz * q1q1 + aux2q2 * my * q3 - mz * q2q2 + mz * q3q3;
%     aux4bx = 2.0 * aux2bx;
%     aux4bz = 2.0 * aux2bz;
%     
%     %Gradient decent algorithm corrective step
%     s0 = -aux2q2 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q1 * (2.0 * q0q1 + aux2q2q3 - ay) - aux2bz * q2 * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (-aux2bx * q3 + aux2bz * q1) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + aux2bx * q2 * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz);
%     s1 = aux2q3 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q0 * (2.0 * q0q1 + aux2q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + aux2bz * q3 * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (aux2bx * q2 + aux2bz * q0) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + (aux2bx * q3 - aux4bz * q1) * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz);
%     s2 = -aux2q0 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q3 * (2.0 * q0q1 + aux2q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-aux4bx * q2 - aux2bz * q0) * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (aux2bx * q1 + aux2bz * q3) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + (aux2bx * q0 - aux4bz * q2) * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz);
%     s3 = aux2q1 * (2.0 * q1q3 - aux2q0q2 - ax) + aux2q2 * (2.0 * q0q1 + aux2q2q3 - ay) + (-aux4bx * q3 + aux2bz * q1) * (aux2bx * (0.5 - q2q2 - q3q3) + aux2bz * (q1q3 - q0q2) - mx) + (-aux2bx * q0 + aux2bz * q2) * (aux2bx * (q1q2 - q0q3) + aux2bz * (q0q1 + q2q3) - my) + aux2bx * q1 * (aux2bx * (q0q2 + q1q3) + aux2bz * (0.5 - q1q1 - q2q2) - mz);
%     recipNorm = 1/(sqrt((s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3))); % normalise step magnitude
%     s0 = s0 * recipNorm;
%     s1 = s1 * recipNorm;
%     s2 = s2 * recipNorm;
%     s3 = s3 * recipNorm;
%     
%     %Apply feedback step
%     qDot1 = qDot1 - (beta * s0);
%     qDot2 = qDot2 - (beta * s1);
%     qDot3 = qDot3 - (beta * s2);
%     qDot4 = qDot4 - (beta * s3);
% 
%     q0 = q0 + (qDot1 * invSampleFreq);
% 	q1 = q1 + (qDot2 * invSampleFreq);
% 	q2 = q2 + (qDot3 * invSampleFreq);
% 	q3 = q3 + (qDot4 * invSampleFreq);
%     
%     recipNorm = 1/(sqrt((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)));
% 	q0 = q0 * recipNorm;
% 	q1 = q1 * recipNorm;
% 	q2 = q2 * recipNorm;
% 	q3 = q3 * recipNorm;
%     
%     roll = atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
% 	pitch = asin(-2.0 * (q1*q3 - q0*q2));
% 	yaw = atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
%     
%     roll_r = (roll*pi)/180;
%     pitch_r = (pitch*pi)/180;
%     yaw_r = (yaw*pi)/180;
% 
%   % Calcula cada item da matriz individualmente 
%   R(1) = cos(yaw_r)*cos(pitch_r);
%   R(2) = (cos(yaw_r)*sin(roll_r)*sin(pitch_r))-(cos(roll_r)*sin(yaw_r));
%   R(3) = (sin(roll_r)*sin(yaw_r))+(cos(roll_r)*cos(yaw_r)*sin(pitch_r));
% 
%   R(4) = cos(pitch_r)*sin(yaw_r);
%   R(5) = (cos(roll_r)*cos(yaw_r))+(sin(roll_r)*sin(yaw_r)*sin(pitch_r));
%   R(6) = (cos(roll_r)*sin(yaw_r)*sin(pitch_r))-(cos(yaw_r)*sin(roll_r));
% 
%   R(7) = sin(pitch_r) * (-1);
%   R(8) = cos(pitch_r)*sin(roll_r);
%   R(9) = cos(roll_r)*cos(pitch_r);
%     
%   matrizRotacao = R;
%   
%   r = roll;
%   p = pitch;
%   y = yaw;
end
