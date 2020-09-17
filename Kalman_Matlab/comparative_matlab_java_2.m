angulos_java = csvread("datas_java/angulos_java.csv")
  
% Yaw
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
 title('Yaw - Matlab');
 %plot(stepGPS,gps,'K+')
 legend('Yaw AHRS', 'Yaw Android', 'Yaw DCM', 'Yaw Filtrado')
 
 % Roll
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
 title('Roll - Matlab');
 legend('Roll AHRS', 'Roll Android', 'Roll DCM', 'Roll Filtrado')
 
 % Pitch
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
 title('Pitch - Matlab');
 legend('Pitch AHRS', 'Pitch Android', 'Pitch DCM', 'Pitch Filtrado')


% Yaw
step = 1 : numAmostras;
 figure(5)
 %plot(step,vec,'r')
 %hold on
 %plot(step,angulo,'b+')
 %hold on
 plot(step,angulos_java(1,:),'r')
 hold on
 plot(step,angulos_java(2,:),'b')
 hold on
 plot(step,angulos_java(3,:),'g*')
 hold on
 plot(step,angulos_java(4,:),'K')
 title('Yaw - Java');
 %plot(stepGPS,gps,'K+')
 legend('Yaw AHRS', 'Yaw Android', 'Yaw DCM', 'Yaw Filtrado')
 
 % Roll
 figure(6)
 %plot(step,vec,'r')
 %hold on
 %plot(step,angulo,'b+')
 %hold on
 plot(step,angulos_java(5,:),'r')
 hold on
 plot(step,angulos_java(6,:),'b')
 hold on
 plot(step,angulos_java(7,:),'g*')
 hold on
 plot(step,angulos_java(8,:),'K')
 title('Roll - Java');
 legend('Roll AHRS', 'Roll Android', 'Roll DCM', 'Roll Filtrado')
 
 % Pitch
 figure(7)
 %plot(step,vec,'r')
 %hold on
 %plot(step,angulo,'b+')
 %hold on
 plot(step,angulos_java(9,:),'r')
 hold on
 plot(step,angulos_java(10,:),'b')
 hold on
 plot(step,angulos_java(11,:),'g*')
 hold on
 plot(step,angulos_java(12,:),'K')
 title('Pitch - Java');
 legend('Pitch AHRS', 'Pitch Android', 'Pitch DCM', 'Pitch Filtrado')

