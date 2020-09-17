 estados_java = csvread("datas_java/estados_java.csv")
 estimados_java = csvread("datas_java/estimados_java.csv")
 
figure(1)
plot(estados(1,:),estados(2,:),'r+')
hold on
plot(lon,lat,'b*')
hold on 
plot(estimado(1,:),estimado(2,:),'g+')

legend('estado', 'medida', 'estimado')
title('MATLAB')
axis([-52.1688 -52.1684 -32.07266 -32.0723]) 
grid

figure(2)
plot(estados_java(1,:),estados_java(2,:),'r+')
hold on
plot(lon,lat,'b*')
hold on 
plot(estimados_java(1,:),estimados_java(2,:),'g+')

legend('estado', 'medida', 'estimado')
title('JAVA')
axis([-52.1688 -52.1684 -32.07266 -32.0723]) 
grid