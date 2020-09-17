%write data to end of file
% ------------------------------------------------------------------------------------------------------------------------------
% Varianza
%GPS
dlmwrite('datas/latVariancia.csv', latVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/lonVariancia.csv', lonVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

%IMU
dlmwrite('datas/gravXVariancia.csv', gravXVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gravYVariancia.csv', gravYVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gravZVariancia.csv', gravZVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

dlmwrite('datas/accXVariancia.csv', accXVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/accYVariancia.csv', accYVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/accZVariancia.csv', accZVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

dlmwrite('datas/magXVariancia.csv', magXVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/magYVariancia.csv', magYVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/magZVariancia.csv', magZVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

dlmwrite('datas/gyroXVariancia.csv', gyroXVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gyroYVariancia.csv', gyroYVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gyroZVariancia.csv', gyroZVariancia, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
% ------------------------------------------------------------------------------------------------------------------------------
%GPS
dlmwrite('datas/lat.csv', lat, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/lon.csv', lon, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

%IMU
dlmwrite('datas/gravX.csv', gravX, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gravY.csv', gravY, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gravZ.csv', gravZ, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

dlmwrite('datas/accX.csv', accX, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/accY.csv', accY, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/accZ.csv', accZ, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

dlmwrite('datas/magX.csv', magX, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/magY.csv', magY, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/magZ.csv', magZ, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');

dlmwrite('datas/gyroX.csv', gyroX, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gyroY.csv', gyroY, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');
dlmwrite('datas/gyroZ.csv', gyroZ, 'delimiter', ' ', 'newline', 'pc', 'precision', '%5.14f');