function [m2gX, m2gY] = geraFatoresConversaoMetros2Graus(lat,lon)


    %Define as referencias antes de iniciar os calculos
    refLat = lat;
    refLon = lon;
    [refX, refY, refZona] = conversao_coordenadas_geo2utm(refLat, refLon);
    
    
    %Calcula ganho na direção do eixo X
    novoX = refX + 5000;
    [novoLat, novoLon] = conversao_coordenadas_utm2geo(novoX, refY, refZona);
    difLon = refLon - novoLon;
    m2gX = difLon/5000;
    
        
    %Calcula ganho na direção do eixo Y
    novoY = refY + 5000;
    [novoLat, novoLon] = conversao_coordenadas_utm2geo(refX, novoY, refZona);
    difLat = refLat - novoLat;   
    m2gY = difLat/5000;

end
