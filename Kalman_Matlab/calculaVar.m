function [varLat, varLon, varAccX, varAccY, varAng] = calculaVar (latVariancia, lonVariancia, accXVariancia, accYVariancia, gravXVariancia, gravYVariancia, angVar)

    %Calcula varianca para as coordenadas geodesicas
    numAmostras = size(latVariancia);
    varLat = 0.0;
    mediaLat = 0.0;
    varLon = 0.0;
    mediaLon = 0.0;
    
    for i=1:numAmostras(1)
        mediaLat = mediaLat + latVariancia(i);
        mediaLon = mediaLon + lonVariancia(i);
    end

    mediaLat = mediaLat/numAmostras(1);
    mediaLon = mediaLon/numAmostras(1);
    
    for i=1:numAmostras(1)
        varLat = varLat + ( ((latVariancia(i)-mediaLat)^2)/numAmostras(1) );
        varLon = varLon + ( ((lonVariancia(i)-mediaLon)^2)/numAmostras(1) );
    end
    
    
    
    %Calcula varianca para as aceleracoes
    numAmostrasX = size(accXVariancia);
    varAccX = 0.0;
    mediaAccX = 0.0;
    varAccY = 0.0;
    mediaAccY = 0.0;
    
    for i=1:numAmostrasX(1)
        mediaAccX = mediaAccX + (accXVariancia(i)-gravXVariancia(i));
        mediaAccY = mediaAccY + (accYVariancia(i)-gravYVariancia(i));
    end

    mediaAccX = mediaAccX/numAmostrasX(1);
    mediaAccY = mediaAccY/numAmostrasX(1);
    
    for i=1:numAmostrasX(1)
        varAccX = varAccX + ( (((accXVariancia(i)-gravXVariancia(i))-mediaAccX)^2)/numAmostrasX(1) );
        varAccY = varAccY + ( (((accYVariancia(i)-gravYVariancia(i))-mediaAccY)^2)/numAmostrasX(1) );
    end
    
    
    %Calcula varianca para o angulo de heading
     numAmostras = size(accXVariancia);
     varAng = 0.0;
     mediaAng = 0.0;
     
     
     
     
     for i=1:numAmostras(1)
         mediaAng = mediaAng + angVar(i);
     end
 
     mediaAng = mediaAng/numAmostras(1);
     
     for i=1:numAmostras(1)
         varAng = varAng + (((angVar(i)-mediaAng)^2)/numAmostras(1));
     end
     
    
    
    
end

        
        
        
        
        