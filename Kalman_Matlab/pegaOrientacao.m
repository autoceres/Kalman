function [vecOrientacao] = pegaOrientacao (matrizRotacao)
        
        R = matrizRotacao;
        
        %public static float[] getOrientation(float[] R, float[] values) {

         % 3x3 (length=9) case:
         %   /  R[ 0]   R[ 1]   R[ 2]  \
         %   |  R[ 3]   R[ 4]   R[ 5]  |
         %   \  R[ 6]   R[ 7]   R[ 8]  /
         

            vecOrientacao(1) = atan2(R(2), R(5));
            vecOrientacao(2) = asin(-R(8));
            vecOrientacao(3) = atan2(-R(7), R(9));


end

    


