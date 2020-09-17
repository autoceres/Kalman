function [ R ] = quartenion2MatrizRotacao( escalar, x, y, z )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    %MATLAB built in funcion
    quat = [escalar, x, y, z];
    R = quat2rotm(quat);
    
    

end

