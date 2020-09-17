function [ M ] = matrixRot( r,p,y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    M(1) = cos(y)*cos(p);
    M(2) = cos(y)*sin(r)*sin(p) - cos(r)*sin(y);
    M(3) = sin(r)*sin(y) + cos(r)*cos(y)*sin(p);
    
    M(4) = cos(p)*sin(y);
    M(5) = cos(r)*cos(y)+sin(r)*sin(y)*sin(p);
    M(6) = cos(r)*sin(y)*sin(p) - cos(y)*sin(r);
    
    M(7) = -sin(p);
    M(8) = cos(p)*sin(r);
    M(9) = cos(r)*cos(p);
    
end

