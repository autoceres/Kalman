function [ mCos ] = geraMatrizCossenosDiretores( r, p, y )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    mCos = [ cos(y)*cos(p),     cos(y)*sin(p)*sin(r)-sin(y)*cos(r),         cos(y)*sin(p)*cos(r)-sin(y)*sin(r)
             sin(y)*cos(p),     sin(y)*sin(p)*sin(r)-cos(y)*cos(r)*cos(p),  sin(y)*sin(p)*cos(r)-cos(y)*sin(r)
             -sin(p),           cos(p)*sin(r),                              cos(p)*cos(r)];
         
    


end

