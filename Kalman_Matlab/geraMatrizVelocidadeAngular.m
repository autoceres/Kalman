function [ mW ] = geraMatrizVelocidadeAngular( wx, wy, wz )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

        mW = [ 0,   -wz,    wy
               wz,   0,    -wx
              -wy,   wx,     0];
end

