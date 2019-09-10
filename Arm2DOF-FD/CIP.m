function [a,  b,  c,  d] = CIP(q_0,q_1, w_0, w_1, T)
%CUBIC_INTERPOLATING_POLYNOMIAL Summary of this function goes here
%   Detailed explanation goes here


a = q_0;
b = w_0;
c = ( 3*(q_1-q_0)- T*(2*w_0+w_1) ) / T^2;
d = ( 2*(q_0-q_1) - T*(w_0+w_1) )/ T^3; 



end

