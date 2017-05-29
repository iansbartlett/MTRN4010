%Author: Ian Bartlett, z3419581
%Program: A utility function to integrate 3D gyro data.

function [n_att] = integrate_gyros3d(rates,T, o_att)
% A function to integrate discrete gyroscope data
%
% Inputs:
% rates - vector of rotation rates
% T - integration timestep
% o_att - old attitute vector of Euler angles, in counts 
%
% Returns:
% n_att - new estimated attitude vector of Euler angles

n_att = zeros(3,1);

% Unit conversions - NO CONVERSIONS NOW
%rates = rates*rate_const;
%o_att = o_att*att_const;

% Discrete integration, not vectorized:

n_att(1) = o_att(1) + ...
          (rates(1)+(rates(2)*sin(o_att(1))+rates(3)*cos(o_att(1)))*tan(o_att(2)))*T;
n_att(2) = o_att(2) + ...
          ((rates(2)*cos(o_att(1))-rates(3)*sin(o_att(1))))*T; 
n_att(3) = o_att(3) + ...
          (rates(2)*sin(o_att(1))+rates(3)*cos(o_att(1)))*(sec(o_att(2)))*T;

end
