function [omghat, theta] = AxisAng3(expc3)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes A 3-vector of exponential coordinates for rotation.
% Returns the unit rotation axis omghat and the corresponding rotation 
% angle theta.
% Example Input:
% 
% clear; clc;
% expc3 = [1; 2; 3];
% [omghat, theta] = AxisAng3(expc3)  
% 
% Output:
% omghat =
%    0.2673
%    0.5345
%    0.8018
% theta =
%    3.7417

theta = norm(expc3);
if isa(theta, 'sym')
    % Symbolic norm() wraps real expressions in abs(), which pollutes all
    % downstream results (e.g. sin(abs(th)) in MatrixExp3/MatrixExp6) unless
    % the angles are assumed positive. Stripping abs() is exact here: it
    % amounts to picking the opposite sign for theta, and omghat = expc3 /
    % theta flips along with it, so the axis-angle pair stays consistent.
    % simplify() first: norm() returns (abs(x)^2)^(1/2), which only
    % collapses to abs(x) after simplification.
    theta = mapSymType(simplify(theta), 'abs', @(x) children(x, 1));
end
omghat = expc3 / theta;
end