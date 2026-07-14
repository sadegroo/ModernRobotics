function [S, theta] = AxisAng6(expc6)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 6-vector of exponential coordinates for rigid-body motion
% S*theta. 
% Returns S: the corresponding normalized screw axis,
%         theta: the distance traveled along/about S.
% Example Input:
% 
% clear; clc;
% expc6 = [1; 0; 0; 1; 2; 3];
% [S, theta] = AxisAng6(expc6)
%  
% Output:
% S =
%     1
%     0
%     0 
%     1
%     2
%     3
% theta =
%     1

theta = norm(expc6(1: 3));
if NearZero(theta)
    theta = norm(expc6(4: 6));
end
if isa(theta, 'sym')
    % Symbolic norm() wraps real expressions in abs(); strip it so results
    % stay clean without assuming positive angles. This just flips the sign
    % convention of theta, and S = expc6 / theta flips consistently with it.
    % simplify() first: norm() returns (abs(x)^2)^(1/2), which only
    % collapses to abs(x) after simplification.
    theta = mapSymType(simplify(theta), 'abs', @(x) children(x, 1));
end
S = expc6 / theta;
end