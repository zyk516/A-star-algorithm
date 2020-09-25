% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;

set(gcf, 'Renderer', 'painters');
set(gcf, 'Position', [500, 50, 700, 700]);

% Environment map in 2D space 
xStart = 5.0;
yStart = 9.0;
xTarget =3.0 ;
yTarget = 2.0;
MAX_X = 10;
MAX_Y = 10;
%save map,where begin where end,and where is obstacle
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);
%map=[9,9;1,7;2,10;3,4;3,6;4,4;4,9;4,10;5,8;6,2;6,4;6,5;6,10;7,2;7,3;8,1;8,4;8,6;8,7;8,8;9,5;9,7;10,3;10,8;10,9;1,1];
% Waypoint Generator Using the A*
%path=[];
%visualize_map(map, path);
path = A_star_search(map, MAX_X,MAX_Y);
% visualize the 2D grid map
visualize_map(map, path);
