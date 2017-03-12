function [points] = circlePoints(center, radius)
  
    angles = [0:0.5:360]'*pi/180;
    points = [radius*cos(angles)+center(1), radius*sin(angles)+center(2)];

end
