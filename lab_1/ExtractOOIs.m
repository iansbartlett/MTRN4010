function [ooi_list] = ExtractOOIs(range, intensity)
% A function to sort objects in LIDAR range data into clusters
% Ian Bartlett

threshold = 0.5;

angles = [0:360]'*0.5*pi/180;

%Find the gaps
object_edges = [1 find(diff(range) > threshold)'];

%If the difference between the points is greater than threshold,
%create a new object, assign to it
%For each object, compute center, size, and reflectivity check

num_obs = length(object_edges)-1;
ooi_list = struct('N',num_obs, ...
                  'Centers',zeros(2, num_obs), ...
		  'Diameters',zeros(1, num_obs), ...
		  'Color',zeros(1, num_obs));

for i = 1:ooi_list.N

    object_ranges = range(object_edges(i):object_edges(i+1));
    object_angles = angles(object_edges(i):object_edges(i+1));

    object_X = cos(object_angles).*object_ranges;   
    object_Y = sin(object_angles).*object_ranges;   

    %Use the mean of the X,Y points as the centre
    ooi_list.Centers(:,i) = [mean(object_X),mean(object_Y)];

    %Use the Euclidian distance from first to last point as the size
    ooi_list.Diameters(i) = norm([object_X(end)-object_X(1)],object_Y(end)-object_Y(1));

    %Check if any pixels are reflective
    if max(intensity(object_edges(i):object_edges(i+1)) > 0)
	ooi_list.Color(i) = 1;
    else
	ooi_list.Color(i) = 0;
    end

end	

disp(ooi_list.Color(:))

end
