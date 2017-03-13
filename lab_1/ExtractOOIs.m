function [ooi_list] = ExtractOOIs(range, intensity)
% A function to sort objects in LIDAR range data into clusters
% Ian Bartlett

threshold = 0.2;

angles = [0:360]'*0.5*pi/180;

X_pos = cos(angles).*range;
Y_pos = sin(angles).*range;


%Find the gaps
object_edges = [0 (find(sqrt(diff(X_pos).^2+diff(Y_pos).^2) > threshold))'];

%If the difference between the points is greater than threshold,
%create a new object, assign to it
%For each object, compute center, size, and reflectivity check

num_obs = length(object_edges)-1;
ooi_list = struct('N',num_obs, ...
                  'Centers',zeros(2, num_obs), ...
		  'Diameters',zeros(1, num_obs), ...
		  'Color',zeros(1, num_obs));

for i = 1:ooi_list.N

    object_X = X_pos(object_edges(i)+1:object_edges(i+1));
    object_Y = Y_pos(object_edges(i)+1:object_edges(i+1));

    %Use the mean of the X,Y points as the centre
    ooi_list.Centers(:,i) = [median(object_X),median(object_Y)];

    %Use the Euclidian distance from first to last point as the size
    %If it is outside the valid distance range, reject. 
    ooi_list.Diameters(i) = norm([object_X(end)-object_X(1)],object_Y(end)-object_Y(1));
    if (ooi_list.Diameters(i) > 0.2 | ooi_list.Diameters(i) < 0.05)
	   ooi_list.Diameters(i) = -1; 
   end

    %Check if any pixels are reflective
    if max(intensity(object_edges(i)+1:object_edges(i+1)) > 0)
	ooi_list.Color(i) = 1;
    else
	ooi_list.Color(i) = 0;
    end

end	

% Clean up invalid objects
ooi_list.Color(find(ooi_list.Diameters < 0)) = [];
ooi_list.Centers(:,find(ooi_list.Diameters < 0)) = [];
ooi_list.Diameters(find(ooi_list.Diameters < 0)) = [];
ooi_list.N = length(ooi_list.Diameters);

end
