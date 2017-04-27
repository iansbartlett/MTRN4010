%Master run file for robot localization lab (MTRN4010) 
%Ian Bartlett

%TODO
% - Integrations aren't working atm- some kind of unit error, I think
% - Function-ify the individual integration steps. Do all the live plotting
%   at once, with all three plots updating at each step. 

function [] = lab_2()

time_conv_factor = 10000;

IMU_data = load('DataForProject02/IMU_dataB.mat');
IMU_times = double(IMU_data.IMU.times)/time_conv_factor;
IMU_accel = IMU_data.IMU.DATAf(1:3,:)';
IMU_omega = IMU_data.IMU.DATAf(4:6,:)';

IMU_times = IMU_times - IMU_times(1);

% 2D change of attitude representation
IMU_omega(:,3) = -1*IMU_omega(:,3);

speed_data = load('DataForProject02/Speed_dataB.mat');
velocity = speed_data.Vel.speeds;

laser_data = load('DataForProject02/Laser__2.mat');
laser_times = double(laser_data.dataL.times)/time_conv_factor;
laser_times = laser_times - laser_times(1);
laser_scans = laser_data.dataL.Scans;

handles = initialize_plots(IMU_times);

%Extract linear trend
start_index = find(IMU_times - IMU_times(1) < 20,1,'last');
omega_offset = mean(IMU_omega(1:start_index,3));
IMU_omega(:,3) = IMU_omega(:,3) - omega_offset;

%Part A and B: Integrate the gyro data, obtain dead-reckoning
%estimates of our attitude and X-Y position
%Run the plotting loop:

initial_state = [0 0 pi/2]; %Initial x, y, theta

state = zeros(length(IMU_times),3);

state(1,:) = initial_state;

for i = 1:length(IMU_times)-1
	dt = IMU_times(i+1) - IMU_times(i);
    state(i+1,:) = int_state(IMU_omega(i,3),velocity(i),dt,state(i,:));
end

set(handles.att_trace,'xdata',IMU_times(1:i),'ydata',(180/pi)*state(1:i,3));
set(handles.xy_trace,'xdata',state(:,1),'ydata',state(:,2));

% Parts B and C - data fusion and object associtaion

% Initialize and plot starting positions of OOIs
% Process first scan
[range_0, intensity_0] = extract_laser_data(laser_scans(:,1));
plot_scan(range_0, intensity_0, state(1,:), handles);
global_OOI_list = ExtractOOIs(range_0, intensity_0);

[global_OOI_list.Centers(1,:),global_OOI_list.Centers(2,:)] = ...
            transform_position(global_OOI_list.Centers(1,:)', ...
                               global_OOI_list.Centers(2,:)', ...
                               state(1,:));

set(handles.object_global_centers,'xdata',global_OOI_list.Centers(1,:),...
                                  'ydata',global_OOI_list.Centers(2,:));
                              
for i = 1:global_OOI_list.N
    set(handles.object_global_labels(i),...
        'position',[global_OOI_list.Centers(1,i)+0.2,global_OOI_list.Centers(2,i)],...
        'String',['#',num2str(i)]);
end

%Initialize
local_OOI_list = global_OOI_list;                             
                              
% Iterate through data set, using IMU times as clock
current_scan = 1;

for i = 2:length(IMU_times)-1
    % When a new laser scan arrives, update
    if (IMU_times(i) > laser_times(current_scan) && current_scan < length(laser_times))

        current_scan = current_scan + 1;
        [range_i, intensity_i] = ...
            extract_laser_data(laser_scans(:,current_scan));
        
        %disp(state(i,:))
        
        local_OOI_list = ExtractOOIs(range_i,intensity_i);
        
        [adjusted_centers_X,adjusted_centers_Y] = ...
        transform_position(local_OOI_list.Centers(1,:)', ...
                           local_OOI_list.Centers(2,:)', ...
                           state(i,:));
                       
         plot_scan(range_i, intensity_i, state(i,:), handles);
        set(handles.object_local_centers,...
        'xdata',adjusted_centers_X,...
        'ydata',adjusted_centers_Y);
        
        %Associate objects
        for j = 1:local_OOI_list.N
            x_dists = global_OOI_list.Centers(1,:) - adjusted_centers_X(j);
            y_dists = global_OOI_list.Centers(2,:) - adjusted_centers_Y(j);
            euclidian_dists = sqrt(x_dists.^2 + y_dists.^2);
            [mindist,mini] = min(euclidian_dists);
            if mindist < 0.4
                set(handles.object_local_labels(j),...
                    'position',[adjusted_centers_X(j)-1,adjusted_centers_Y(j)-0.5],...
                    'String',['Best match: #',num2str(mini),', Error: ', num2str(mindist), ' m']);
            else
                 set(handles.object_local_labels(j),...
                    'position',[0,0],...
                    'String','');
            end
            
        end
        
        for j = local_OOI_list.N+1:length(handles.object_local_labels)
              set(handles.object_local_labels(j),...
                    'position',[0,0],...
                    'String','');
        end
    
    end
        
 
    
    s = sprintf('Processed: IMU scan %d, Laser scan %d', i, current_scan);
    set(handles.object_title, 'string', s);
    
    set(handles.vehicle_trace,'xdata',state(1:i,1),'ydata',state(1:i,2));
    pause(0.0001);
end

end

function new_state = int_state(omega, speed, dt, old_state)
        new_state = zeros(1,3);
		new_state(1) = speed*cos(old_state(3))*dt + old_state(1); 
		new_state(2) = speed*sin(old_state(3))*dt + old_state(2); 
		new_state(3) = omega*dt + old_state(3); 
end

function [ranges, intensity] = extract_laser_data(scan)
         mask_low_13_bits = uint16(2^13-1);
         rangesA = bitand(scan,mask_low_13_bits);
         ranges = 0.01*double(rangesA);

         %Extract intensity data
         maskE000 = bitshift(uint16(7),13);
         intensity = bitand(scan, maskE000);
end

function plot_scan(ranges, intensity, state, handles)
        laser_offset = 0.46;
        angles = [0:360]'*0.5*pi/180;
        X = -cos(angles).*ranges;
        Y = sin(angles).*ranges + laser_offset;
        
        [X_t,Y_t] = transform_position(X,Y,state);
        
        ii_bright = find(intensity~=0);
        set(handles.object_data_points,'xdata',X_t,'ydata',Y_t);
        set(handles.object_bright_points,'xdata',X_t(ii_bright),...
            'ydata',Y_t(ii_bright));
        
end

function [transformed_X, transformed_Y] = transform_position(X, Y, state)
       
       transform_matrix = rotz(state(3)*180/pi - 90) + [[0 0 state(1)];...
                                         [0 0 state(2)];...
                                         [0 0 0]];
       homogenous_coords = [X'; Y'; ones(1,length(X))];
       transformed_coords = transform_matrix*homogenous_coords;
       transformed_X = transformed_coords(1,:);
       transformed_Y = transformed_coords(2,:);
   
end

