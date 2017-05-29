%Master run file for robot localization lab (MTRN4010) 
%Part 3 of Project 1 - Full Kalman Filter
%Ian Bartlett

function [] = lab_4_2()
% Kalman filter parameters

Xe = [0 0 pi/2 0] ;        
P = zeros(4,4);
P(4,4) = (5*pi/180)^2;

Xe_History = zeros(4)

stdDevGyro = 5*pi/180 ;        
stdDevSpeed = 0.15 ; 
sdev_rangeMeasurement = 0.05;
sdev_angleMeasurement = 0.5*pi/180; 

Q_1 = diag( [ (0.01)^2 ,(0.01)^2 , (0.05*pi/180)^2, 0]) ;
Q_u = diag([0 stdDevSpeed]);

times = [];

% Begin configuration initialization

handles = initialize_plots_live;

%Extract linear trend to estimate IMU data

% Data fusion
                
                              
% Iterate through data set, using IMU times as clock
current_scan = 1;
global_OOI_list = [];

udp_receiver = dsp.UDPReceiver('RemoteIPAddress','127.0.0.1',...
                               'LocalIPPort',1115,...
                               'MaximumMessageLength',1024,...
                               'MessageDataType','uint8');
                           
step(udp_receiver);

time = 0;
          
velocity = 0;
imu_gyro = 0;
num_measurements=0;

while (1)
    
    got_frame = 0;
    prev_time = time;
    buf_size = -1;
    %Fetch a UDP packet
    buf = step(udp_receiver);
    if(~isempty(buf))
        buf_size = typecast(buf(5:6),'uint16');
    else
        buf_size = -1;
    end
    
    if buf_size == 8
        fprintf('IMU Packet\n');
        time = double(typecast(buf(9:12), 'uint32'))*1e-4;
        velocity = double(typecast(buf(13:14), 'int16'))*1e-3;
        imu_gyro = -1*double(typecast(buf(15:16), 'int16'))*0.02*pi/180;
         
        MeasuredRanges = [];
        MeasuredBearings = [];
        ObservedLandmarks = [];
        num_measurements = 0;
        if(isempty(times))
            times = time;
        end
        got_frame = 1;
        
    elseif buf_size == 736
        got_frame = 1;
        fprintf('LIDAR Packet\n');
        time = double(typecast(buf(9:12), 'uint32'))*1e-4;
        current_scan = typecast(buf(15:736),'uint16');
        [range_i, intensity_i] = ...
            extract_laser_data(current_scan);
        
        if (isempty(global_OOI_list))
                    % Initialize and plot starting positions of OOIs
                    % Process first scan
                    range_0 = range_i;
                    intensity_0 = intensity_i;
                    plot_scan(range_0, intensity_0, Xe, handles);
                    global_OOI_list = ExtractOOIs(range_0, intensity_0);

                    [global_OOI_list.Centers(1,:),global_OOI_list.Centers(2,:)] = ...
                                transform_position(global_OOI_list.Centers(1,:)', ...
                                                   global_OOI_list.Centers(2,:)', ...
                                                   Xe);

                    set(handles.object_global_centers,'xdata',global_OOI_list.Centers(1,:),...
                                                      'ydata',global_OOI_list.Centers(2,:));

                    for i = 1:global_OOI_list.N
                        set(handles.object_global_labels(i),...
                            'position',[global_OOI_list.Centers(1,i)+0.2,global_OOI_list.Centers(2,i)],...
                            'String',['#',num2str(i)]);
                    end

                    %Initialize
                    local_OOI_list = global_OOI_list;             
        else
            local_OOI_list = ExtractOOIs(range_i,intensity_i);

            [adjusted_centers_X,adjusted_centers_Y] = ...
                transform_position(local_OOI_list.Centers(1,:)', ...
                local_OOI_list.Centers(2,:)', ...
                Xe);

            plot_scan(range_i, intensity_i, Xe, handles);

            set(handles.object_local_centers,...
                'xdata',adjusted_centers_X,...
                'ydata',adjusted_centers_Y);

            %Associate objects

            %Shouldn't just be inventing this attribute here
            local_OOI_list.global_ID = zeros(local_OOI_list.N, 1);

            for j = 1:local_OOI_list.N
                x_dists = global_OOI_list.Centers(1,:) - adjusted_centers_X(j);
                y_dists = global_OOI_list.Centers(2,:) - adjusted_centers_Y(j);
                euclidian_dists = sqrt(x_dists.^2 + y_dists.^2);
                [mindist,mini] = min(euclidian_dists);
                if mindist < 0.4
                    set(handles.object_local_labels(j),...
                        'position',[adjusted_centers_X(j)-1,adjusted_centers_Y(j)-0.5],...
                        'String',['Best match: #',num2str(mini),', Error: ', num2str(mindist), ' m']);
                    local_OOI_list.global_ID(j) = mini;

                    MeasuredRanges = [MeasuredRanges, sqrt(local_OOI_list.Centers(1,j)^2 + ...
                        local_OOI_list.Centers(2,j)^2)];
                    MeasuredBearings = [MeasuredBearings, atan2(local_OOI_list.Centers(2,j),...
                        local_OOI_list.Centers(1,j))];

                    ObservedLandmarks = [ObservedLandmarks, local_OOI_list.global_ID(j)];
                    num_measurements = num_measurements + 1;

                else
                    set(handles.object_local_labels(j),...
                        'position',[0,0],...
                        'String','');
                    local_OOI_list.global_ID(j) = -1;
                end
                            
            end
        end
        
        for j = local_OOI_list.N+1:length(handles.object_local_labels)
            set(handles.object_local_labels(j),...
                'position',[0,0],...
                'String','');
        end
    elseif buf_size > 0
        fprintf('Bad buffer size, %d bytes\n', buf_size);
        continue;
    else
        continue;
    end

    if(~got_frame)
       disp(skipped)
       continue; 
    end
    
    
    Dt = double(time)-double(times(end))
    times = [times, double(time)];

    J = [[1,0,-Dt*velocity*sin(Xe(3)), 0];...
         [0,1,Dt*velocity*cos(Xe(3)),0] ;...
	 [ 0,0,1,-1*Dt ];...
	 [0,0,0,1] ] ; 

    %disp(velocity)
    F_u = Dt*[[velocity*cos(Xe(3)) 0]; [velocity*sin(Xe(3)) 0]; [0 1]; [0, 0];];

    % MODIFIED: added J*Q_u*J' term to include input uncertainty
    %Q(4,4) = 0;%(Dt*pi/(180*10*60))^2;
    Q = (Q_1 + F_u*Q_u*F_u');
    P = J*P*J'+ Q; 
    % -----------------------------------------------
    % and the predicted expected value. 
    Xe = int_state(imu_gyro, velocity, Dt, Xe)';  

    %If any landmarks have been detected and assocated, use them to compute new 
    %range/bearing measurements, and perform an EKF update. 
    for j = 1:num_measurements
	    %disp(local_OOI_list.global_ID(j))
	    landmark_x = global_OOI_list.Centers(1,ObservedLandmarks(j));
	    landmark_y = global_OOI_list.Centers(2,ObservedLandmarks(j));

	    eDX = (landmark_x-Xe(1)) ;      % (xu-x)
	    eDY = (landmark_y-Xe(2)) ;      % (yu-y)
	    eDD = sqrt( eDX*eDX + eDY*eDY ) ; 
	
	    % New 2D measurement matrix:
	    H = [[  -eDX/eDD , -eDY/eDD , 0 , 0]; 
		 [eDY/(eDX^2 + eDY^2), -eDX/(eDX^2 + eDY^2), -1, 0]];

	    ExpectedRange = eDD ;  
	    ExpectedBearing = wrapToPi(atan2(eDY,eDX) - Xe(3) + pi/2);

	    z = [MeasuredRanges(j) - ExpectedRange; MeasuredBearings(j) - ExpectedBearing]; 

	    disp('Measured bearings')
	    disp(MeasuredBearings)

	    % ------ covariance of the noise/uncetainty in the measurements
	    R = diag([sdev_rangeMeasurement*sdev_rangeMeasurement,...
		      sdev_angleMeasurement*sdev_angleMeasurement]);

	    S = R + H*P*H' ;
	    iS=inv(S);
	    K = P*H'*iS            % Kalman gain

	    % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)
	    Xe = Xe+K*z            % update the  expected value
	    P = P-P*H'*iS*H*P ;     % update the Covariance
    end

    Xe_History = [Xe_History, Xe];
    
    %s = sprintf('Processed: IMU scan %d, Laser scan %d', i, current_scan);
    %set(handles.object_title, 'string', s);
    
    set(handles.kalman_trace,'xdata',Xe_History(1,:),'ydata',Xe_History(2,:));
    set(handles.bias_kalman,'xdata',times-times(1),...
        'ydata', Xe_History(4,1:length(times)))
    %pause(0.0001);
    drawnow;
end

end

function new_state = int_state(omega, speed, dt, old_state)
        new_state = zeros(1,3);
		new_state(1) = speed*cos(old_state(3))*dt + old_state(1); 
		new_state(2) = speed*sin(old_state(3))*dt + old_state(2); 
		new_state(3) = (omega - old_state(4))*dt + old_state(3); 
		new_state(4) = old_state(4);
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

