udp_receiver = dsp.UDPReceiver('RemoteIPAddress','127.0.0.1',...
                               'LocalIPPort',1115,...
                               'MaximumMessageLength',1024,...
                               'MessageDataType','uint8');
                           
while(1)
    buf = step(udp_receiver);
    if length(buf) == 16
        fprintf('IMU Packet\n');
        time = typecast(buf(9:12), 'uint32');
        velocity = typecast(buf(13:14), 'int16')
        omega = typecast(buf(15:16), 'int16')
    elseif length(buf) == 744
        fprintf('LIDAR Packet\n');
        time = typecast(buf(9:12), 'uint32');
        range = buf(15:373);
    end
            
     
end
