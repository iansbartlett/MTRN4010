
% Overall code for Lab 1
% Ian Bartlett
% Code adapted from ExampleProcessLaserData.m and ExampleUseLaserData.,
% by Dr. Jose Guivant, provided as part of MTRN4010

function [] = Lab1(file)

	%if ~exist('file')
	%       	printf('Error- file not found!\n'); 
	%	return;
	%end;

	load(file)

	% Configure plots

	figure(1);
	clf();

	hold on;

	%gui_handles.data_plot = subplot(1,2,1);
	hold on;
	gui_handles.data_axes = plot(0,0,'b. ');
	gui_handles.bright_axes = plot(0,0,'r* ');
	gui_handles.ooi_centers = plot(0,0,'m* ');
	gui_handles.ooi_circles = plot(0,0,'m. ');
	gui_handles.other_circles = plot(0,0,'k. ');
	axis([-10,10,0,10]);
	xlabel('X (m)');
	ylabel('Y (m)');
	gui_handles.data_plot_title = title('');
	zoom on;
	grid on;

	%gui_handles.object_plot = subplot(1,2,2)
	%hold on;
	%gui_handles.ooi_centers = plot(0,0,'g* ');
	%gui_handles.ooi_circles = plot(0,0,'g. ');
	%gui_handles.other_circles = plot(0,0,'b. ');
	%gui_handles.c1 = viscircles([0,0],0)
	%gui_handles.c2 = viscircles([0,0],0)
	%axis([-10,10,0,10]);
	%xlabel('X (m)');
	%ylabel('Y (m)');
	%gui_handles.object_plot_title = title('');
	%zoom on;
	%grid on;

	N = dataL.N;
	skip = 10;

	for i=1:skip:N
		t = double(dataL.times(i) - dataL.times(1))/10000;
		scan_i = dataL.Scans(:,i);
		fprintf('attempting scan %d\n', i)
		process_scan(scan_i,t,gui_handles,i);
		pause(0.01);
	end;

	fprintf('\nProcessing complete.\n');

end

function process_scan(scan,t,mh,i)
   tic
   angles = [0:360]'*0.5*pi/180;

   %Obtain range data
   mask_low_13_bits = uint16(2^13-1);
   rangesA = bitand(scan,mask_low_13_bits);
   ranges = 0.01*double(rangesA);

   %Extract intensity data
   maskE000 = bitshift(uint16(7),13);
   intensity = bitand(scan, maskE000);

   %Calculate Cartesian coords
   X = cos(angles).*ranges;
   Y = sin(angles).*ranges;
   
   oois = ExtractOOIs(ranges, intensity);
   toc 
   count = PlotOOIs(oois, mh);

   %Update the plots
   ii_bright = find(intensity~=0);
   set(mh.data_axes,'xdata',X,'ydata',Y);
   set(mh.bright_axes,'xdata',X(ii_bright),'ydata',Y(ii_bright));

   %Update plot titles
   s = sprintf('Raw Laser scan # %d at time %.3f secs, tracking %d bright OOIs', i, t, count);
   set(mh.data_plot_title,'string',s);

end
