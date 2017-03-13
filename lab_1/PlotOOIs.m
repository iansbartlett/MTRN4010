function [] = PlotOOIs(ooi, handles)
% A function to display a list of objects of interest
% Ian Bartlett

if ooi.N < 1, return; end;

i_bright = (ooi.Color == 1); 

%handles.c1 = viscircles(handles.object_plot,ooi.Centers(:,i_bright)',ooi.Diameters(i_bright)/2,'Color','g')
%handles.c2 = viscircles(handles.object_plot,ooi.Centers(:,~i_bright)',ooi.Diameters(~i_bright)/2,'Color','b')

bright_circles = [];
dull_circles = [];

for i=find(i_bright)
	bright_circles = [bright_circles; circlePoints(ooi.Centers(:,i),ooi.Diameters(i)/2)];
end

for i=find(~i_bright)
	dull_circles = [dull_circles; circlePoints(ooi.Centers(:,i),ooi.Diameters(i)/2)];
end


set(handles.ooi_centers, 'xdata',ooi.Centers(1,i_bright),'ydata',ooi.Centers(2,i_bright));

if(length(bright_circles)>0)
   set(handles.ooi_circles, 'xdata',bright_circles(:,1),'ydata',bright_circles(:,2));
end

if(length(dull_circles)>0)
   set(handles.other_circles, 'xdata',dull_circles(:,1),'ydata',dull_circles(:,2));
end

s = sprintf('Processed: Tracking %d bright objects out of %d total objects', length(find(i_bright)), ooi.N)

set(handles.object_plot_title,'string',s)

end
