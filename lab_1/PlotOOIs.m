function [] = PlotOOIs(OOI_list)
% A function to display a list of objects of interest
% Ian Bartlett

if OOI_list.N < 1, return; end;

% Fix this method of handling figures later, it's 
% still hacky
figure(2); clf; hold on;
i_bright = (OOI_list.Color == 1) 

viscircles(OOI_list.Centers(:,i_bright)',OOI_list.Diameters(i_bright)/2,'Color','r')
viscircles(OOI_list.Centers(:,~i_bright)',OOI_list.Diameters(~i_bright)/2,'Color','b')

axis([-10,10,0,20]);

end
