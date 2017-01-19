function latex_figures()

figure(1)
set(gcf,'pos',[10 10 1800 1200])
print -depsc MB_Tractor_2D_Plot.eps
%copyfile('MB_Tractor_2D_Plot.eps','/Users/joshuacook/Desktop/Final_Tractor_Models_Code/')

figure(12)
set(gcf,'pos',[10 10 1800 1200])
print -depsc MB_Tractor_traj.eps
%copyfile('MB_Tractor_traj.eps','/Users/joshuacook/Desktop/Final_Tractor_Models_Code/')


end

