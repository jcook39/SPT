function latex_figures()

figure(1)
set(gcf,'pos',[10 10 1800 1200])
print -depsc Leader_Follower_2D_Plot_eps
copyfile('Leader_Follower_2D_Plot_eps.eps','S:/NISTcontrols/Josh_Cook/Journal of Terramechanics Documents')

figure(12)
set(gcf,'pos',[10 10 1800 1200])
print -depsc Leader_Follower_traj_eps
copyfile('Leader_Follower_traj_eps.eps','S:/NISTcontrols/Josh_Cook/Journal of Terramechanics Documents')


end

