function plt = plot_rays(x,y,z,tf)

in = find(tf);
out = find(not(tf));
plt = scatter3(x(out), y(out), z(out), '.'); hold on; scatter3(x(in), y(in), z(in), '.'); hold off

end

