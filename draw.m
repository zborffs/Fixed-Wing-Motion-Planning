function draw(aircraft, env)

for i = 1:length(env.obstacles)
    plot(env.obstacles{i}); hold on;
end

plot(aircraft.get_shape()); hold off;

end

