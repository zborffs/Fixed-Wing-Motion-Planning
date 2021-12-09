function draw(aircraft, env)

for i = 1:length(env.obstacles)
    plot(env.obstacles{i}, 'FaceColor', 'red'); hold on;
end

plot(aircraft.get_shape(), 'FaceColor', 'blue'); hold off;

end

