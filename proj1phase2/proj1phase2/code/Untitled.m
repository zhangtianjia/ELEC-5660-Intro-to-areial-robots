i=1;
for j=0:0.1:24
    pose = trajectory_generator(j);
    x(i) = pose(2);
    i=i+1;
end
figure(1)
plot(0:0.1:24,x)
figure(2)
plot3(path(:,1),path(:,2),path(:,3))