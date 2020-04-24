world = vrworld('DroneLab.wrl'); 

open(world);

fig = view(world, '-internal');
vrdrawnow;

get(world);

nodes(world);

drone = vrnode(world, 'crazyflie');

%%
drone.translation=[0 0 0];
drone.rotation=[0 0 0 0];
counter=0;
for i=1:10:length(vQ)   
   drone.translation = [Vicon(i,2) Vicon(i,3) Vicon(i,4)];
   drone.rotation = [vQ(i,2), vQ(i,3), vQ(i,4), -2*acos(vQ(i,1))];
   vrdrawnow;
   pause(0.05);
end