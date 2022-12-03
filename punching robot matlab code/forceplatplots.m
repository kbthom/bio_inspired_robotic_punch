walking = readmatrix('samuel1.txt');
time = walking(18:1017,1);
ry = walking(18:1017,4);
plot(time,ry)
title 'Walking'
xlabel 'time'
ylabel 'reaction force'

running = readmatrix('samuel2.txt');
time2 = running(18:1017,1);
ry2 = running(18:1017,4);
plot(time2,ry2)
title 'Running'
xlabel 'time'
ylabel 'reaction force'

