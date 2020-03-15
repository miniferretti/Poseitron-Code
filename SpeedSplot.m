filename = 'logFile.txt';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);

refR=A.data(:,1);
speedR=A.data(:,2);
refL=A.data(:,3);
speedL=A.data(:,4);
time= A.data(:,5);



close all;
figure;

subplot(2,1,1);
hold on;
plot(time',refR);
plot(time',speedR);
xlabel("Time [sec]");
ylabel("Speed [rad/sec]");
title("Right wheel control log");

subplot(2,1,2);
hold on;
plot(time',refL);
plot(time',speedL);
xlabel("Time [sec]");
ylabel("Speed [rad/sec]");
title("Left wheel control log");


