filename = 'logFile.txt';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);

refR=(A.data(:,1));
speedR=(A.data(:,2));

time = 0:1:length(refR)-1;

figure;
hold on;
plot(time',refR);
plot(time',speedR);
