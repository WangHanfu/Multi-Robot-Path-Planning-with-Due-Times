clear;
clc;
%% makespan
largeNumber = 10000000;
load('AMRPP-DT-Results20x20x0-up.mat');
[valueMat,timeMat] = statistics(Results);
y = zeros(1,9);
for i=1:9
    vec = valueMat(i,:);
    vec(vec==largeNumber)=[];
    if ~isempty(vec) && length(vec)>=15
        y(1,i) = mean(vec);
    elseif length(vec)<15
        disp("cannot solve 3/4 instances");
        y(1,i) = largeNumber;
    elseif isempty(vec)
        disp("empty value");
        y(1,i) = largeNumber;
    end
end
y1 = y;

timeMat = timeMat';
t1 = mean(timeMat);

load('AMRPP-DT-Results20x20x10-up.mat');
[valueMat,timeMat] = statistics(Results);
y = zeros(1,9);
for i=1:9
    vec = valueMat(i,:);
    vec(vec==largeNumber)=[];
    if ~isempty(vec) && length(vec)>=15
        y(1,i) = mean(vec);
    elseif length(vec)<15
        disp("cannot solve 3/4 instances");
        y(1,i) = largeNumber;
    elseif isempty(vec)
        disp("empty value");
        y(1,i) = largeNumber;
    end
end
y2 = y;

timeMat = timeMat';
t2 = mean(timeMat);

load('AMRPP-DT-Results20x20x20-up.mat');
[valueMat,timeMat] = statistics(Results);
y = zeros(1,9);
for i=1:9
    vec = valueMat(i,:);
    vec(vec==largeNumber)=[];
    if ~isempty(vec) && length(vec)>=15
        y(1,i) = mean(vec);
    elseif length(vec)<15
        disp("cannot solve 3/4 instances");
        y(1,i) = largeNumber;
    elseif isempty(vec)
        disp("empty value");
        y(1,i) = largeNumber;
    end
end
y3 = y;

timeMat = timeMat';
t3 = mean(timeMat);

figure(1);
set(gca,'xtick',0:10:90);
x=10:10:90;
plot(x,y1,'-ro','MarkerFaceColor','red');
%errorbar(x,y1,z,'-ro','MarkerFaceColor','red');
hold on;
plot(x,y2,'-gd','MarkerFaceColor','green');
%errorbar(x,y2,z,'-gd','MarkerFaceColor','green');
hold on;
plot(x,y3,'-bs','MarkerFaceColor','blue');
%errorbar(x,y3,z,'-bs','MarkerFaceColor','blue');
grid on;
legend('0','10%','20%','Location','northeast','FontSize',12);

figure(2);
set(gca,'xtick',0:10:90);
x=10:10:90;
plot(x,t1,'-ro','MarkerFaceColor','red');
%errorbar(x,y1,z,'-ro','MarkerFaceColor','red');
hold on;
plot(x,t2,'-gd','MarkerFaceColor','green');
%errorbar(x,y2,z,'-gd','MarkerFaceColor','green');
hold on;
plot(x,t3,'-bs','MarkerFaceColor','blue');
%errorbar(x,y3,z,'-bs','MarkerFaceColor','blue');
grid on;
legend('0','10%','20%','Location','northwest','FontSize',12);