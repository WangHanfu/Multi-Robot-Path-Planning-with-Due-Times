clear;
clc;
%% makespan
largeNumber = 10000000;
load('Results20x20x0-lateness.mat');
[valueMat,timeMat] = statistics(Results);
y = zeros(1,6);
for i=1:6
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

load('Results20x20x10-lateness.mat');
[valueMat,timeMat] = statistics(Results);
y = zeros(1,6);
for i=1:6
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

load('Results20x20x20-lateness.mat');
[valueMat,timeMat] = statistics(Results);
y = zeros(1,6);
for i=1:6
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
set(gca,'xtick',0:10:60);
x=10:10:60;
span = 6;
plot(x(1,1:span),y1(1,1:span),'-ro','MarkerFaceColor','red');
hold on;
span = 5;
plot(x(1,1:span),y2(1,1:span),'-gd','MarkerFaceColor','green');
hold on;
span = 5;
plot(x(1,1:span),y3(1,1:span),'-bs','MarkerFaceColor','blue');
grid on;
set(gca,'xtick',0:10:60);
legend('0','10%','20%','Location','northwest','FontSize',12);

figure(2);
set(gca,'xtick',0:10:60);
x=10:10:60;
span = 6;
semilogy(x(1,1:span),t1(1,1:span),'-ro','MarkerFaceColor','red');
%plot(x(1,1:span),t1(1,1:span),'-ro','MarkerFaceColor','red');
hold on;
span = 5;
semilogy(x(1,1:span),t2(1,1:span),'-gd','MarkerFaceColor','green');
%plot(x(1,1:span),t2(1,1:span),'-gd','MarkerFaceColor','green');
hold on;
span = 5;
semilogy(x(1,1:span),t3(1,1:span),'-bs','MarkerFaceColor','blue');
%plot(x(1,1:span),t3(1,1:span),'-bs','MarkerFaceColor','blue');
grid on;
set(gca,'xtick',0:10:60);
legend('0','10%','20%','Location','northwest','FontSize',12);