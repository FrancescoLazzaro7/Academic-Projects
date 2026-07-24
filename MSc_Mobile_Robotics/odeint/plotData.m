clear all
close all

% Import data from csv file
data = importdata('simulation_results.csv',',');
t   = data.data(:,1);
x1  = data.data(:,2);
x2  = data.data(:,3);
u   = data.data(:,4);

figure,plot(t,x1),grid,xlabel('Time [s]'),ylabel('Position [rad]'),xlim([0 10])
figure,plot(t,x2),grid,xlabel('Time [s]'),ylabel('Velocity [rad/s]'),xlim([0 10])
