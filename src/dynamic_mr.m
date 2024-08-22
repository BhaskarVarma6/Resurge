% dynamic model of mobile robot

clear all; clc; close all;

%% Simulation parameters (Euler's method)
dt = 0.1; % stepsize
ts = 20; % total simulation time
t = 0:dt:ts; %span 0, 0.1, 0.2,....,9.9,10.

%% Initial conditions

eta0 = [0;0;pi/4]; %initial posn and orientation
zeta0 = [1 ; 0; -5]; % initial vector of input command (body fixed velocities)

eta(:,1) = eta0;
zeta(:,1) = zeta0;

% vehicle parameters

m = 10; % mass of the vehicle
Iz = 0.1; % inertia of vehicle

xbc = 0; 
ybc = 0; % cordinates of com 

% state propogation

for i= 1:length(t)
    u = zeta(1,i);
    v = zeta(2,i);
    r = zeta(3,i);

    % inertia matrix D
    D = [m,0, -ybc*m;
        0, m, xbc*m;
        -ybc*m, xbc*m, Iz+m*(xbc^2 + ybc^2);];

    n_v = [-m*r*(v + xbc*r);
        m*r*(u - ybc*r);
        m*r*(xbc*u + ybc*v);];
    % Input vector
    tau(:,i) = [0; 0; 0];

    % Jacobian matrix
    psi = eta(3,i);
     J_eta = [cos(psi), -sin(psi), 0;
        sin(psi), cos(psi), 0;
        0,0,1];

    zeta_dot(:,i) = inv(D)*(tau(:,i)-n_v);
    zeta(:,i+1) = zeta(:,i) + dt * zeta_dot(:,i); % velocity update

    eta(:,i+1) = eta(:,i) + dt * (J_eta*(zeta(:,i) + dt *zeta_dot(:,i)) ); %state update
end

 %animation
w = 0.4;
l =0.6;
box_v = [-l/2,l/2,l/2,-l/2,-l/2;
    -w/2,-w/2,w/2,w/2,-w/2];
figure
for i = 1:length(t)    
    psi = eta(3,i);
    R_psi = [cos(psi), -sin(psi);
        sin(psi), cos(psi)]; % rotation matrix
    veh_ani = R_psi * box_v;
   
    fill(veh_ani(1,:)+eta(1,i),veh_ani(2,:)+eta(2,i),'g');
    hold on, grid on
    %  l_lim = min(min(eta(1:2,:))) - 0.5;
    %  u_lim = max(max(eta(1:2,:))) - 0.5;
    % axis([l_lim u_lim l_lim u_lim ])
     axis([0 20 0 20])
     axis square
    
    
    plot(eta(1,1:i), eta(2,1:i), 'b-');
    set(gca, 'fontsize', 24)
    xlabel('x,[m]');
    ylabel('y,[m]');
    % llim = min(min(x),min(y))-0.5;
    % ulim = max(max(x),max(y))-0.5;
    pause(0.1)
    hold off
end


