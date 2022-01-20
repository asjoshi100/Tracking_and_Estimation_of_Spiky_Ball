%% Course Project - Advanced Control System (ME 8281) 
%% Amey Joshi
% Mechanical - MSME - Spring 2019
%% Ask Which Control to use. 
clear;
clc;
option=menu('Control System to Use','1.Pole place approach to steady state and observer','2. Least square estimate','3. Linear quadratic control','4. Kalman filter','Exit');
%% Defining initial conditions and A, B, C matrices
m = 1;
b = 2;
k = 1;
A = [0 0 0 0 0 0;
    0 0 0 0 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    (k/m) 0 -(k/m) 0 -(b/m) 0;
    0 (k/m) 0 -(k/m) 0 -(b/m)];
B = [1 0;
    0 1;
    0 0;
    0 0;
    0 0;
    0 0];
C = [1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0];
K = [0 0 1 0 0 0;
    0 0 0 1 0 0];
delta_t = 0.01;
simulation = {'Project_Papi_Rubber_Game_SSObs_Sim.slx','Project_Papi_Rubber_Game_LSE_Sim.slx','Project_Papi_Rubber_Game_LQR_Sim.slx','Project_Papi_Rubber_Game_KF_Sim.slx'};
title_str = {'State Feedback and Observer Gain','Least Square Estimation','Linear Quadratic Control','Kalman Filter'};
%% Defining Enemy trajectory
dia1 = 7.5;
dia2 = 5;
dia3 = 5;
enemy_x = [10 -10 -10 10];
enemy_y = [20 20 -20 -20];
slop = [2 -1 3 -2];

num = 1;
%% If it is steady state and observer
if option == 1
A_new_x = [A(1,1) A(1,3) A(1,5) 0; A(3,1) A(3,3) A(3,5) 0; A(5,1) A(5,3) A(5,5) 0; -K(1,1) -K(1,3) -K(1,5) 0];
B_new_x = [B(1,1); B(3,1); B(5,1); 0];
C_new_x = [K(1,1) K(1,3) K(1,5) 0];

A_new_y = [A(2,2) A(2,4) A(2,6) 0; A(4,2) A(4,4) A(4,6) 0; A(6,2) A(6,2) A(6,6) 0; -K(2,2) -K(2,4) -K(2,6) 0];
B_new_y = [B(2,2); B(4,2); B(6,2); 0];
C_new_y = [K(2,2) K(2,4) K(2,6) 0];

OS_x = 10;
Ts_x = 0.1;
z_x = 0.5;
Wn_x = 5.8335/Ts_x;
den_x = [1 2*z_x*Wn_x Wn_x^2];
desiredPoles_x = roots(den_x);      
K_x = acker(A_new_x, B_new_x, [-5; -5; -10; -10]);

OS_y = 5;
Ts_y = 0.1;
z_y = 1;
Wn_y = 5.8335/Ts_y;
den_y = [1 2*z_y*Wn_y Wn_y^2];
desiredPoles_y = roots(den_y);
K_y = acker(A_new_y, B_new_y, [-5; -5; -10; -10]);

K_ss = [K_x(1,1) 0 K_x(1,2) 0 K_x(1,3) 0; 0 K_y(1,1) 0 K_y(1,2) 0 K_y(1,3)];

K_int = [K_x(1,4) 0; 0 K_y(1,4)];

A_x = [A(1,1) A(1,3) A(1,5); A(3,1) A(3,3) A(3,5); A(5,1) A(5,3) A(5,5)];
A_y = [A(2,2) A(2,4) A(2,6); A(4,2) A(4,4) A(4,6); A(6,2) A(6,2) A(6,6)];
C_x = [K(1,1) K(1,3) K(1,5)];
C_y = [K(2,2) K(2,4) K(2,6)];
L_x = acker(A_x.', C_x.', [-5*5; -5*5; -10]);
L_y = acker(A_y.', C_y.', [-5*5; -5*5; -10]);
L_obs = [L_x(1,1) 0 0 0; 0 L_y(1,1) 0 0; 0 0 L_x(1,2) 0; 0 0 0 L_y(1,2); zeros(2,4)];
end
%% If it is least square estimation
if option == 2
    
end
%% If it is LQR control
if option == 3
Q = eye(6) * 1;
R = 0.1;
K_lqr = lqr(A,B,Q,R);
K_lqr_sim = [K_lqr(1,3) 0; 0 K_lqr(2,4)];
end
%% If it is Kalman Filter
if option == 4
Q = 0.1 * eye(2);
R = 300;
end
%% Doing animation
if option == 1 || option == 2 || option == 3 || option == 4
for m = 1:1:4
    if m == 1
        init_cond = zeros(6,1);
    else
        init_cond = [out.x_plant.Data(j,1); out.x_plant.Data(j,2); out.x_plant.Data(j,3); out.x_plant.Data(j,4); out.x_plant.Data(j,5); out.x_plant.Data(j,6)];
    end
    
    if m == 1
        ene = [enemy_x(1,1) enemy_y(1,1)];
    elseif m == 2
        ene = [ene2(1,1)+(dia3/2) ene2(1,2)+(dia3/2)];
    elseif m == 3
        ene = [ene3(1,1)+(dia3/2) ene3(1,2)+(dia3/2)];
    else 
        ene = [ene4(1,1)+(dia3/2) ene4(1,2)+(dia3/2)];
    end
    
    out = sim(simulation{option});
    num_check = num;
    for j=1:1:5000
        if(abs(out.x_plant.Data(j,3) - ((slop(1,m)*num_check*delta_t) + enemy_x(1,m))) < 2.5 && abs(out.x_plant.Data(j,4) - ((slop(1,m)*num_check*delta_t) + enemy_y(1,m))) < 2.5)
            break;
        end
        num_check = num_check + 1;
    end
    
    for i = 1:1:j
        figure(1)
        clf('reset')
        pos1 = [out.x_plant.Data(i,1)-dia1/2 out.x_plant.Data(i,2)-dia1/2 dia1 dia1];
        rectangle('Position',pos1,'Curvature',[1 1], 'FaceColor','r')
        pos2 = [out.x_plant.Data(i,3)-dia2/2 out.x_plant.Data(i,4)-dia2/2 dia2 dia2];
        rectangle('Position',pos2,'Curvature',[1 1], 'FaceColor','y')
        line([out.x_plant.Data(i,1) out.x_plant.Data(i,3)],[out.x_plant.Data(i,2) out.x_plant.Data(i,4)], 'linewidth',2)
        ene1 = [(slop(1,1)*num*delta_t + enemy_x(1,1))-(dia3/2) (slop(1,1)*num*delta_t + enemy_y(1,1))-(dia3/2) dia3 dia3];
        if m == 1
            ene1 = [(slop(1,1)*num*delta_t + enemy_x(1,1))-(dia3/2) (slop(1,1)*num*delta_t + enemy_y(1,1))-(dia3/2) dia3 dia3];
            rectangle('Position',ene1,'Curvature',[1 1], 'FaceColor','b')
        end
        if m == 1 || m == 2
            ene2 = [(slop(1,2)*num*delta_t + enemy_x(1,2))-(dia3/2) (slop(1,2)*num*delta_t + enemy_y(1,2))-(dia3/2) dia3 dia3];
            rectangle('Position',ene2,'Curvature',[1 1], 'FaceColor','b')
        end
        if m == 1 || m == 2 || m == 3
            ene3 = [(slop(1,3)*num*delta_t + enemy_x(1,3))-(dia3/2) (slop(1,3)*num*delta_t + enemy_y(1,3))-(dia3/2) dia3 dia3];
            rectangle('Position',ene3,'Curvature',[1 1], 'FaceColor','b')
        end
        if m ==1 || m == 2 || m == 3 || m == 4
            ene4 = [(slop(1,4)*num*delta_t + enemy_x(1,4))-(dia3/2) (slop(1,4)*num*delta_t + enemy_y(1,4))-(dia3/2) dia3 dia3];
            rectangle('Position',ene4,'Curvature',[1 1], 'FaceColor','b')
        end
        axis equal
        title(title_str{option});
        xlim([-100 100]);
        ylim([-100 100]);
        pause(0.0002)
        num = num + 1;
    end
    figure(m+15)
    plot(out.y);
    title(['Variation of position for ', title_str{option}]);
    legend('Papi x co-ordinate','Papi y co-ordinate','Spiky ball x co-ordinate','Spiky ball y co-ordinate');
end
end