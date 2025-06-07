close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Befor running this simulation make sure you constructed the gaintable
%first with gaintable_6DOF_sim.m file

%This simulation contains:
%(1) external wind field 
%(2) measurement noise and bias
%(3) nozzle dynamics
%(4) an offset nozzle of 5cm displacement in y direction
%(5) an oscillation of phi angle that might induce by RCS control

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Constants and Initial Conditions
dt = 0.01;              % Time step (s)
sim_time = 30;          % Total simulation time (s)
g = 9.81;               % Gravity (m/s²)
Isp = 130;              % Specific impulse
m_init = 150;           % Initial mass (kg)
Ixx_init = 3;           % Initial moment of inertia about X (kg·m²)
Iyy_init = 51.4950;     % Initial moment of inertia about Y (kg·m²)
Izz_init = 51.4950;     % Initial moment of inertia about Z (kg·m²)
dcn = 0.67;             % Some geometry constant
height  = 2;
radius = 0.2;
Kr = 1/12 * (3 * radius^2 + height^2); 

mass_values = linspace(50, 200, 50);
thrust_values = linspace(0.1, 2500, 50); 
mass_values(1) = 0;
thrust_values(1) = 0;

% Initial state for the rocket:
% [x, x_d, y, y_d, z, z_d, theta, theta_d, psi, psi_d]
state = zeros(10,1);  % initial rocket state (all zeros)

m = m_init;           % mass
Ixx = Ixx_init;
Iyy = Iyy_init;
Izz = Izz_init;
m_d = 0;
Ixx_dot = 0;
Iyy_dot = 0;
Izz_dot = 0;
T = 0;

%Initial state for the estimator and disturbance
x_a = zeros(13,1);


% Reference state (from file or other source)
reference = readtable('rocket_trajectory.csv');
reference_x = reference.x;
reference_xdot = reference.x_dot;
reference_state = zeros(10,1);  % Example reference state

% --- Nozzle Dynamics Initialization ---
% Represent nozzle orientation as a unit vector.
% We choose an initial nozzle orientation corresponding to zero pitch:
%   theta_noz = 0, psi_noz = 0  => r_noz = [cos(0); sin(0)*cos(0); sin(0)*sin(0)] = [1;0;0]
r_noz = [1; 0; 0];  
v_noz = [0; 0; 0];  % initial nozzle angular velocity (tangent to the sphere)
cmd_max = 5/100; %maiximum input of the nozzle dynamics (the leading number*0.21 ~ the maximum rad/s)

% Parameters for nozzle dynamics (spring–damper)
wn_noz   = 4*2*pi;   % natural frequency (rad/s)
zeta_noz = 1;  % damping ratio

% For plotting nozzle dynamics:
theta_noz_data = zeros(1, sim_time/dt);
psi_noz_data   = zeros(1, sim_time/dt);

%% Preallocation for Data Storage (rocket trajectory, forces, mass, etc.)
num_steps = sim_time / dt;
trajectory = zeros(10, num_steps);
control_forces_data = zeros(3, num_steps);
total_forces = zeros(1, num_steps);
ma = zeros(1, num_steps);
cmd = zeros(2,num_steps);
% (The original TVC angles are now used as the command signal; the actual nozzle angles come from r_noz)

%% Simulation Loop
for t_step = 1:num_steps
    time_now = (t_step-1)*dt;
    
    % === Extract Rocket State Variables ===
    x = state(1);  x_d = state(2);
    y = state(3);  y_d = state(4);
    z = state(5);  z_d = state(6);
    theta = state(7); theta_d = state(8);
    psi = state(9); psi_d = state(10);

    % (phi is an auxiliary angle for body rotation, here given as a sinusoid)
    phi = 0.25*sin(time_now);  
    phi_d = 0.25*cos(time_now);
    
    % Update reference state from file (if available)
    reference_state(1) = reference_x(t_step);
    reference_state(2) = reference_xdot(t_step);

    A_c = [ 0,   1,                  0,      0,                   0,     0,                   0,      0,                 0,      0;
        0,   T/(Isp*g*m),        0,      0,                   0,     0,                   0,      0,                 0,      0;
        0,   0,                  0,      1,                   0,     0,                   0,      0,                 0,      0;
        0,   0,                  0,      T/(Isp*g*m),         0,     0,                   0,      0,                 g,      0;
        0,   0,                  0,      0,                   0,     1,                   0,      0,                 0,      0;
        0,   0,                  0,      0,                   0,     T/(Isp*g*m),         -g,     0,                 0,      0;
        0,   0,                  0,      0,                   0,     0,                   0,      1,                 0,      0;
        0,   0,                  0,      0,                   0,     0,                   0,      T/(Isp*g*m),       0,      0;
        0,   0,                  0,      0,                   0,     0,                   0,      0,                 0,      1;
        0,   0,                  0,      0,                   0,     0,                   0,      0,                 0,      T/(Isp*g*m)
     ];

    % Define symbolic B_c matrix
    B_c = [ 0,       0,            0;
            1/m,     0,            0;
            0,       0,            0;
            0,       1/m,          0;
            0,       0,            0;
            0,       0,            1/m;
            0,       0,            0;
            0,       0,            dcn/(Kr*m);
            0,       0,            0;
            0,       -dcn/(Kr*m),  0
          ];

    C_c = eye(10);
    D_c = 0;
    A_d = eye(10) + A_c*dt;
    B_d = B_c*dt;
    C_d = C_c;
    D_d = D_c;

    %% --- Interpolation to Get Observer gain ---
    L = zeros (10, 10);
    L(1, 1) = interp2(mass_values, thrust_values, x_table_x_est, m, T, "linear");
    L(1, 2) = interp2(mass_values, thrust_values, x_dot_table_x_est, m, T, "linear");
    L(1, 3) = interp2(mass_values, thrust_values, y_table_x_est, m, T, "linear");
    L(1, 4) = interp2(mass_values, thrust_values, y_dot_table_x_est, m, T, "linear");
    L(1, 5) = interp2(mass_values, thrust_values, z_table_x_est, m, T, "linear");
    L(1, 6) = interp2(mass_values, thrust_values, z_dot_table_x_est, m, T, "linear");
    L(1, 7) = interp2(mass_values, thrust_values, theta_table_x_est, m, T, "linear");
    L(1, 8) = interp2(mass_values, thrust_values, theta_dot_table_x_est, m, T, "linear");
    L(1, 9) = interp2(mass_values, thrust_values, psi_table_x_est, m, T, "linear");
    L(1, 10) = interp2(mass_values, thrust_values, psi_dot_table_x_est, m, T, "linear");
    
    L(2, 1)  = interp2(mass_values, thrust_values, x_table_x_dot_est, m, T, "linear");
    L(2, 2)  = interp2(mass_values, thrust_values, x_dot_table_x_dot_est, m, T, "linear");
    L(2, 3)  = interp2(mass_values, thrust_values, y_table_x_dot_est, m, T, "linear");
    L(2, 4)  = interp2(mass_values, thrust_values, y_dot_table_x_dot_est, m, T, "linear");
    L(2, 5)  = interp2(mass_values, thrust_values, z_table_x_dot_est, m, T, "linear");
    L(2, 6)  = interp2(mass_values, thrust_values, z_dot_table_x_dot_est, m, T, "linear");
    L(2, 7)  = interp2(mass_values, thrust_values, theta_table_x_dot_est, m, T, "linear");
    L(2, 8)  = interp2(mass_values, thrust_values, theta_dot_table_x_dot_est, m, T, "linear");
    L(2, 9)  = interp2(mass_values, thrust_values, psi_table_x_dot_est, m, T, "linear");
    L(2, 10) = interp2(mass_values, thrust_values, psi_dot_table_x_dot_est, m, T, "linear");
    
    L(3, 1)  = interp2(mass_values, thrust_values, x_table_y_est, m, T, "linear");
    L(3, 2)  = interp2(mass_values, thrust_values, x_dot_table_y_est, m, T, "linear");
    L(3, 3)  = interp2(mass_values, thrust_values, y_table_y_est, m, T, "linear");
    L(3, 4)  = interp2(mass_values, thrust_values, y_dot_table_y_est, m, T, "linear");
    L(3, 5)  = interp2(mass_values, thrust_values, z_table_y_est, m, T, "linear");
    L(3, 6)  = interp2(mass_values, thrust_values, z_dot_table_y_est, m, T, "linear");
    L(3, 7)  = interp2(mass_values, thrust_values, theta_table_y_est, m, T, "linear");
    L(3, 8)  = interp2(mass_values, thrust_values, theta_dot_table_y_est, m, T, "linear");
    L(3, 9)  = interp2(mass_values, thrust_values, psi_table_y_est, m, T, "linear");
    L(3, 10) = interp2(mass_values, thrust_values, psi_dot_table_y_est, m, T, "linear");
    
    L(4, 1)  = interp2(mass_values, thrust_values, x_table_y_dot_est, m, T, "linear");
    L(4, 2)  = interp2(mass_values, thrust_values, x_dot_table_y_dot_est, m, T, "linear");
    L(4, 3)  = interp2(mass_values, thrust_values, y_table_y_dot_est, m, T, "linear");
    L(4, 4)  = interp2(mass_values, thrust_values, y_dot_table_y_dot_est, m, T, "linear");
    L(4, 5)  = interp2(mass_values, thrust_values, z_table_y_dot_est, m, T, "linear");
    L(4, 6)  = interp2(mass_values, thrust_values, z_dot_table_y_dot_est, m, T, "linear");
    L(4, 7)  = interp2(mass_values, thrust_values, theta_table_y_dot_est, m, T, "linear");
    L(4, 8)  = interp2(mass_values, thrust_values, theta_dot_table_y_dot_est, m, T, "linear");
    L(4, 9)  = interp2(mass_values, thrust_values, psi_table_y_dot_est, m, T, "linear");
    L(4, 10) = interp2(mass_values, thrust_values, psi_dot_table_y_dot_est, m, T, "linear");

    L(5, 1)  = interp2(mass_values, thrust_values, x_table_z_est, m, T, "linear");
    L(5, 2)  = interp2(mass_values, thrust_values, x_dot_table_z_est, m, T, "linear");
    L(5, 3)  = interp2(mass_values, thrust_values, y_table_z_est, m, T, "linear");
    L(5, 4)  = interp2(mass_values, thrust_values, y_dot_table_z_est, m, T, "linear");
    L(5, 5)  = interp2(mass_values, thrust_values, z_table_z_est, m, T, "linear");
    L(5, 6)  = interp2(mass_values, thrust_values, z_dot_table_z_est, m, T, "linear");
    L(5, 7)  = interp2(mass_values, thrust_values, theta_table_z_est, m, T, "linear");
    L(5, 8)  = interp2(mass_values, thrust_values, theta_dot_table_z_est, m, T, "linear");
    L(5, 9)  = interp2(mass_values, thrust_values, psi_table_z_est, m, T, "linear");
    L(5, 10) = interp2(mass_values, thrust_values, psi_dot_table_z_est, m, T, "linear");

    L(6, 1)  = interp2(mass_values, thrust_values, x_table_z_dot_est, m, T, "linear");
    L(6, 2)  = interp2(mass_values, thrust_values, x_dot_table_z_dot_est, m, T, "linear");
    L(6, 3)  = interp2(mass_values, thrust_values, y_table_z_dot_est, m, T, "linear");
    L(6, 4)  = interp2(mass_values, thrust_values, y_dot_table_z_dot_est, m, T, "linear");
    L(6, 5)  = interp2(mass_values, thrust_values, z_table_z_dot_est, m, T, "linear");
    L(6, 6)  = interp2(mass_values, thrust_values, z_dot_table_z_dot_est, m, T, "linear");
    L(6, 7)  = interp2(mass_values, thrust_values, theta_table_z_dot_est, m, T, "linear");
    L(6, 8)  = interp2(mass_values, thrust_values, theta_dot_table_z_dot_est, m, T, "linear");
    L(6, 9)  = interp2(mass_values, thrust_values, psi_table_z_dot_est, m, T, "linear");
    L(6, 10) = interp2(mass_values, thrust_values, psi_dot_table_z_dot_est, m, T, "linear");

    L(7, 1)  = interp2(mass_values, thrust_values, x_table_theta_est, m, T, "linear");
    L(7, 2)  = interp2(mass_values, thrust_values, x_dot_table_theta_est, m, T, "linear");
    L(7, 3)  = interp2(mass_values, thrust_values, y_table_theta_est, m, T, "linear");
    L(7, 4)  = interp2(mass_values, thrust_values, y_dot_table_theta_est, m, T, "linear");
    L(7, 5)  = interp2(mass_values, thrust_values, z_table_theta_est, m, T, "linear");
    L(7, 6)  = interp2(mass_values, thrust_values, z_dot_table_theta_est, m, T, "linear");
    L(7, 7)  = interp2(mass_values, thrust_values, theta_table_theta_est, m, T, "linear");
    L(7, 8)  = interp2(mass_values, thrust_values, theta_dot_table_theta_est, m, T, "linear");
    L(7, 9)  = interp2(mass_values, thrust_values, psi_table_theta_est, m, T, "linear");
    L(7, 10) = interp2(mass_values, thrust_values, psi_dot_table_theta_est, m, T, "linear");

    L(8, 1)  = interp2(mass_values, thrust_values, x_table_theta_dot_est, m, T, "linear");
    L(8, 2)  = interp2(mass_values, thrust_values, x_dot_table_theta_dot_est, m, T, "linear");
    L(8, 3)  = interp2(mass_values, thrust_values, y_table_theta_dot_est, m, T, "linear");
    L(8, 4)  = interp2(mass_values, thrust_values, y_dot_table_theta_dot_est, m, T, "linear");
    L(8, 5)  = interp2(mass_values, thrust_values, z_table_theta_dot_est, m, T, "linear");
    L(8, 6)  = interp2(mass_values, thrust_values, z_dot_table_theta_dot_est, m, T, "linear");
    L(8, 7)  = interp2(mass_values, thrust_values, theta_table_theta_dot_est, m, T, "linear");
    L(8, 8)  = interp2(mass_values, thrust_values, theta_dot_table_theta_dot_est, m, T, "linear");
    L(8, 9)  = interp2(mass_values, thrust_values, psi_table_theta_dot_est, m, T, "linear");
    L(8, 10) = interp2(mass_values, thrust_values, psi_dot_table_theta_dot_est, m, T, "linear");
    
    L(9, 1)  = interp2(mass_values, thrust_values, x_table_psi_est, m, T, "linear");
    L(9, 2)  = interp2(mass_values, thrust_values, x_dot_table_psi_est, m, T, "linear");
    L(9, 3)  = interp2(mass_values, thrust_values, y_table_psi_est, m, T, "linear");
    L(9, 4)  = interp2(mass_values, thrust_values, y_dot_table_psi_est, m, T, "linear");
    L(9, 5)  = interp2(mass_values, thrust_values, z_table_psi_est, m, T, "linear");
    L(9, 6)  = interp2(mass_values, thrust_values, z_dot_table_psi_est, m, T, "linear");
    L(9, 7)  = interp2(mass_values, thrust_values, theta_table_psi_est, m, T, "linear");
    L(9, 8)  = interp2(mass_values, thrust_values, theta_dot_table_psi_est, m, T, "linear");
    L(9, 9)  = interp2(mass_values, thrust_values, psi_table_psi_est, m, T, "linear");
    L(9, 10) = interp2(mass_values, thrust_values, psi_dot_table_psi_est, m, T, "linear");
    
    L(10, 1)  = interp2(mass_values, thrust_values, x_table_psi_dot_est, m, T, "linear");
    L(10, 2)  = interp2(mass_values, thrust_values, x_dot_table_psi_dot_est, m, T, "linear");
    L(10, 3)  = interp2(mass_values, thrust_values, y_table_psi_dot_est, m, T, "linear");
    L(10, 4)  = interp2(mass_values, thrust_values, y_dot_table_psi_dot_est, m, T, "linear");
    L(10, 5)  = interp2(mass_values, thrust_values, z_table_psi_dot_est, m, T, "linear");
    L(10, 6)  = interp2(mass_values, thrust_values, z_dot_table_psi_dot_est, m, T, "linear");
    L(10, 7)  = interp2(mass_values, thrust_values, theta_table_psi_dot_est, m, T, "linear");
    L(10, 8)  = interp2(mass_values, thrust_values, theta_dot_table_psi_dot_est, m, T, "linear");
    L(10, 9)  = interp2(mass_values, thrust_values, psi_table_psi_dot_est, m, T, "linear");
    L(10, 10) = interp2(mass_values, thrust_values, psi_dot_table_psi_dot_est, m, T, "linear");

    L_d = zeros(3,10);
    L_d(1, 1)  = interp2(mass_values, thrust_values, x_table_dx_est, m, T, "linear");
    L_d(1, 2)  = interp2(mass_values, thrust_values, x_dot_table_dx_est, m, T, "linear");
    L_d(1, 3)  = interp2(mass_values, thrust_values, y_table_dx_est, m, T, "linear");
    L_d(1, 4)  = interp2(mass_values, thrust_values, y_dot_table_dx_est, m, T, "linear");
    L_d(1, 5)  = interp2(mass_values, thrust_values, z_table_dx_est, m, T, "linear");
    L_d(1, 6)  = interp2(mass_values, thrust_values, z_dot_table_dx_est, m, T, "linear");
    L_d(1, 7)  = interp2(mass_values, thrust_values, theta_table_dx_est, m, T, "linear");
    L_d(1, 8)  = interp2(mass_values, thrust_values, theta_dot_table_dx_est, m, T, "linear");
    L_d(1, 9)  = interp2(mass_values, thrust_values, psi_table_dx_est, m, T, "linear");
    L_d(1, 10) = interp2(mass_values, thrust_values, psi_dot_table_dx_est, m, T, "linear");
    
    L_d(2, 1)  = interp2(mass_values, thrust_values, x_table_dy_est, m, T, "linear");
    L_d(2, 2)  = interp2(mass_values, thrust_values, x_dot_table_dy_est, m, T, "linear");
    L_d(2, 3)  = interp2(mass_values, thrust_values, y_table_dy_est, m, T, "linear");
    L_d(2, 4)  = interp2(mass_values, thrust_values, y_dot_table_dy_est, m, T, "linear");
    L_d(2, 5)  = interp2(mass_values, thrust_values, z_table_dy_est, m, T, "linear");
    L_d(2, 6)  = interp2(mass_values, thrust_values, z_dot_table_dy_est, m, T, "linear");
    L_d(2, 7)  = interp2(mass_values, thrust_values, theta_table_dy_est, m, T, "linear");
    L_d(2, 8)  = interp2(mass_values, thrust_values, theta_dot_table_dy_est, m, T, "linear");
    L_d(2, 9)  = interp2(mass_values, thrust_values, psi_table_dy_est, m, T, "linear");
    L_d(2, 10) = interp2(mass_values, thrust_values, psi_dot_table_dy_est, m, T, "linear");

    L_d(3, 1)  = interp2(mass_values, thrust_values, x_table_dz_est, m, T, "linear");
    L_d(3, 2)  = interp2(mass_values, thrust_values, x_dot_table_dz_est, m, T, "linear");
    L_d(3, 3)  = interp2(mass_values, thrust_values, y_table_dz_est, m, T, "linear");
    L_d(3, 4)  = interp2(mass_values, thrust_values, y_dot_table_dz_est, m, T, "linear");
    L_d(3, 5)  = interp2(mass_values, thrust_values, z_table_dz_est, m, T, "linear");
    L_d(3, 6)  = interp2(mass_values, thrust_values, z_dot_table_dz_est, m, T, "linear");
    L_d(3, 7)  = interp2(mass_values, thrust_values, theta_table_dz_est, m, T, "linear");
    L_d(3, 8)  = interp2(mass_values, thrust_values, theta_dot_table_dz_est, m, T, "linear");
    L_d(3, 9)  = interp2(mass_values, thrust_values, psi_table_dz_est, m, T, "linear");
    L_d(3, 10) = interp2(mass_values, thrust_values, psi_dot_table_dz_est, m, T, "linear");


    %% --- Interpolation to Get Gain K (unchanged) ---
    K = zeros(3,10);
    % Perform interpolation for Tx-related table values
    K(1, 1) = interp2(mass_values, thrust_values, x_table_Tx, m, T, "linear");
    K(1, 2) = interp2(mass_values, thrust_values, x_dot_table_Tx, m, T, "linear");
    K(1, 3) = interp2(mass_values, thrust_values, y_table_Tx, m, T, "linear");
    K(1, 4) = interp2(mass_values, thrust_values, y_dot_table_Tx, m, T, "linear");
    K(1, 5) = interp2(mass_values, thrust_values, z_table_Tx, m, T, "linear");
    K(1, 6) = interp2(mass_values, thrust_values, z_dot_table_Tx, m, T, "linear");
    K(1, 7) = interp2(mass_values, thrust_values, theta_table_Tx, m, T, "linear");
    K(1, 8) = interp2(mass_values, thrust_values, theta_dot_table_Tx, m, T, "linear");
    K(1, 9) = interp2(mass_values, thrust_values, psi_table_Tx, m, T, "linear");
    K(1, 10) = interp2(mass_values, thrust_values, psi_dot_table_Tx, m, T, "linear");
    
    % Perform interpolation for Ty-related table values
    K(2, 1) = interp2(mass_values, thrust_values, x_table_Ty, m, T, "linear");
    K(2, 2) = interp2(mass_values, thrust_values, x_dot_table_Ty, m, T, "linear");
    K(2, 3) = interp2(mass_values, thrust_values, y_table_Ty, m, T, "linear");
    K(2, 4) = interp2(mass_values, thrust_values, y_dot_table_Ty, m, T, "linear");
    K(2, 5) = interp2(mass_values, thrust_values, z_table_Ty, m, T, "linear");
    K(2, 6) = interp2(mass_values, thrust_values, z_dot_table_Ty, m, T, "linear");
    K(2, 7) = interp2(mass_values, thrust_values, theta_table_Ty, m, T, "linear");
    K(2, 8) = interp2(mass_values, thrust_values, theta_dot_table_Ty, m, T, "linear");
    K(2, 9) = interp2(mass_values, thrust_values, psi_table_Ty, m, T, "linear");
    K(2, 10) = interp2(mass_values, thrust_values, psi_dot_table_Ty, m, T, "linear");
    
    % Perform interpolation for Tz-related table values
    K(3, 1) = interp2(mass_values, thrust_values, x_table_Tz, m, T, "linear");
    K(3, 2) = interp2(mass_values, thrust_values, x_dot_table_Tz, m, T, "linear");
    K(3, 3) = interp2(mass_values, thrust_values, y_table_Tz, m, T, "linear");
    K(3, 4) = interp2(mass_values, thrust_values, y_dot_table_Tz, m, T, "linear");
    K(3, 5) = interp2(mass_values, thrust_values, z_table_Tz, m, T, "linear");
    K(3, 6) = interp2(mass_values, thrust_values, z_dot_table_Tz, m, T, "linear");
    K(3, 7) = interp2(mass_values, thrust_values, theta_table_Tz, m, T, "linear");
    K(3, 8) = interp2(mass_values, thrust_values, theta_dot_table_Tz, m, T, "linear");
    K(3, 9) = interp2(mass_values, thrust_values, psi_table_Tz, m, T, "linear");
    K(3, 10) = interp2(mass_values, thrust_values, psi_dot_table_Tz, m, T, "linear");

    
    %% --- Compute Control Force (Tx, Ty, Tz) ---
    sigma = diag(1e-1*[0,0,0.05,0.1,0.05,0.1,0.05,0.2,0.05,0.2]);
    bias = 1e-2*[0;0;0.01;0.01;0.01;0.01;0.25; 0.1;0.25;0.1];
    measurementError = 1*sigma * randn(10, 1) + 1*bias*time_now;
    error_state = state + 0*measurementError - reference_state;
    
    control_force = -K * error_state;   % state feedback control

    
    A_a = [A_d-L*C_d, B_d;
            -L_d*C_d,   eye(3)];
    B_a = [B_d;
            zeros(3,3)];
    L_a = [L;
           L_d];

    
    % Preliminary Tx, Ty, Tz from the control law:
    Tx = control_force(1) + 0*x_a(11);
    Ty = control_force(2) + 1*x_a(12);
    Tz = control_force(3) + 1*x_a(13);

    x_a_next = A_a*x_a + B_a*[Tx;Ty;Tz] + L_a*(state+0*measurementError);
    
    % --- Gravity Compensation ---
    Tx = Tx + m*g*cos(psi)*cos(theta);
    Ty = Ty + m*g*sin(psi);
    Tz = Tz + m*g*cos(psi)*sin(theta);
    
    % --- Compute Thrust Magnitude ---
    T = sqrt(Tx^2 + Ty^2 + Tz^2);
    
    % --- Compute TVC Command Angles from Control Force ---
    % These are now used as the command for the nozzle dynamics.
    cmd_theta = acos(Tx/T);          % commanded pitch (0 ≤ cmd_theta ≤ pi)
    cmd_psi   = atan2(-Tz, -Ty);       % commanded yaw  (in (-pi,pi])
    if cmd_theta > 0.25
        cmd_theta = 0.25;            % enforce a maximum command pitch
    end
    cmd(:,t_step)=[rad2deg(cmd_theta);rad2deg(cmd_psi)];
    
    % --- Form Commanded Nozzle Orientation (as a unit vector) ---
    r_cmd = [cos(cmd_theta);
             sin(cmd_theta)*cos(cmd_psi);
             sin(cmd_theta)*sin(cmd_psi)];
    
    % --- Shortest-Path Flip: If the commanded orientation is on the far side, flip it ---
    if dot(r_noz, r_cmd) < 0
        r_cmd = -r_cmd;
    end
    
    %% --- Update Nozzle Dynamics (using Euler integration) ---
    % Nozzle dynamics:
    %   dot(r_noz) = v_noz
    %   dot(v_noz) = -2*zeta_noz*wn_noz*v_noz + wn_noz^2 * ([r_cmd - r_noz]_tan)
    %
    % Compute the error projected onto the tangent plane:
    err = r_cmd - r_noz;
    err_tan = err - (dot(r_noz, err))*r_noz;
    if norm(err_tan)>cmd_max %input saturation
        err_tan = (cmd_max/norm(err_tan))*err_tan;
    end
    
    % Compute derivatives:
    dr_noz = v_noz;
    dv_noz = -2*zeta_noz*wn_noz*v_noz + wn_noz^2 * err_tan;
    
    % Euler integration step:
    r_noz = r_noz + dr_noz * dt;
    v_noz = v_noz + dv_noz * dt;
    
    % Re-normalize r_noz (to keep it on the unit sphere):
    r_noz = r_noz / norm(r_noz);
    
    % --- Enforce Nozzle Pitch Clamp (θ_noz ≤ 0.25 rad) ---
    theta_noz = acos(r_noz(1));  % since r_noz(1)=cos(theta_noz)
    if theta_noz > 0.25
        % Get the current nozzle yaw from r_noz:
        psi_noz = atan2(r_noz(3), r_noz(2));
        theta_noz = 0.25;
        % Reset r_noz to be on the boundary:
        r_noz = [cos(theta_noz);
                 sin(theta_noz)*cos(psi_noz);
                 sin(theta_noz)*sin(psi_noz)];
        % Remove any component of v_noz that would increase theta_noz:
        drdtheta = [-sin(theta_noz);
                     cos(theta_noz)*cos(psi_noz);
                     cos(theta_noz)*sin(psi_noz)];
        if dot(v_noz, drdtheta) > 0
            v_noz = v_noz - dot(v_noz, drdtheta)*drdtheta; %set the velocity of dtheta direction to be zero
        end
        % Ensure v_noz is tangent:
        v_noz = v_noz - dot(r_noz, v_noz)*r_noz;
    end
    
    % Store actual nozzle angles for later plotting:
    theta_noz = acos(r_noz(1));
    psi_noz = atan2(r_noz(3), r_noz(2));
    theta_noz_data(t_step) = rad2deg(theta_noz);
    psi_noz_data(t_step) = rad2deg(psi_noz);
    
    %% --- Use the True Nozzle Orientation to Compute Thrust Vector ---
    % The nozzle output (actual orientation) is now used to compute the effective thrust vector.
        % --- Thrust Saturation ---
    if T > 2500
        T = 2500;
    end
    if T < 0
        T = 0;
    end
    Tx = T * cos(theta_noz);
    Ty = -T * sin(theta_noz) * cos(psi_noz);
    Tz = -T * sin(theta_noz) * sin(psi_noz);
    
    %% --- Update Mass and Inertia ---
    m_d = -T/(Isp*g);
    m = m + m_d * dt;
    Ixx_d = 1/2*m_d*0.2^2; % Placeholder derivative
    Iyy_d = 0.3433*m_d;
    Izz_d = 0.3433*m_d;
    Ixx = Ixx + Ixx_d*dt;
    Iyy = Iyy + Iyy_d*dt;
    Izz = Izz + Izz_d*dt;
    
    %% --- Disturbances --- 5.5m/s 15.2 14m/s 112.3 
    windforce_y = 15.2 * sin(0.2 * pi * time_now);
    windforce_z = 15.2 * cos(0.2 * pi * time_now);
    
    %% --- Rocket Dynamics (EOM_displaced_nozzle) ---
    % (These equations are as before; note that Tx, Ty, Tz now come from the nozzle dynamics.)

    % Without nozzle offset  
    f1 = x_d;
    f2 = -(Ty*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - Tz*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + g*m + m_d*x_d - Tx*cos(psi)*cos(theta))/m;
    f3 = y_d;
    f4 = (Ty*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - Tz*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - m_d*y_d + Tx*cos(theta)*sin(psi))/m;
    f5 = z_d;
    f6 = -(m_d*z_d + Tx*sin(theta) - Tz*cos(phi)*cos(theta) - Ty*cos(theta)*sin(phi))/m;
    f7 = theta_d;
    f8 = (Izz*phi_d*psi_d - Ixx*phi_d*psi_d - Iyy_d*theta_d + Tz*dcn*cos(phi) + Ty*dcn*sin(phi))/Iyy - phi_d*psi_d*(cos(theta)*sin(phi)^2 + cos(phi)*cos(theta)*sin(phi)) + psi_d*theta_d*(cos(phi)^2*sin(theta) - cos(phi)*sin(phi)*sin(theta));
    f9 = psi_d;
    f10 = (Izz*phi_d*theta_d - Ty*dcn*cos(phi) - Izz_d*psi_d*cos(theta) + Tz*dcn*sin(phi) - Izz*phi_d*psi_d*cos(theta) + Ixx*phi_d*theta_d*cos(theta) - Iyy*phi_d*theta_d*cos(theta) + Izz*phi_d*psi_d*cos(phi)^2*cos(theta) + Izz*psi_d*theta_d*cos(phi)^2*sin(theta) + Izz*phi_d*psi_d*cos(phi)*cos(theta)*sin(phi) + Izz*psi_d*theta_d*cos(phi)*sin(phi)*sin(theta))/(Izz*cos(theta));
    
    % With nozzle offset (5 cm in y-direction)
    % f1 = x_d;
    % f2 = -( Ty*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - ...
    %        Tz*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + ...
    %        g*m + m_d*x_d - Tx*cos(psi)*cos(theta) )/m;
    % f3 = y_d;
    % f4 = ( Ty*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - ...
    %        Tz*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - m_d*y_d + ...
    %        Tx*cos(theta)*sin(psi) )/m;
    % f5 = z_d;
    % f6 = -( m_d*z_d + Tx*sin(theta) - Tz*cos(phi)*cos(theta) - ...
    %        Ty*cos(theta)*sin(phi) )/m;
    % f7 = theta_d;
    % f8 = ( sin(phi)*(Tx/20 + Ty*dcn) - Iyy_d*theta_d - Ixx*phi_d*psi_d + Izz*phi_d*psi_d + ...
    %        Tz*dcn*cos(phi) )/Iyy - phi_d*psi_d*(cos(theta)*sin(phi)^2 + cos(phi)*cos(theta)*sin(phi)) + ...
    %        psi_d*theta_d*(cos(phi)^2*sin(theta) - cos(phi)*sin(phi)*sin(theta));
    % f9 = psi_d;
    % f10 = phi_d*theta_d*(cos(phi)^2/cos(theta) + sin(phi)^2/cos(theta)) - ...
    %       ( Izz_d*psi_d - Ixx*phi_d*theta_d + Iyy*phi_d*theta_d + (cos(phi)*(Tx/20 + Ty*dcn))/cos(theta) - ...
    %         (Tz*dcn*sin(phi))/cos(theta) )/Izz + phi_d*psi_d*(cos(phi)*sin(phi) - sin(phi)^2) + ...
    %       psi_d*theta_d*((cos(phi)^2*sin(theta))/cos(theta) + (cos(phi)*sin(phi)*sin(theta))/cos(theta));
      
    % Add wind disturbances:
    f4 = f4 + windforce_y/m;
    f6 = f6 + windforce_z/m;
    f8 = f8 - windforce_z*0.33/Iyy;
    f10 = f10 + windforce_y*0.33/Izz;
    
    %% --- Update Rocket State (Euler integration) ---
    state(1) = state(1) + f1 * dt;
    state(2) = state(2) + f2 * dt;
    state(3) = state(3) + f3 * dt;
    state(4) = state(4) + f4 * dt;
    state(5) = state(5) + f5 * dt;
    state(6) = state(6) + f6 * dt;
    state(7) = state(7) + f7 * dt;
    state(8) = state(8) + f8 * dt;
    state(9) = state(9) + f9 * dt;
    state(10)= state(10)+ f10 * dt;
    
    %% --- Store Data ---
    trajectory(:, t_step) = state;
    estimate_state (:,t_step) = x_a(1:10);
    estimate_dist (:,t_step) = x_a(11:13);
    control_forces_data(:, t_step) = [Tx; Ty; Tz];
    total_forces(t_step) = T;
    ma(t_step) = m;
    x_a = x_a_next;
end

% %% --- Plotting Results ---
% 
% dt = time(2) - time(1);
% 
% % Define search start time in seconds
% min_search_time = 15;  % [s]
% 
% % Compute index to start search from
% start_idx = round(min_search_time / dt);
% 
% % Ensure start_idx does not exceed array bounds
% start_idx = min(start_idx, length(time));
% 
% % Find the landing index: where trajectory(1,:) drops below 0.1
% landingIndex = find(trajectory(1, start_idx:end) < 0.1, 1);
% 
% 
% 
% % === Fuel Consumption Summary ===
% if ~isempty(landingIndex)
%     landingVelocity = trajectory(2, landingIndex);
%     final_mass = ma(landingIndex);
%     fuel_used = m_init - final_mass;
%     landingTime = landingIndex * dt;
%     fprintf('Rocket landed at %.2f seconds\n', landingTime);
%     fprintf('Landing velocity: %.4f m/s\n', landingVelocity);
%     fprintf('Final mass at landing: %.2f kg\n', final_mass);
%     fprintf('Fuel consumed: %.2f kg\n', fuel_used);
% else
%     disp('x never goes below 0.');
% end
% 
% % If there is such an index, extract the corresponding velocity
% if ~isempty(FuelStopIndex)
%     fuelStopTime = FuelStopIndex*dt;
%     fprintf('Fuel empty at time: %.2f seconds\n ',fuelStopTime);
% else
%     disp('Fuel never goes empty');
% end


% Plot results
time = 0:dt:sim_time - dt;
figure;

subplot(5, 1, 1);
plot(time, trajectory(7, :)); % theta
hold on;
plot(time, trajectory(9, :)); % psi
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('\theta', '\psi');
title('Angles');

subplot(5, 1, 2);
plot(time, control_forces_data(1, :)); % Tx
hold on;
plot(time, control_forces_data(2, :)); % Ty
plot(time, control_forces_data(3, :)); % Tz
xlabel('Time (s)');
ylabel('Control Forces (N)');
legend('Tx', 'Ty', 'Tz');
title('Control Forces (Tx, Ty, Tz)');


subplot(5, 1, 3);
plot(time, control_forces_data(2, :)); % Ty
hold on;
plot(time, control_forces_data(3, :)); % Tz
xlabel('Time (s)');
ylabel('Control Forces (N)');
legend('Ty', 'Tz');
title('Control Forces (Ty, Tz)');


subplot(5, 1, 4);
plot(time, total_forces); % Total thrust
xlabel('Time (s)');
ylabel('Control Forces (N)');
legend('T');
title('Total Force');


subplot(5,1,5);
plot(time, ma); % mass
xlabel('Time(s)');
ylabel('Mass(kg)');
legend('m');
title('Mass Change')


figure;
subplot(2,1,1);
plot(time, trajectory(1, :)); % x position
hold on;
plot(time, trajectory(3, :)); % y position
plot(time, trajectory(5, :)); % z position
plot(time, reference_x(1:length(time),1)'); % x reference
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z', 'x reference');
title('Position');
grid on;
grid minor;
% if ~isnan(landingTime)
%     xline(landingTime, '--r', 'Landing', 'LabelOrientation', 'horizontal');
% end
% if ~isnan(fuelStopTime)
%     xline(fuelStopTime, '--k', 'Fuel Empty', 'LabelOrientation', 'horizontal');
% end


subplot(2,1,2);
plot(time, trajectory(2, :)); % x velocity
hold on;
plot(time, trajectory(4, :)); % y velocity
hold on;
plot(time, trajectory(6, :)); % z velocity
hold on;
plot(time, reference_xdot(1:length(time),1)'); % x reference
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('x\_d', 'y\_d', 'z\_d', 'x\_d reference');
title('Velocity');
grid on;
grid minor;
% if ~isnan(landingTime)
%     xline(landingTime, '--r', 'Landing', 'LabelOrientation', 'horizontal');
% end
% if ~isnan(fuelStopTime)
%     xline(fuelStopTime, '--k', 'Fuel Empty', 'LabelOrientation', 'horizontal');
% end


figure;
subplot(2,1,1);
plot(time,cmd(1,:));
hold on;
plot(time,theta_noz_data,LineWidth=2);
xlabel('Time(s)');
ylabel('Anagle(deg)');
legend('theta_{cmd}','theta_{tvc}');
title('\theta_{tvc}');



subplot(2,1,2);
plot(time,cmd(2,:));
hold on
plot(time,psi_noz_data,LineWidth=2);
xlabel('Time(s)');
ylabel('Anagle(deg)');
legend('psi_{cmd}','psi_{tvc}');
title('\psi_{tvc}');

figure;
subplot(1,2,1)
plot(time,estimate_state(1,:));
hold on;
plot(time,estimate_state(2,:));
plot(time,estimate_state(3,:));
plot(time,estimate_state(4,:));
plot(time,estimate_state(5,:));
plot(time,estimate_state(6,:));
plot(time,estimate_state(7,:));
plot(time,estimate_state(8,:));
plot(time,estimate_state(9,:));
plot(time,estimate_state(10,:));
xlabel('Time(s)');
title('est state');


subplot(1,2,2)
plot(time,trajectory(1,:));
hold on;
plot(time,trajectory(2,:));
plot(time,trajectory(3,:));
plot(time,trajectory(4,:));
plot(time,trajectory(5,:));
plot(time,trajectory(6,:));
plot(time,trajectory(7,:));
plot(time,trajectory(8,:));
plot(time,trajectory(9,:));
plot(time,trajectory(10,:));
xlabel('Time(s)');
title('trajectory');

% Calculate error between estimated state and actual trajectory
error = estimate_state - trajectory;

% Compute 2-norm of the error at each time step
error_norms = vecnorm(error, 2, 1);  % 2-norm across rows (i.e., across the 10 states) for each time step

% Find time indices up to 28 seconds
valid_indices = time <= 28;

% Sum the 2-norm errors over time up to 28 seconds
total_error_norm = sum(error_norms(valid_indices));




% Display the result
fprintf('Total 2-norm error up to 28 seconds: %.4f\n', total_error_norm);

figure;
plot(time,estimate_dist(1,:));
hold on;
plot(time,estimate_dist(2,:));
plot(time,estimate_dist(3,:));
xlabel('Time(s)');
title('est dist');


landingIndex = find(trajectory(1, :) < 0, 1);

% If there is such an index, extract the corresponding velocity
if ~isempty(landingIndex)
    landingVelocity = trajectory(2, landingIndex);
    disp(['Landing velocity when x first < 0: ', num2str(landingVelocity),'at time:', num2str(landingIndex*dt)]);
else
    disp('x never goes below 0.');
end

FuelStopIndex = find(ma(:) < 105, 1);

% If there is such an index, extract the corresponding velocity
if ~isempty(FuelStopIndex)
    fprintf(' fuel empty at time: %.2f\n ',FuelStopIndex*dt);
else
    disp('fuel never goes empty');
end

disp("fuel used");
disp(m_init-ma(landingIndex));


y = trajectory(3, :);
z = trajectory(5, :);

distance_squared = y.^2 + z.^2;
[max_distance_squared, max_idx] = max(distance_squared);
max_distance = sqrt(max_distance_squared);

fprintf('Maximum distance from origin in y-z plane = %.4f\n m', max_distance);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Additional Figures for Analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Figure 1: Altitude vs. Reference
figure;
plot(time, trajectory(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, reference_x(1:length(time),1)', 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude vs. Reference');
legend('Actual Altitude', 'Reference Altitude');
grid on;

% Figure 2: Velocity vs. Reference
figure;
plot(time, trajectory(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, reference_xdot(1:length(time),1)', 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity vs. Reference');
legend('Actual Velocity', 'Reference Velocity');
grid on;

% Figure 3: Tracking Error (Altitude)
tracking_error = trajectory(1,:) - reference_x(1:length(time),1)';
figure;
plot(time, tracking_error, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Altitude Tracking Error (m)');
title('Tracking Error');
grid on;

% Figure 4: Control Input
figure;
plot(time, control_forces_data(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, control_forces_data(2,:), 'r', 'LineWidth', 1.5);
plot(time, control_forces_data(3,:), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Force (N)');
title('Control Input (Tx, Ty, Tz)');
legend('Tx', 'Ty', 'Tz');
grid on;

% Figure 5: Cumulative Fuel Usage
% Compute cumulative fuel usage based on total thrust (approximation)
% Assuming instantaneous mdot = T / (Isp * g), integrate over time.
mdot = total_forces ./ (Isp * g);
cumulative_fuel_usage = cumtrapz(time, abs(mdot));
figure;
plot(time, cumulative_fuel_usage, 'm', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Cumulative Fuel Usage (kg)');
title('Cumulative Fuel Usage');
grid on;
