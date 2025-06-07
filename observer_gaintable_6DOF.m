function observer_gaintable_6DOF()

ts = 0.01; % sampling time (s)
dcn = 0.67; % distance from center of mass to nozzle
radius = 0.2; % rocket radius
height = 2; % rocket height
Kr = 1/12 * (3 * radius^2 + height^2); % Kr = inertia / mass
Isp = 130;
g = 9.81;

%% Preallocate storage for gain look-up table
% Range of mass and thrust values
mass_values = linspace(50, 200, 50);
thrust_values = linspace(0.1, 2500, 50); 

mass_num = length(mass_values);
thrust_num = length(thrust_values);

%x_est
x_table_x_est = zeros(mass_num,thrust_num);
x_dot_table_x_est = zeros(mass_num,thrust_num);
y_table_x_est = zeros(mass_num,thrust_num);
y_dot_table_x_est = zeros(mass_num,thrust_num);
z_table_x_est = zeros(mass_num,thrust_num);
z_dot_table_x_est = zeros(mass_num,thrust_num);
theta_table_x_est = zeros(mass_num,thrust_num);
theta_dot_table_x_est = zeros(mass_num,thrust_num);
psi_table_x_est = zeros(mass_num,thrust_num);
psi_dot_table_x_est = zeros(mass_num,thrust_num);
dx_table_x_est = zeros(mass_num,thrust_num);
dy_table_x_est = zeros(mass_num,thrust_num);
dz_table_x_est = zeros(mass_num,thrust_num);

%x_dot_est
x_table_x_dot_est = zeros(mass_num,thrust_num);
x_dot_table_x_dot_est = zeros(mass_num,thrust_num);
y_table_x_dot_est = zeros(mass_num,thrust_num);
y_dot_table_x_dot_est = zeros(mass_num,thrust_num);
z_table_x_dot_est = zeros(mass_num,thrust_num);
z_dot_table_x_dot_est = zeros(mass_num,thrust_num);
theta_table_x_dot_est = zeros(mass_num,thrust_num);
theta_dot_table_x_dot_est = zeros(mass_num,thrust_num);
psi_table_x_dot_est = zeros(mass_num,thrust_num);
psi_dot_table_x_dot_est = zeros(mass_num,thrust_num);
dx_table_x_dot_est = zeros(mass_num,thrust_num);
dy_table_x_dot_est = zeros(mass_num,thrust_num);
dz_table_x_dot_est = zeros(mass_num,thrust_num);

%y_est
x_table_y_est = zeros(mass_num,thrust_num);
x_dot_table_y_est = zeros(mass_num,thrust_num);
y_table_y_est = zeros(mass_num,thrust_num);
y_dot_table_y_est = zeros(mass_num,thrust_num);
z_table_y_est = zeros(mass_num,thrust_num);
z_dot_table_y_est = zeros(mass_num,thrust_num);
theta_table_y_est = zeros(mass_num,thrust_num);
theta_dot_table_y_est = zeros(mass_num,thrust_num);
psi_table_y_est = zeros(mass_num,thrust_num);
psi_dot_table_y_est = zeros(mass_num,thrust_num);
dx_table_y_est = zeros(mass_num,thrust_num);
dy_table_y_est = zeros(mass_num,thrust_num);
dz_table_y_est = zeros(mass_num,thrust_num);

%y_dot_est
x_table_y_dot_est = zeros(mass_num,thrust_num);
x_dot_table_y_dot_est = zeros(mass_num,thrust_num);
y_table_y_dot_est = zeros(mass_num,thrust_num);
y_dot_table_y_dot_est = zeros(mass_num,thrust_num);
z_table_y_dot_est = zeros(mass_num,thrust_num);
z_dot_table_y_dot_est = zeros(mass_num,thrust_num);
theta_table_y_dot_est = zeros(mass_num,thrust_num);
theta_dot_table_y_dot_est = zeros(mass_num,thrust_num);
psi_table_y_dot_est = zeros(mass_num,thrust_num);
psi_dot_table_y_dot_est = zeros(mass_num,thrust_num);
dx_table_y_dot_est = zeros(mass_num,thrust_num);
dy_table_y_dot_est = zeros(mass_num,thrust_num);
dz_table_y_dot_est = zeros(mass_num,thrust_num);

%z_est
x_table_z_est = zeros(mass_num,thrust_num);
x_dot_table_z_est = zeros(mass_num,thrust_num);
y_table_z_est = zeros(mass_num,thrust_num);
y_dot_table_z_est = zeros(mass_num,thrust_num);
z_table_z_est = zeros(mass_num,thrust_num);
z_dot_table_z_est = zeros(mass_num,thrust_num);
theta_table_z_est = zeros(mass_num,thrust_num);
theta_dot_table_z_est = zeros(mass_num,thrust_num);
psi_table_z_est = zeros(mass_num,thrust_num);
psi_dot_table_z_est = zeros(mass_num,thrust_num);
dx_table_z_est = zeros(mass_num,thrust_num);
dy_table_z_est = zeros(mass_num,thrust_num);
dz_table_z_est = zeros(mass_num,thrust_num);

%z_dot_est
x_table_z_dot_est = zeros(mass_num,thrust_num);
x_dot_table_z_dot_est = zeros(mass_num,thrust_num);
y_table_z_dot_est = zeros(mass_num,thrust_num);
y_dot_table_z_dot_est = zeros(mass_num,thrust_num);
z_table_z_dot_est = zeros(mass_num,thrust_num);
z_dot_table_z_dot_est = zeros(mass_num,thrust_num);
theta_table_z_dot_est = zeros(mass_num,thrust_num);
theta_dot_table_z_dot_est = zeros(mass_num,thrust_num);
psi_table_z_dot_est = zeros(mass_num,thrust_num);
psi_dot_table_z_dot_est = zeros(mass_num,thrust_num);
dx_table_z_dot_est = zeros(mass_num,thrust_num);
dy_table_z_dot_est = zeros(mass_num,thrust_num);
dz_table_z_dot_est = zeros(mass_num,thrust_num);

%theta_est
x_table_theta_est = zeros(mass_num,thrust_num);
x_dot_table_theta_est = zeros(mass_num,thrust_num);
y_table_theta_est = zeros(mass_num,thrust_num);
y_dot_table_theta_est = zeros(mass_num,thrust_num);
z_table_theta_est = zeros(mass_num,thrust_num);
z_dot_table_theta_est = zeros(mass_num,thrust_num);
theta_table_theta_est = zeros(mass_num,thrust_num);
theta_dot_table_theta_est = zeros(mass_num,thrust_num);
psi_table_theta_est = zeros(mass_num,thrust_num);
psi_dot_table_theta_est = zeros(mass_num,thrust_num);
dx_table_theta_est = zeros(mass_num,thrust_num);
dy_table_theta_est = zeros(mass_num,thrust_num);
dz_table_theta_est = zeros(mass_num,thrust_num);

%theta_dot_est
x_table_theta_dot_est = zeros(mass_num,thrust_num);
x_dot_table_theta_dot_est = zeros(mass_num,thrust_num);
y_table_theta_dot_est = zeros(mass_num,thrust_num);
y_dot_table_theta_dot_est = zeros(mass_num,thrust_num);
z_table_theta_dot_est = zeros(mass_num,thrust_num);
z_dot_table_theta_dot_est = zeros(mass_num,thrust_num);
theta_table_theta_dot_est = zeros(mass_num,thrust_num);
theta_dot_table_theta_dot_est = zeros(mass_num,thrust_num);
psi_table_theta_dot_est = zeros(mass_num,thrust_num);
psi_dot_table_theta_dot_est = zeros(mass_num,thrust_num);
dx_table_theta_dot_est = zeros(mass_num,thrust_num);
dy_table_theta_dot_est = zeros(mass_num,thrust_num);
dz_table_theta_dot_est = zeros(mass_num,thrust_num);

%psi_est
x_table_psi_est = zeros(mass_num,thrust_num);
x_dot_table_psi_est = zeros(mass_num,thrust_num);
y_table_psi_est = zeros(mass_num,thrust_num);
y_dot_table_psi_est = zeros(mass_num,thrust_num);
z_table_psi_est = zeros(mass_num,thrust_num);
z_dot_table_psi_est = zeros(mass_num,thrust_num);
theta_table_psi_est = zeros(mass_num,thrust_num);
theta_dot_table_psi_est = zeros(mass_num,thrust_num);
psi_table_psi_est = zeros(mass_num,thrust_num);
psi_dot_table_psi_est = zeros(mass_num,thrust_num);
dx_table_psi_est = zeros(mass_num,thrust_num);
dy_table_psi_est = zeros(mass_num,thrust_num);
dz_table_psi_est = zeros(mass_num,thrust_num);

%psi_dot_est
x_table_psi_dot_est = zeros(mass_num,thrust_num);
x_dot_table_psi_dot_est = zeros(mass_num,thrust_num);
y_table_psi_dot_est = zeros(mass_num,thrust_num);
y_dot_table_psi_dot_est = zeros(mass_num,thrust_num);
z_table_psi_dot_est = zeros(mass_num,thrust_num);
z_dot_table_psi_dot_est = zeros(mass_num,thrust_num);
theta_table_psi_dot_est = zeros(mass_num,thrust_num);
theta_dot_table_psi_dot_est = zeros(mass_num,thrust_num);
psi_table_psi_dot_est = zeros(mass_num,thrust_num);
psi_dot_table_psi_dot_est = zeros(mass_num,thrust_num);
dx_table_psi_dot_est = zeros(mass_num,thrust_num);
dy_table_psi_dot_est = zeros(mass_num,thrust_num);
dz_table_psi_dot_est = zeros(mass_num,thrust_num);

%dx_est
x_table_dx_est = zeros(mass_num,thrust_num);
x_dot_table_dx_est = zeros(mass_num,thrust_num);
y_table_dx_est = zeros(mass_num,thrust_num);
y_dot_table_dx_est = zeros(mass_num,thrust_num);
z_table_dx_est = zeros(mass_num,thrust_num);
z_dot_table_dx_est = zeros(mass_num,thrust_num);
theta_table_dx_est = zeros(mass_num,thrust_num);
theta_dot_table_dx_est = zeros(mass_num,thrust_num);
psi_table_dx_est = zeros(mass_num,thrust_num);
psi_dot_table_dx_est = zeros(mass_num,thrust_num);
dx_table_dx_est = zeros(mass_num,thrust_num);
dy_table_dx_est = zeros(mass_num,thrust_num);
dz_table_dx_est = zeros(mass_num,thrust_num);

%dy_est
x_table_dy_est = zeros(mass_num,thrust_num);
x_dot_table_dy_est = zeros(mass_num,thrust_num);
y_table_dy_est = zeros(mass_num,thrust_num);
y_dot_table_dy_est = zeros(mass_num,thrust_num);
z_table_dy_est = zeros(mass_num,thrust_num);
z_dot_table_dy_est = zeros(mass_num,thrust_num);
theta_table_dy_est = zeros(mass_num,thrust_num);
theta_dot_table_dy_est = zeros(mass_num,thrust_num);
psi_table_dy_est = zeros(mass_num,thrust_num);
psi_dot_table_dy_est = zeros(mass_num,thrust_num);
dx_table_dy_est = zeros(mass_num,thrust_num);
dy_table_dy_est = zeros(mass_num,thrust_num);
dz_table_dy_est = zeros(mass_num,thrust_num);

%dz_est
x_table_dz_est = zeros(mass_num,thrust_num);
x_dot_table_dz_est = zeros(mass_num,thrust_num);
y_table_dz_est = zeros(mass_num,thrust_num);
y_dot_table_dz_est = zeros(mass_num,thrust_num);
z_table_dz_est = zeros(mass_num,thrust_num);
z_dot_table_dz_est = zeros(mass_num,thrust_num);
theta_table_dz_est = zeros(mass_num,thrust_num);
theta_dot_table_dz_est = zeros(mass_num,thrust_num);
psi_table_dz_est = zeros(mass_num,thrust_num);
psi_dot_table_dz_est = zeros(mass_num,thrust_num);
dx_table_dz_est = zeros(mass_num,thrust_num);
dy_table_dz_est = zeros(mass_num,thrust_num);
dz_table_dz_est = zeros(mass_num,thrust_num);






%% Contruct Observer Gain table 


% Loop over each combination of mass and thrust to calculate LQR gains
for i = 1: mass_num
    for j = 1: thrust_num

        % Update mass and thrust
        m = mass_values(i);
        T = thrust_values(j);     

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
        
        %aug plant for disturbance of Tx Ty Tz observer (dx, dy, dz)
       

        sysc = ss(A_c,B_c,C_c,D_c);
        sysd = c2d(sysc,ts,'zoh');

        A_d = sysd.A;
        B_d = sysd.B;
        C_d = sysd.C;
        D_d = sysd.D;

        Aa = [A_d,                B_d;
                zeros(3,10),       eye(3)]; 

        Ba = [B_d;
                zeros(3,3)];

        Ca = [C_d, zeros(10,3)];

        Da = 0;
        
        desired_poles = linspace(0.85, 0.95, 13);
        
        L = place(Aa', Ca', desired_poles)';



        % Extract values for x_est
        x_table_x_est(i, j) = L(1, 1);
        x_dot_table_x_est(i, j) = L(1, 2);
        y_table_x_est(i, j) = L(1, 3);
        y_dot_table_x_est(i, j) = L(1, 4);
        z_table_x_est(i, j) = L(1, 5);
        z_dot_table_x_est(i, j) = L(1, 6);
        theta_table_x_est(i, j) = L(1, 7);
        theta_dot_table_x_est(i, j) = L(1, 8);
        psi_table_x_est(i, j) = L(1, 9);
        psi_dot_table_x_est(i, j) = L(1, 10);

        % Extract values for x_dot_est
        x_table_x_dot_est(i, j)     = L(2, 1);
        x_dot_table_x_dot_est(i, j) = L(2, 2);
        y_table_x_dot_est(i, j)     = L(2, 3);
        y_dot_table_x_dot_est(i, j) = L(2, 4);
        z_table_x_dot_est(i, j)     = L(2, 5);
        z_dot_table_x_dot_est(i, j) = L(2, 6);
        theta_table_x_dot_est(i, j) = L(2, 7);
        theta_dot_table_x_dot_est(i, j) = L(2, 8);
        psi_table_x_dot_est(i, j)   = L(2, 9);
        psi_dot_table_x_dot_est(i, j) = L(2, 10);
        
        % Extract values for y_est
        x_table_y_est(i, j)     = L(3, 1);
        x_dot_table_y_est(i, j) = L(3, 2);
        y_table_y_est(i, j)     = L(3, 3);
        y_dot_table_y_est(i, j) = L(3, 4);
        z_table_y_est(i, j)     = L(3, 5);
        z_dot_table_y_est(i, j) = L(3, 6);
        theta_table_y_est(i, j) = L(3, 7);
        theta_dot_table_y_est(i, j) = L(3, 8);
        psi_table_y_est(i, j)   = L(3, 9);
        psi_dot_table_y_est(i, j) = L(3, 10);

        % Extract values for y_dot_est
        x_table_y_dot_est(i, j)     = L(4, 1);
        x_dot_table_y_dot_est(i, j) = L(4, 2);
        y_table_y_dot_est(i, j)     = L(4, 3);
        y_dot_table_y_dot_est(i, j) = L(4, 4);
        z_table_y_dot_est(i, j)     = L(4, 5);
        z_dot_table_y_dot_est(i, j) = L(4, 6);
        theta_table_y_dot_est(i, j) = L(4, 7);
        theta_dot_table_y_dot_est(i, j) = L(4, 8);
        psi_table_y_dot_est(i, j)   = L(4, 9);
        psi_dot_table_y_dot_est(i, j) = L(4, 10);
        
        % Extract values for z_est
        x_table_z_est(i, j)     = L(5, 1);
        x_dot_table_z_est(i, j) = L(5, 2);
        y_table_z_est(i, j)     = L(5, 3);
        y_dot_table_z_est(i, j) = L(5, 4);
        z_table_z_est(i, j)     = L(5, 5);
        z_dot_table_z_est(i, j) = L(5, 6);
        theta_table_z_est(i, j) = L(5, 7);
        theta_dot_table_z_est(i, j) = L(5, 8);
        psi_table_z_est(i, j)   = L(5, 9);
        psi_dot_table_z_est(i, j) = L(5, 10);
        
        % Extract values for z_dot_est
        x_table_z_dot_est(i, j)     = L(6, 1);
        x_dot_table_z_dot_est(i, j) = L(6, 2);
        y_table_z_dot_est(i, j)     = L(6, 3);
        y_dot_table_z_dot_est(i, j) = L(6, 4);
        z_table_z_dot_est(i, j)     = L(6, 5);
        z_dot_table_z_dot_est(i, j) = L(6, 6);
        theta_table_z_dot_est(i, j) = L(6, 7);
        theta_dot_table_z_dot_est(i, j) = L(6, 8);
        psi_table_z_dot_est(i, j)   = L(6, 9);
        psi_dot_table_z_dot_est(i, j) = L(6, 10);

        % Extract values for theta_est
        x_table_theta_est(i, j)     = L(7, 1);
        x_dot_table_theta_est(i, j) = L(7, 2);
        y_table_theta_est(i, j)     = L(7, 3);
        y_dot_table_theta_est(i, j) = L(7, 4);
        z_table_theta_est(i, j)     = L(7, 5);
        z_dot_table_theta_est(i, j) = L(7, 6);
        theta_table_theta_est(i, j) = L(7, 7);
        theta_dot_table_theta_est(i, j) = L(7, 8);
        psi_table_theta_est(i, j)   = L(7, 9);
        psi_dot_table_theta_est(i, j) = L(7, 10);
        
        % Extract values for theta_dot_est
        x_table_theta_dot_est(i, j)     = L(8, 1);
        x_dot_table_theta_dot_est(i, j) = L(8, 2);
        y_table_theta_dot_est(i, j)     = L(8, 3);
        y_dot_table_theta_dot_est(i, j) = L(8, 4);
        z_table_theta_dot_est(i, j)     = L(8, 5);
        z_dot_table_theta_dot_est(i, j) = L(8, 6);
        theta_table_theta_dot_est(i, j) = L(8, 7);
        theta_dot_table_theta_dot_est(i, j) = L(8, 8);
        psi_table_theta_dot_est(i, j)   = L(8, 9);
        psi_dot_table_theta_dot_est(i, j) = L(8, 10);
        
        % Extract values for psi_est
        x_table_psi_est(i, j)     = L(9, 1);
        x_dot_table_psi_est(i, j) = L(9, 2);
        y_table_psi_est(i, j)     = L(9, 3);
        y_dot_table_psi_est(i, j) = L(9, 4);
        z_table_psi_est(i, j)     = L(9, 5);
        z_dot_table_psi_est(i, j) = L(9, 6);
        theta_table_psi_est(i, j) = L(9, 7);
        theta_dot_table_psi_est(i, j) = L(9, 8);
        psi_table_psi_est(i, j)   = L(9, 9);
        psi_dot_table_psi_est(i, j) = L(9, 10);

        % Extract values for psi_dot_est
        x_table_psi_dot_est(i, j)     = L(10, 1);
        x_dot_table_psi_dot_est(i, j) = L(10, 2);
        y_table_psi_dot_est(i, j)     = L(10, 3);
        y_dot_table_psi_dot_est(i, j) = L(10, 4);
        z_table_psi_dot_est(i, j)     = L(10, 5);
        z_dot_table_psi_dot_est(i, j) = L(10, 6);
        theta_table_psi_dot_est(i, j) = L(10, 7);
        theta_dot_table_psi_dot_est(i, j) = L(10, 8);
        psi_table_psi_dot_est(i, j)   = L(10, 9);
        psi_dot_table_psi_dot_est(i, j) = L(10, 10);
        
        % Extract values for dx_est
        x_table_dx_est(i, j)     = L(11, 1);
        x_dot_table_dx_est(i, j) = L(11, 2);
        y_table_dx_est(i, j)     = L(11, 3);
        y_dot_table_dx_est(i, j) = L(11, 4);
        z_table_dx_est(i, j)     = L(11, 5);
        z_dot_table_dx_est(i, j) = L(11, 6);
        theta_table_dx_est(i, j) = L(11, 7);
        theta_dot_table_dx_est(i, j) = L(11, 8);
        psi_table_dx_est(i, j)   = L(11, 9);
        psi_dot_table_dx_est(i, j) = L(11, 10);
        
        % Extract values for dy_est
        x_table_dy_est(i, j)     = L(12, 1);
        x_dot_table_dy_est(i, j) = L(12, 2);
        y_table_dy_est(i, j)     = L(12, 3);
        y_dot_table_dy_est(i, j) = L(12, 4);
        z_table_dy_est(i, j)     = L(12, 5);
        z_dot_table_dy_est(i, j) = L(12, 6);
        theta_table_dy_est(i, j) = L(12, 7);
        theta_dot_table_dy_est(i, j) = L(12, 8);
        psi_table_dy_est(i, j)   = L(12, 9);
        psi_dot_table_dy_est(i, j) = L(12, 10);
        
        % Extract values for dz_est
        x_table_dz_est(i, j)     = L(13, 1);
        x_dot_table_dz_est(i, j) = L(13, 2);
        y_table_dz_est(i, j)     = L(13, 3);
        y_dot_table_dz_est(i, j) = L(13, 4);
        z_table_dz_est(i, j)     = L(13, 5);
        z_dot_table_dz_est(i, j) = L(13, 6);
        theta_table_dz_est(i, j) = L(13, 7);
        theta_dot_table_dz_est(i, j) = L(13, 8);
        psi_table_dz_est(i, j)   = L(13, 9);
        psi_dot_table_dz_est(i, j) = L(13, 10);




    end
end

reference = readtable('reference.csv');
reference_x = reference.x;
reference_xdot = reference.x_dot;
reference_time = reference.t;

% Assign all gain tables to MATLAB base workspace
state_names = {'x', 'x_dot', 'y', 'y_dot', 'z', 'z_dot', ...
               'theta', 'theta_dot', 'psi', 'psi_dot', ...
               'dx', 'dy', 'dz'};

for row = 1:13  % each estimator
    for col = 1:13  % each estimated state
        row_name = state_names{row};
        col_name = state_names{col};
        table_name = sprintf('%s_table_%s_est', col_name, row_name);
        assignin('base', table_name, eval(table_name));
    end
end

disp("✅ All 13×13 observer gain tables assigned to base workspace successfully.");



assignin('base', 'reference_x', reference_x);
assignin('base', 'reference_xdot', reference_xdot);
assignin('base', 'reference_time', reference_time);

disp("All observer gain tables assigned to base workspace successfully.");
