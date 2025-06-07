clear;
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

%Tx
x_table_Tx = zeros(mass_num,thrust_num);
x_dot_table_Tx = zeros(mass_num,thrust_num);
y_table_Tx = zeros(mass_num,thrust_num);
y_dot_table_Tx = zeros(mass_num,thrust_num);
z_table_Tx = zeros(mass_num,thrust_num);
z_dot_table_Tx = zeros(mass_num,thrust_num);
theta_table_Tx = zeros(mass_num,thrust_num);
theta_dot_table_Tx = zeros(mass_num,thrust_num);
psi_table_Tx = zeros(mass_num,thrust_num);
psi_dot_table_Tx = zeros(mass_num,thrust_num);
%Ty
x_table_Ty = zeros(mass_num,thrust_num);
x_dot_table_Ty = zeros(mass_num,thrust_num);
y_table_Ty = zeros(mass_num,thrust_num);
y_dot_table_Ty = zeros(mass_num,thrust_num);
z_table_Ty = zeros(mass_num,thrust_num);
z_dot_table_Ty = zeros(mass_num,thrust_num);
theta_table_Ty = zeros(mass_num,thrust_num);
theta_dot_table_Ty = zeros(mass_num,thrust_num);
psi_table_Ty = zeros(mass_num,thrust_num);
psi_dot_table_Ty = zeros(mass_num,thrust_num);
%Tz
x_table_Tz = zeros(mass_num,thrust_num);
x_dot_table_Tz = zeros(mass_num,thrust_num);
y_table_Tz = zeros(mass_num,thrust_num);
y_dot_table_Tz = zeros(mass_num,thrust_num);
z_table_Tz = zeros(mass_num,thrust_num);
z_dot_table_Tz = zeros(mass_num,thrust_num);
theta_table_Tz = zeros(mass_num,thrust_num);
theta_dot_table_Tz = zeros(mass_num,thrust_num);
psi_table_Tz = zeros(mass_num,thrust_num);
psi_dot_table_Tz = zeros(mass_num,thrust_num);

%% Contruct Gain table 
% Define Q and R for LQR
Q = diag(1e5*[9.0411    7.4568    7.9289    9.8359    7.9289    9.8359    8.1396    7.9781    8.1396    7.9781]);
R = diag([1 1 1]);

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
        sysc = ss(A_c,B_c,C_c,D_c);
        sysd = c2d(sysc,ts,'zoh');

        A_d = sysd.A;
        B_d = sysd.B;
        C_d = sysd.C;
        D_d = sysd.D;

        [K,~, closed_loop_poles] = dlqr(A_d, B_d, Q, R);
        
        %G = ss(A_d-B_d*K, B_d, eye(10), zeros(10, 3), ts);

        % Convert K to a static gain system: maps x -> u
        %sysK = ss([], [], [], K, ts);

        % Form the loop transfer function: L = G then K, which is K*(zI-A_d)^{-1}B_d
        %L_sys = series(G, sysK);  % L_sys is now 3x3 (square)

        % Compute disk margin for the loop system L_sys
        %[~, info] = diskmargin(L_sys);

        % Compute an effective gain margin as the geometric mean of the lower and upper bounds
        % Gm_eff = info.LowerBound;
        % fprintf("m: %.2f kg T: %.2f N\n", m, T);
        % fprintf("gm: %.2f dB\n", 20*log10(Gm_eff));
        % fprintf("pm: %.2f degs\n", info.PhaseMargin(2));


        % Extract values for Tx
        x_table_Tx(i, j) = K(1, 1);
        x_dot_table_Tx(i, j) = K(1, 2);
        y_table_Tx(i, j) = K(1, 3);
        y_dot_table_Tx(i, j) = K(1, 4);
        z_table_Tx(i, j) = K(1, 5);
        z_dot_table_Tx(i, j) = K(1, 6);
        theta_table_Tx(i, j) = K(1, 7);
        theta_dot_table_Tx(i, j) = K(1, 8);
        psi_table_Tx(i, j) = K(1, 9);
        psi_dot_table_Tx(i, j) = K(1, 10);

        % Extract values for Ty
        x_table_Ty(i, j) = K(2, 1);
        x_dot_table_Ty(i, j) = K(2, 2);
        y_table_Ty(i, j) = K(2, 3);
        y_dot_table_Ty(i, j) = K(2, 4);
        z_table_Ty(i, j) = K(2, 5);
        z_dot_table_Ty(i, j) = K(2, 6);
        theta_table_Ty(i, j) = K(2, 7);
        theta_dot_table_Ty(i, j) = K(2, 8);
        psi_table_Ty(i, j) = K(2, 9);
        psi_dot_table_Ty(i, j) = K(2, 10);

        % Extract values for Tz
        x_table_Tz(i, j) = K(3, 1);
        x_dot_table_Tz(i, j) = K(3, 2);
        y_table_Tz(i, j) = K(3, 3);
        y_dot_table_Tz(i, j) = K(3, 4);
        z_table_Tz(i, j) = K(3, 5);
        z_dot_table_Tz(i, j) = K(3, 6);
        theta_table_Tz(i, j) = K(3, 7);
        theta_dot_table_Tz(i, j) = K(3, 8);
        psi_table_Tz(i, j) = K(3, 9);
        psi_dot_table_Tz(i, j) = K(3, 10);

    end
end

reference = readtable('reference.csv');
reference_x = reference.x;
reference_xdot = reference.x_dot;
reference_time = reference.t;
