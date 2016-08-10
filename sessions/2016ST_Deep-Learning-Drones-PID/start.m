% Rigid body properties
m = 3; % in kg

% Simulation stepsize
dt = 0.1; % in seconds
% Total simulation time
T = 100; % in seconds
% Number of simulation steps
n_steps = T/dt;


% Body state variables
x = 10;     % height
v = 0;      % velocity

% Value stores
v_array = [];
x_array = [];
t_array = [];

xprev = 0;
% Start simulation
for i=0:n_steps
    
    % Current time
    t = i * dt;
    
    % Calculate total force on rigid body 
    F_total = -9.81 * m + 50;
    
    % Calculate resulting acceleration
    accel = F_total / m;
    
    % Integrate velocity
     v = v + (accel * dt);
    
    % Integrate position in space
    xprev = x;
    x = x + (v * dt);
    
    % Store values for display
    v_array = vertcat(v_array, v);
    x_array = vertcat(x_array, x);
    t_array = vertcat(t_array, t);
end

% Plot
plot(t_array, v_array, 'r', t_array, x_array, 'b');
legend('Velocity', 'Height');
