% Rigid body properties
m = 3;

% Simulation stepsize
dt = 0.1;
% Total simulation time
T = 100;
n_steps = T/dt;


% Body state variables
x = 10;     % height
v = 0;      % velocity

% Value stores
v_array = [];
h_array = [];
t_array = [];

% Start simulation
for i=0:n_steps
    % Current time
    t = i * dt;
    
    % Calculate total force on rigid body 
    F_total = -9.81 * m;
    
    % Calculate resulting acceleration
    accel = F_total / m;
    
    % Integrate velocity
    v = v + (accel * dt);
    
    % Integrate position in space
    x = x + (v * dt);
    
    % Store values for display
    v_array = vertcat(v_array, v);
    h_array = vertcat(h_array, x);
    t_array = vertcat(t_array, t);
end

% Render
plot(t_array, v_array, 'g', t_array, h_array, 'r');
