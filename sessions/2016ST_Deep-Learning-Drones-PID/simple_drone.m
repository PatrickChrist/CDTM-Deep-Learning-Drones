% Rigid body properties
m = 3;

% Simulation stepsize
dt = 0.1;
% Total sim time
T = 100;
n_steps = T/dt;


% Body state variables

h = 10;     % height
v = 0;      % velocity

% Value store
v_array = [];
h_array = [];
t_array = [];

% Start simulation
for i=0:n_steps
    
    % Propeller force (upwards)
    F_propeller = 100;
    % Calculate total force on rigid body 
    F_total = -9.81 * m + F_propeller;
    
    % Calculate resulting acceleration
    accel = F_total / m;
    
    % Integrate velocity
    v = v + (accel * dt);
    
    % Integrate position in space
    h = h + (v * dt);
    
    v_array = vertcat(v_array, v);
    h_array = vertcat(h_array, h);
    
    t = i * dt;
    t_array = vertcat(t_array, t);
end

plot(t_array, v_array, 'g', t_array, h_array, 'r');
