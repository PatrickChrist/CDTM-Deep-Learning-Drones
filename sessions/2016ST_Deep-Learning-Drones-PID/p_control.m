% Rigid body properties
m = 3;

% Simulation stepsize
dt = 0.1;
% Total sim time
T = 100;
n_steps = T/dt;

% Proportional gain
K_p  = 1;


% Start location
x = 10;
dx = 0;
v = 0;
dv = 0;

% Input signals
u = 50;


x_values = [];
v_values = [];
t_values = [];

for i=1:n_steps
   t = i*dt;
   
   % Control
   u_t = K_p * (u - x);
   
   % Actor dynamics
   F_t = 4 * u_t;
   
   % System dynamics
   F_total = F_t - 9.81*m;
   a = F_total / m;
   v = v + (a * dt);
   x = x + (v * dt);
   
   % Store values in order to plot them later
   x_values = [x_values x];
   v_values = [v_values v];
   t_values = [t_values t];
end

plot(t_values, x_values, 'b', t_values, v_values, 'r', t_values, z);



