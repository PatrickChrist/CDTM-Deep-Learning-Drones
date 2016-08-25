% Rigid body properties
m = 3;

% Simulation stepsize
dt = 0.1;
% Total sim time
T = 100;
n_steps = T/dt;

% Proportional gain
K_p  = 1;
K_d = 3;
K_i = 0.02;

% Start location
x = 10;
v = 0;

% Input signals
u = 50;


x_values = [];
v_values = [];
t_values = [];
F_values = [];
e_int = 0;

for i=1:n_steps
   t = i*dt;
   
   % Control
   % These two lines are required to control the object
   e_int = e_int + (u-x)*dt;
   u_t = K_p * (u - x) + K_d * (0 - v) + K_i * e_int;
   
   
   % Some more complex actor dynamics
   F_t = min(5 * u_t, 100);
   
   F_t = max(F_t, 0);
   
   % System dynamics, propeller force, gravity and some sinusiod wind
   F_total = F_t - 9.81*m + 4*sin(t) + 3;
   a = F_total / m;
   v = v + (a * dt);
   x = x + (v * dt);
   
   % Store values in order to plot them later
   F_values = [F_values F_total];
   x_values = [x_values x];
   v_values = [v_values v];
   t_values = [t_values t];
end

plot(t_values, x_values, 'b', t_values, v_values, 'r');



