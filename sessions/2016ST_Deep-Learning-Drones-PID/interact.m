clf;

% Rigid body properties
m = 3;

% Simulation stepsize
dt = 0.1;
% Maximum total sim time in seconds
T = 1800;
n_steps = T/dt;

% Proportional gain
K_p  = 1;
K_d = 3;
K_i = 0.02;

% Start location
x = 10;
dx = 0;
v = 0;
dv = 0;
w = 0;

% Target height
z = 50;

x_values = [];
v_values = [];
t_values = [];
e_int = 0;


slmin = 0;
slmax = 150;
hsl = uicontrol('Style','slider','Min',slmin,'Max',slmax,...
                'SliderStep',[1 1]./(slmax-slmin),'Value',z,...
                'Position',[150 5 200 20]);
set(hsl,'Callback',@(hObject,eventdata) assignin('base', 'z', get(hObject,'Value')))

sl = uicontrol('Style', 'text',...
   'String', sprintf('Target height: %f', z),... %replace something with the text you want
   'Position', [10 40 50 50]); 

for i=1:n_steps
   t = i*dt;
   u = z;
   
   % Control
   e_int = e_int + (u-x)*dt;
   u_t = K_p * (u - x) + K_d * (w - v) + K_i*e_int;
   
   % Actor dynamics
   F_t = min(5 * u_t, 100);
   
   F_t = max(F_t, 0);
   
   % System dynamics
   F_total = F_t - 9.81*m + 4*sin(t) + 3;
   a = F_total / m;
   v = v + (a * dt);
   x = x + (v * dt);
   
   % Store values in order to plot them later
   x_values = vertcat(x_values, x);
   v_values = vertcat(v_values, v);
   t_values = vertcat(t_values, t);
   
   % Render
   hold on;
   h = plot(t_values, x_values, 'b', t_values, v_values, 'r');
   if i==1
       legend('Height','Velocity', 'Location','north')
   end
   
   set(sl,'string',sprintf('Target height: %f', z));
   
   drawnow;
   % 10 times spedup
   pause(dt / 10);
end







