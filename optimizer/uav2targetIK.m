clear;
clc;



ub = [1.571, 1.57, 0.1745, 0.1, 500, 1, 3, 2];
% ub = [1.571, 1.57, 0.0, 0.1, 500, 1, 3, 3];
lb = [-0.5, 0.0, 0.0, 0, 100, 0, 0.5, 0.5];

options = optimoptions(@fmincon,...
    'Display','iter','Algorithm','interior-point', 'MaxFunctionEvaluations', 3000, 'MaxIterations', 1000);
% options = optimoptions(@fmincon,...
%     'Display','iter','Algorithm','sqp');

% [x, fval] = fmincon(@objfcn,[1.222 1.0472, 0.0873, 0, 455, 0.67, 1.0, 0.6],[],[],[],[],lb,ub,@objfcnconstraint,options)
[x, fval] = fmincon(@objfcn,[1.222 0.8727, 0.0873, 0, 312, 0.47, 1.5, 0.6],[],[],[],[],lb,ub,@objfcnconstraint,options)

% Intrinsic params
len_prop = 0.127;
len_arm = 0.17;
h_ce = 1.2;
len_l = 0.09;
len_e = 0.12;
len_ul = 0.20;


% % Optimization variables % %
% x(1) = gamma
% x(2) = alpha
% x(3) = theta
% x(4) = nozzle_len
% x(5) = spring constant
% x(6) = friction coeff
% x(7) = thrust force
% x(8) = norm force

% % % 
% Remove the len_sc, len_l from variables
% % %


gamma = rad2deg(x(1))
angle = rad2deg(x(2))
theta = rad2deg(x(3))
len_nozz = x(4)
ks = x(5)
miu = x(6)
F_t = x(7)
F_norm = x(8)


xob = (h_ce-len_e)/tan(gamma)
zob = h_ce-len_e

xbn = x(4)*sin(x(2)-x(3));
zbn = x(4)*cos(x(2)-x(3));

zpoc = 0;
xpoc = 2;

zpos = zob - zbn - zpoc
xpos = xpoc - (xob + xbn)


t = linspace(0,10,1000);

% x0 = [0; zpos; 16.8*sin(deg2rad(angle-theta));-16.8*cos(deg2rad(angle-theta))];     % To check the fluid flow
x0 = [0; zpos; 3.5764*sin(deg2rad(angle-theta)); -3.5764*cos(deg2rad(angle-theta))];     % To check the fluid flow
sol = ode45(@traj,t,x0);
zerofnd = fzero(@(r)deval(sol,r,2),[sol.x(1),sol.x(end)]);
xpos_backwards = deval(sol,zerofnd,1)


% % Sub-objective functions
alpha0 = 1.60;
alpha1 = 0;
R = 0.127;
rho = 1.293;
A = pi*R^2;
vi = 5.18;
mg = 2.0*9.81;
theta = x(3) * pi/180;
alpha = x(2) * pi/180;
n = 4;

len_springMax = 0.12;
delta_spring = 0.03;
d_ce = len_springMax;

% % Loaded
delta = R/(d_ce);
gamma = 0.5*(1-(alpha1*(delta^2))) + 0.5*((1-(alpha1*(delta^2)))^2+(alpha0/8*(delta^2)))^0.5;


% Fjetx = 0.901613;
% Fjetz = 0.637527;
Fjetx = 0.1366*sin(alpha-theta);
Fjetz = 0.1366*cos(alpha-theta);


% %  R matrix for body-spring.
R_b = eye(3,3);
R_b(1,1) = cos(theta);
R_b(1,3) = sin(theta);
R_b(3,1) = -sin(theta);
R_b(3,3) = cos(theta);

% % R matrix for body-jet.
R_bj = zeros(3,3);
R_bj(1,1) = 1;
R_bj(2,2) = cos(alpha-theta);
R_bj(2,3) = -sin(alpha-theta);
R_bj(3,2) = sin(alpha-theta);
R_bj(3,3) = cos(alpha-theta);

% % Selecting indi components of mat
x_comp = [1, 0, 0];
y_comp = [0, 1, 0];
z_comp = [0, 0, 1];

% %  Thrust mat
Fthrust_mat = zeros(3,1);
Fthrust_mat(3,1) = x(7)*9.81;

% %  Ceiling effect mat
Fce_mat = zeros(3,1);
Fce_mat(3,1) = (2*rho*A*(gamma^2)*(vi^2));

% % Spring mat
Fsp_mat = zeros(3,1);
Fsp_mat(3,1) = x(5)*delta_spring;

% % Jet mat
Fjet_mat = zeros(3,1);
Fjet_mat(1,1) = Fjetx;
Fjet_mat(3,1) = Fjetz;

% % Weight mat
mg_mat = zeros(3,1);
mg_mat(3,1) = mg;

% % Friction mat 
Ffric_mat = zeros(3,1);
Ffric_mat(1,1) = x(6)*x(8);
Ffric_mat(2,1) = x(6)*x(8);


% % Sum of Forces
sumF_thrust = z_comp*(Fthrust_mat)
sumF_ce = 4*z_comp*(Fce_mat)
sumF_jet = z_comp*(R_bj*Fjet_mat)
sumF_spring = z_comp*(R_b*Fsp_mat)
sumF_mg = z_comp*(mg_mat)
sumF_fric = z_comp*(R_b*Ffric_mat)

SUMofF = z_comp*(Fthrust_mat) + 4*z_comp*(Fce_mat) + z_comp*(R_bj*Fjet_mat) - 2*z_comp*(R_b*Fsp_mat) - z_comp*(mg_mat) - z_comp*(R_b*Ffric_mat)
% SUMofF = z_comp*(Fthrust_mat) + 4*z_comp*(Fce_mat) + z_comp*(R_bj*Fjet_mat) - 4*z_comp*(R_b*Fsp_mat) - z_comp*(mg_mat) - z_comp*(R_b*Ffric_mat)


% %  Sum of Moments_pitch
sumM_thrust = z_comp*(Fthrust_mat)*(len_arm*cos(theta))
sumM_ce = 4*z_comp*(Fce_mat)*(len_arm*cos(theta))
sumM_jet = z_comp*(R_bj*Fjet_mat)*(len_arm*cos(theta))
sumM_spring = 2*z_comp*(R_b*Fsp_mat)*(len_arm*cos(theta))
sumM_mg = z_comp*(mg_mat);
sumM_fric = z_comp*(R_b*Ffric_mat)*(len_arm*cos(theta))

SUMofM = z_comp*(Fthrust_mat)*(len_arm*cos(theta)) + 4*z_comp*(Fce_mat)*(len_arm*cos(theta)) + z_comp*(R_bj*Fjet_mat)*(len_arm*cos(theta)) - 2*z_comp*(R_b*Fsp_mat)*(len_arm*cos(theta)) - z_comp*(R_b*Ffric_mat)*(len_arm*cos(theta))



% % Sending values out to ros network
rosinit("172.22.41.222");

pub1 = rospublisher("/thrust", "std_msgs/Float32");
pub2 = rospublisher("/theta", "std_msgs/Float32");
pub3 = rospublisher("/desired_pos", "geometry_msgs/PoseStamped");

msg1 = rosmessage(pub1);
msg2 = rosmessage(pub2);
msg3 = rosmessage(pub3);


F_t = evalin('base', 'F_t');
theta = evalin('base', 'theta');
xob = evalin('base', 'xob');
zob = evalin('base', 'zob');


msg1.Data = F_t;
msg2.Data = theta;
msg3.Pose.Position.X = xob;
msg3.Pose.Position.Z = zob;


send(pub1, msg1);
send(pub2, msg2);
send(pub3, msg3);


pause(1);

% rosshutdown;







% plot(sol.x, sol.y); legend('x','z','xdot','zdot')



x_coord = [0,xob, xob+xbn, xob+xbn+xpos];
z_coord = [0,zob, zob-zbn, zpoc];
figure(1)
c = linspace(1, 0.5, length(x));
scatter(0, 0, 150 , 'filled','MarkerEdgeColor',[0.1 0.5 0.1], 'MarkerFaceColor','k'); xlabel("X(m)", 'FontSize', 35), ylabel("Z(m)", 'FontSize', 20)
hold on
xlim([0 4]);
ylim([0 5])
scatter(xob, zob, 150 , 'filled','MarkerEdgeColor',[0.1 0.5 0.1], 'MarkerFaceColor','g');
% scatter(xob+xbs, zob-zbs, 150 , 'filled','MarkerEdgeColor',[0.1 0.5 0.1],'MarkerFaceColor','b');
scatter(xob + xbn, zob-zbn, 150 , 'filled','MarkerEdgeColor',[0.1 0.5 0.1], 'MarkerFaceColor','r');
scatter(xob + xbn + xpos, zpoc, 150 , 'filled','MarkerEdgeColor',[0.1 0.5 0.1], 'MarkerFaceColor','b');
legend('P_o','P_b','P_n','P_{poc}')
plot(x_coord, z_coord, 'k', 'HandleVisibility','off')
set(gca,'FontSize',20, 'FontName', 'Times New Roman');
axis square

pause(1)


