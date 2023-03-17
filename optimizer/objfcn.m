function f = objfcn(x)


% Intrinsic params
len_prop = 0.127;
len_arm = 0.17;
h_ce = 1.2;
len_l = 0.08;
len_e = 0.0795;
len_ul = 0.109;


gamma = rad2deg(x(1))
angle = rad2deg(x(2))
theta = rad2deg(x(3))
len_nozz = x(4)



xob = (h_ce-len_e)/tan(gamma)
zob = h_ce-len_e

xbn = x(4)*sin(x(2)-x(3));
zbn = x(4)*cos(x(2)-x(3));

zpoc = 0;
xpoc = 2;

zpos = zob - zbn - zpoc
xpos = xpoc - (xob + xbn)


% % Sub-objective functions
alpha0 = 1.60;
alpha1 = 0;
R = 0.127;
rho = 1.293;
A = pi*R^2;
vi = 5.18;
mg = 1.5*9.81;
theta = x(3) * pi/180;
alpha = x(2) * pi/180;
n = 4;

len_springMax = 0.12;
delta_spring = 0.03;
d_ce = len_springMax;

% % Loaded
delta = R/(d_ce);
gamma = 0.5*(1-(alpha1*(delta^2))) + 0.5*((1-(alpha1*(delta^2)))^2+(alpha0/8*(delta^2)))^0.5;


Fjetx = 0.901613;
Fjetz = 0.637527;


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

% SUMofF = z_comp*(Fthrust_mat) + 4*z_comp*(Fce_mat) + z_comp*(R_bj*Fjet_mat) - 2*z_comp*(R_b*Fsp_mat) - z_comp*(mg_mat) - z_comp*(R_b*Ffric_mat)
SUMofF = z_comp*(Fthrust_mat) + 4*z_comp*(Fce_mat) + z_comp*(R_bj*Fjet_mat) - 4*z_comp*(R_b*Fsp_mat) - z_comp*(mg_mat) - z_comp*(R_b*Ffric_mat)


% %  Sum of Moments_pitch
sumM_thrust = z_comp*(Fthrust_mat)*(len_arm*cos(theta))
sumM_ce = 4*z_comp*(Fce_mat)*(len_arm*cos(theta))
sumM_jet = z_comp*(R_bj*Fjet_mat)*(len_arm*cos(theta))
sumM_spring = 2*z_comp*(R_b*Fsp_mat)*(len_arm*cos(theta))
sumM_mg = z_comp*(mg_mat);
sumM_fric = z_comp*(R_b*Ffric_mat)*(len_arm*cos(theta))

SUMofM = z_comp*(Fthrust_mat)*(len_arm*cos(theta)) + 4*z_comp*(Fce_mat)*(len_arm*cos(theta)) + z_comp*(R_bj*Fjet_mat)*(len_arm*cos(theta)) - 2*z_comp*(R_b*Fsp_mat)*(len_arm*cos(theta)) - z_comp*(R_b*Ffric_mat)*(len_arm*cos(theta))


% Q = 1;
% R = 0.174;
% S = 0.15;

Q = 1;
R = 1;
S = 0;


f = Q*norm(zpos)+ R*norm(xpos) + S*SUMofF; %Q penalises Z position, while R penalises X position. 










