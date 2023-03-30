syms theta_t alpha_t d r;

M = [cos(theta_t), -sin(theta_t)*cos(alpha_t), sin(theta_t)*sin(alpha_t), r*cos(theta_t);
    sin(theta_t), cos(theta_t)*cos(alpha_t), -cos(theta_t)*sin(alpha_t), r*sin(theta_t);
    0,sin(alpha_t),cos(alpha_t),d;
    0,0,0,1];


f(theta_t, alpha_t, r, d)  = M;

syms theta_1 alpha_1 theta_2 alpha_2 d_1 d_2 r_1 r_2;

% f(theta_1, alpha_1, d_1,r_1)*f(theta_2, alpha_2, d_2,r_2)


f(theta_1, pi/2, 0,0)*f(0.35, alpha_2, 0,64)