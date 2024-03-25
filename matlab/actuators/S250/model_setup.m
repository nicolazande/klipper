% clean
clear all;
clc;

%% transfer functions
s = tf('s');

%% user parameters
desired_bandwidth = 20; %desired position closed loop bandwidth [Hz]
current_scaling = 50; %30
position_scaling = 1;
mspc = 4; %min number of sample per cycle (>= 2 for Shannon)

%% load (motor side)
Jlm = 0;
Jgb = 1.4e-6; %gearbox inertial motor side [Kg*m2]
gr = 21/1; %gearhead reduction (motor speed / load speed)
grtot = 1000*2*pi*201.6/48; %total gear ratio
Klt = gr/grtot; %gearhead to load ratio [m/rad]
ppl = 0.06; %magnet pole pair length [m]

%% plant (motor)
L = 5e-3; %inductance [H]
R = 4; %resistance [Ohm]
Kf = 12; %force constant [N/A]
Ke = 4; %back-emf constant [V*s/m]
M = 1 * 0.3; %motor mass [Kg]
Fn = 17; %nominal-continuous force [N]
Fmax = 68; %max-acceleration force [N]
Umax = 90; %input voltage [V]
Un = 22; %nominal voltage [V]
Imax = Fmax/Kf; %output current limit [A]
In = Fn/Kf; %nominal-continuous current [A]

wo = 4680*2*pi/60; %no load speed [rad/s]
wn = 4200*2*pi/60; %nominal speed [rad/s]
Io = 0.726; %no load current [A]
tauGp = 0.645e-3; %mechanical time constant [s] 
B = Kf*Io/wo; %Jm/tauGp; %viscous friction [Nm*s/rad]
B0 = 0; %static friction [Nm] --> TODO: tune it according to experiment

%% current controller
Tsi = 4e-5; %current loop sampling time [s]
Gi = 1/(R+s*L); %(B+s*Jm)/((B+s*Jm)*(R+s*L)+Ke*Kt); %current G(s): [V --> A]
scale = evalfr(Gi,0); %get original gain at zero frequency (used for scaling)
poles = pole(Gi);
[~, idx] = max(real(poles));
wcanc = poles(idx); %select slow pole to cancel
tau = 1/abs(wcanc); %slow pole time constant
wc = 2*pi*min(current_scaling*desired_bandwidth, 1/(mspc*Tsi)); %desired closed loop bandwidth
Ki_si = wc/scale; %244; %integral gain [V/(A*s)]
Kp_si = tau*Ki_si; %0.261; %proportional gain [V/A]
Ri = Kp_si + Ki_si/s; %current R(s): [A --> V]
Li = Gi*Ri; %current L(s) [A --> A]
Fi = minreal(Li/(1+Li)); %current F(s) [A --> A]

%% position controller
Tsp = 4e-4;
FF_a_si = 0; %acceleration feed forward gain [A*s/rad]
FF_w_si = 0; %velocity feed forward gain [A*s2/rad]
Gp = Kf/(B+s*M); %Fi*Kf/(B+s*M); %position G(s) [A --> rad]
scale = evalfr(Gp, 0); %get original gain at zero frequency (used for scaling)
poles = pole(Gp);
[~, idx] = max(real(poles));
wcanc = poles(idx); %select slow pole to cancel
tau = 1/abs(wcanc); %slow pole time constant
wc = 2*pi*min(position_scaling*desired_bandwidth, 1/(mspc*Tsp)); %desired closed loop bandwidth (keep 10 times smaller than current one)
Kip_si = 50; %integral gain [A/(rad*s)]
Kpp_si = 2.9; %proportional gain [A/rad]
Kdp_si = 0.075; %0.065; %derivative gain [A*s/rad] (0.11*Kup*Tup)
Rp = Kpp_si + Kip_si/s + s*Kdp_si/(Kdp_si/(10*Kpp_si)*s + 1); %position: R(s) [rad --> A]
Lp = Rp*Gp; %position L(s): [rad --> rad]
Fp = minreal(Lp/(1+Lp)); %position F(s): [rad --> rad]

%% path planner
Tspp = 0.01;
max_ppa_si = 275000*2*pi/60; %(Kt*Imax)/(Jm+Jgb+Jlm); %max acceleration [rad/s^2]
max_ppv_si = wo*(Umax/Un); %no load speed at selected voltage
Kppph_si = (max_ppa_si/max_ppv_si); 
Kpppl_si = Kppph_si;
Hpp = 12.5; %depends on acceleration


