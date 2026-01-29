function [advance, RPM, Thrust, eff, CT, CP] = Propeller_Mejzlik32x18_Pwr(airspeed, rho, Power)
% Sept 4 2024
% current prop curves are based on data from Mejzlik datasheet, assumed
% static thrust data; extrapolated to forward airspeeds by Athule

D = 32/12; % prop diameter
V = airspeed;

% CT = Thrust / (rho*(n^2)*(d^4))
% thrust curve coefficients
A2 = -0.1050;
A1 = -0.0183;
A0 = 0.0972;

% CP = Power / (rho*(n^3)*(d^5))
% power curve coefficients
P3 = 0.0620;
P2 = -0.1950;
P1 = 0.0980;
P0 = 0.0350;

revs = [0:10:150];
adv_vec = V./(revs.*D);
% CT_vec = thrust./(rho.*(revs.^2).*(D^4));
CP_vec = Power./(rho.*(revs.^3).*(D^5));
J_varied = [0:0.01:1.2];
CT_curve = A2*(J_varied).^2 + A1*(J_varied) + A0;
CP_curve = P3*(J_varied).^3 + P2*(J_varied).^2 + P1*(J_varied) + P0;

% CT = A2*J^2 + A1*J + A0
% CT = A2*(V/nD)^2 + A1*(V/nD) + A0


myfun = @(n) (P3*(V./(n.*D)).^3 + P2*(V./(n.*D)).^2 + P1*(V./(n.*D)) + P0) - (Power./(rho.*(n.^3).*(D.^5)));

n = fzero(myfun,70);

advance = V./(n.*D);
RPM = n*60;

CT = (A2.*(advance).^2 + A1.*(advance) + A0);
CP = (P3.*(advance).^3 + P2.*(advance).^2 + P1.*(advance) + P0);
Thrust = CT.*(rho.*(n.^2).*(D.^4));

eff = advance.*(CT./CP);


end