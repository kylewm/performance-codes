function [advance, RPM, Power, eff, CT, CP] = Propeller_Mejzlik32x18(airspeed, rho, thrust)
% Sept 4 2024
% current prop curves are based on data from Mejzlik datasheet, assumed
% static thrust data; extrapolated to forward airspeeds by Athule

D = 32/12; % prop diameter, ft
V = airspeed; % ft/s

% CT = Thrust / (rho*(n^2)*(d^4))
% thrust curve coefficients
A2 = -0.11;
A1 = 0;
A0 = 0.0838;

% CP = Power / (rho*(n^3)*(d^5))
% power curve coefficients
P3 = 0.0620;
P2 = -0.1950;
P1 = 0.1000;
P0 = 0.0435;

revs = [0:10:150];
adv_vec = V./(revs.*D);
CT_vec = thrust./(rho.*(revs.^2).*(D^4));
J_varied = [0:0.01:1.2];
% CT_curve = A2*(J_varied).^2 + A1*(J_varied) + A0;
% CP_curve = P3*(J_varied).^3 + P2*(J_varied).^2 + P1*(J_varied) + P0;


% CT = A2*J^2 + A1*J + A0
% CT = A2*(V/nD)^2 + A1*(V/nD) + A0

myfun = @(n) (A2*(V./(n.*D)).^2 + A1*(V./(n.*D)) + A0) - (thrust/(rho.*(n^2).*(D^4)));

n = fzero(myfun,50);

advance = V./(n.*D);
RPM = n*60;

CT = (A2.*(advance)^2 + A1.*(advance) + A0);
CP = (P3.*(advance).^3 + P2.*(advance).^2 + P1.*(advance) + P0);
Power = CP.*(rho.*(n.^3)*(D.^5));

eff = advance*(CT/CP);


end
