function [temperature, pressure, density, sigma] = ATMOS76(altitude,dTemp)
%UNTITLED2 Summary of this function goes here

m2ft = 3.28084;
N2lb = 0.224809;

alt_m = altitude./m2ft; % all units now metric

R = 287;
g0 = 9.8;
P_s = 101325; % N/m^2
rho_s = 1.225; % kg/m^3
T_s = 288.16 + dTemp; % K
a1 = -0.0065;
a2 = 0.003;

alt_vec = 0:500:47000;

for ii = 1:1:23
    Temps(ii) = T_s + (a1.*alt_vec(ii)); % temp in K
    P_vec(ii) = P_s.*(Temps(ii)./T_s).^(-g0./(a1*R)); % pressure in N/m^2
    rho_vec(ii) = rho_s.*(Temps(ii)./T_s).^(-1*(g0./(a1*R)+1)); % density in kg/m^3
    ii = ii + 1;
end
for ii = 24:1:51
    Temps(ii) = 216.66; % temp in K
    P_vec(ii) = P_vec(23).*exp(-(g0./(R.*Temps(ii))).*(alt_vec(ii) - 11000)); % pressure in N/m^2
    rho_vec(ii) = rho_vec(23).*exp(-(g0./(R.*Temps(ii))).*(alt_vec(ii) - 11000)); % density in kg/m^3
    ii = ii + 1;
end
for ii = 52:1:95
    Temps(ii) = 216.66 + (a2.*(alt_vec(ii) - 47000)); % temp in K
    P_vec(ii) = P_vec(51).*(Temps(ii)./T_s).^(-g0./(a2*R)); % pressure in N/m^2
    rho_vec(ii) = rho_vec(51).*(Temps(ii)./T_s).^(-1*(g0./(a1*R)+1)); % density in kg/m^3
    ii = ii + 1;
end

sigma_vec = rho_vec./rho_s;

temperature_m = interp1(alt_vec,Temps,alt_m);
pressure_m = interp1(alt_vec,P_vec,alt_m);
density_m = interp1(alt_vec,rho_vec,alt_m);

temperature = temperature_m - 273.15; % in C
pressure = pressure_m.*(((1/m2ft).^2)./4.44822); % in lb/ft^2
density = density_m.*(((1/m2ft).^3)./14.5939); % in slugs/ft^3
sigma = interp1(alt_vec,sigma_vec,alt_m);