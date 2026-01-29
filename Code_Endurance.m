clear all; close all; clc;

ft2kts = 0.592484;
W2ftlb = 550/746; % 550 ft-lb/s = 746 W
W2hp = 1/746; % 1 hp = 746 W
ftlb2hp = 1/550;
ft2nm = 1/6076.12;

colors = [0 0.4470 0.7410;
         0.8500 0.3250 0.0980;
         0.9290 0.6940 0.1250;
         0.4940 0.1840 0.5560;
         0.4660 0.6740 0.1880;
         0.3010 0.7450 0.9330;
         0.6350 0.0780 0.1840];


%% Aircraft Geometry

load('TrimPolar.mat')

Sref = TrimPolar.Sref(1)./144;
cref = TrimPolar.Cref(1)./12;
bref = TrimPolar.Bref(1)./12;
RefQ = [Sref, bref, cref];

CL_stall = TrimPolar.CLtot(21);
CL_max = TrimPolar.CLtot(23);
stall_idx = 21;

% NW230 Engine
% maxPower_W = 11600; % Watts
% SFC = 0.72; % lb/hp/hr
% maxPower_W = 10000; % Watts
% SFC = 0.58; % lb/hp/hr

% maxPower = maxPower_W*W2ftlb;
% internalLosses = maxPower.*0.13; % assumes 15% of power generated is lost due to internal efficiencies

Power_electric_W = 500; % Watts, base avionics + AC-14
Power_electric = Power_electric_W*W2ftlb; % conversion from Watts to ft-lb/s
spinningprops = 2; % number of propellers used in forward flight

%%


%% Describe Mission Profile

% establish starting weights
MGTOW = 330; % lbs
EmptyWeight = 210; % lbs
Payload = 65; % lbs, Alticam AC-14 plus extra goodies
DryWeight = EmptyWeight + Payload;
Fuel = MGTOW - (DryWeight); % lbs

launchAlt = 1000; % launch altitude, ft
launchDist = 10000; % feet for launch pattern 
missionAlt = 8000; % ft, mission altitude

loiterTime = 13; % hours, time on station
setAirspeed = 65; % knots EAS


%%


%% Launch Phase

timeIdx = 1;
timeIncrement = 30; % time step in seconds
Telemetry.timeStamp(timeIdx) = 0;
Telemetry.Altitude(timeIdx) = launchAlt; % ft
Telemetry.groundCover(timeIdx) = 0;
RPMset = 5250;

[~, ~, Telemetry.density(timeIdx), Telemetry.sigma(timeIdx)] = ATMOS76(Telemetry.Altitude(timeIdx),0);

Telemetry.Weight(timeIdx) = MGTOW; % lbs
Telemetry.FuelLoad(timeIdx) = Fuel; % lbs
Telemetry.FuelBurn(timeIdx) = 0; % lb/s
Telemetry.CL_req(timeIdx) = 0;
Telemetry.CD_gen(timeIdx) = 0;
Telemetry.Thrust_req(timeIdx) = 0;  % lbs
Telemetry.TotalPowerReq(timeIdx) = 0; % ft-lb/s
Telemetry.TotalPowerReq_hp(timeIdx) = 0; % hp

% this returns the airspeeds for best range and best endurance given an aircraft weight and altitude
[BestE,BestR] = FindBestAirspeeds(Telemetry.Weight(timeIdx),Telemetry.density(timeIdx),Telemetry.sigma(timeIdx),TrimPolar.CLtot,TrimPolar.CDtot,Sref);

% setting our starting airspeeds to the best endurance speed
% Telemetry.KEAS(timeIdx) = BestE.V_KEAS;
% Telemetry.KTAS(timeIdx) = BestE.V_KTAS;
% Telemetry.Vfps(timeIdx) = BestE.V_fps_TAS;

% setting our starting airspeeds to our desired speed
Telemetry.KEAS(timeIdx) = setAirspeed;
Telemetry.KTAS(timeIdx) = Telemetry.KEAS(timeIdx)./sqrt(Telemetry.sigma(timeIdx));
Telemetry.Vfps(timeIdx) = Telemetry.KTAS(timeIdx)./ft2kts;

% Telemetry.MaxPower(timeIdx) = (maxPower + internalLosses).*Telemetry.sigma(timeIdx) - internalLosses; % max power available for propulsion + electrical loads, derated for altitude (ft-lb/s)

timeIdx = 2;

% launch pattern loop, continues until desired ground cover distance is achieved
while Telemetry.groundCover < launchDist

    % Telemetry.MaxPower(timeIdx) = (maxPower + internalLosses).*Telemetry.sigma(timeIdx-1) - internalLosses; % max power available out of the engine, derated for altitude (ft-lb/s)

    % calculating lift coefficient, drag coefficient, and resultant required total thrust for the current weight/altitude/airspeed
    Telemetry.CL_req(timeIdx) = Telemetry.Weight(timeIdx-1)./(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref);
    Telemetry.CD_gen(timeIdx) = interp1(TrimPolar.CLtot(1:stall_idx),TrimPolar.CDtot(1:stall_idx),Telemetry.CL_req(timeIdx));
    Telemetry.Thrust_req(timeIdx) = Telemetry.CD_gen(timeIdx).*(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref); % lbs

    % calculating the thrust required from EACH propeller (since there are multiple)
    Telemetry.PropThrust(timeIdx) = Telemetry.Thrust_req(timeIdx)./spinningprops; % dividing required thrust by number of props
    % this function calculates the matching RPM and required power consumption for the propeller operating at the provided airspeed/altitude/thrust, using constructed coefficient curves
    [Telemetry.AdvanceJ(timeIdx), Telemetry.RPM(timeIdx), Telemetry.Power(timeIdx), Telemetry.Eta(timeIdx), Telemetry.CT(timeIdx), Telemetry.CP(timeIdx)] = Propeller_Mejzlik32x18(Telemetry.Vfps(timeIdx-1), Telemetry.density(timeIdx-1), Telemetry.PropThrust(timeIdx));
    
    % calculating the total power required by multiplying the individual propeller power consumption by the number of props, and then adding in the electrical power draw
    Telemetry.TotalPowerReq(timeIdx) = Telemetry.Power(timeIdx).*spinningprops + Power_electric; % total power in ft-lb/s
    Telemetry.TotalPowerReq_hp(timeIdx) = Telemetry.TotalPowerReq(timeIdx).*ftlb2hp; % total power in hp

    % calculating fuel burn by finding throttle for required power and outputting associated SFC
    [Telemetry.Torque(timeIdx), Telemetry.BSFC(timeIdx)] = Engine_NW230(Telemetry.sigma(timeIdx-1),RPMset,Telemetry.TotalPowerReq_hp(timeIdx)./W2hp); % BSFC in lb/hp/hr

    Telemetry.FuelBurn(timeIdx) = Telemetry.BSFC(timeIdx).*Telemetry.TotalPowerReq_hp(timeIdx); % fuel burn in lbs/hr
    Telemetry.FuelLoad(timeIdx) = Telemetry.FuelLoad(timeIdx-1) - (Telemetry.FuelBurn(timeIdx)./3600)*timeIncrement; % calculating fuel burn in lb/s and multiplying by flight time increment
    Telemetry.Weight(timeIdx) = Telemetry.FuelLoad(timeIdx) + DryWeight;

    % we now go through and update all our flight conditions since we know the change in weight and how far the aircraft must have traveled
    Telemetry.Altitude(timeIdx) = Telemetry.Altitude(timeIdx-1);
    [~, ~, Telemetry.density(timeIdx), Telemetry.sigma(timeIdx)] = ATMOS76(Telemetry.Altitude(timeIdx),0);

    % [BestE,BestR] = FindBestAirspeeds(Telemetry.Weight(timeIdx),Telemetry.density(timeIdx),Telemetry.sigma(timeIdx),TrimPolar.CLtot,TrimPolar.CDtot,Sref);
    % Telemetry.KEAS(timeIdx) = BestE.V_KEAS;
    % Telemetry.KTAS(timeIdx) = BestE.V_KTAS;
    % Telemetry.Vfps(timeIdx) = BestE.V_fps_TAS;

    Telemetry.groundCover(timeIdx) = (Telemetry.Vfps(timeIdx-1)).*timeIncrement + Telemetry.groundCover(timeIdx-1); % distance covered, ft
    Telemetry.timeStamp(timeIdx) = timeIncrement + Telemetry.timeStamp(timeIdx-1);
    Telemetry.KEAS(timeIdx) = Telemetry.KEAS(timeIdx-1);
    Telemetry.KTAS(timeIdx) = Telemetry.KEAS(timeIdx)./sqrt(Telemetry.sigma(timeIdx));
    Telemetry.Vfps(timeIdx) = Telemetry.KTAS(timeIdx)./ft2kts;

    timeIdx = timeIdx+1;
end

%%


%% Climb to Mission Alt

while Telemetry.Altitude < missionAlt

    % max power available out of the engine, derated for altitude (ft-lb/s)
    [Telemetry.MaxPower_W(timeIdx), Telemetry.Torque(timeIdx), Telemetry.BSFC(timeIdx)] = Engine_NW230_MaxP(Telemetry.sigma(timeIdx-1),RPMset);
    % to do this right, we need to calculate the maximum amount of power that can be provided to each propeller by taking the maximum engine output power, pulling out the electrical draw, and then dividing this by the number of spinning props
    Telemetry.MaxPower_Prop(timeIdx) = (Telemetry.MaxPower_W(timeIdx).*W2ftlb - Power_electric)./spinningprops;
    Telemetry.TotalPowerReq(timeIdx) = Telemetry.MaxPower_W(timeIdx).*W2ftlb;
    Telemetry.TotalPowerReq_hp(timeIdx) = Telemetry.MaxPower_W(timeIdx).*W2hp;

    % this function calculates the matching RPM and output thrust for the propeller operating at the provided airspeed/altitude and consuming the prescribed amount of power, using constructured coefficient curves
    [Telemetry.AdvanceJ(timeIdx), Telemetry.RPM(timeIdx), Telemetry.PropThrust(timeIdx), Telemetry.Eta(timeIdx), Telemetry.CT(timeIdx), Telemetry.CP(timeIdx)] = Propeller_Mejzlik32x18_Pwr(Telemetry.Vfps(timeIdx-1), Telemetry.density(timeIdx-1), Telemetry.MaxPower_Prop(timeIdx));
    Telemetry.Thrust_max(timeIdx) = Telemetry.PropThrust(timeIdx).*spinningprops; % multiplying generated thrust by number of props
    % calculating the maximum PHYSICAL PROPULSIVE power available from the prop by using our max electrical power and our calculated prop efficiency at this specific point in time
    Telemetry.MaxPower_Avail(timeIdx) = (Telemetry.MaxPower_Prop(timeIdx).*spinningprops).*Telemetry.Eta(timeIdx);

    % calculating lift coefficient, drag coefficient, and resultant required total thrust for flight at the current weight/altitude/airspeed
    Telemetry.CL_req(timeIdx) = Telemetry.Weight(timeIdx-1)./(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref);
    Telemetry.CD_gen(timeIdx) = interp1(TrimPolar.CLtot(1:stall_idx),TrimPolar.CDtot(1:stall_idx),Telemetry.CL_req(timeIdx));
    Telemetry.Drag(timeIdx) = Telemetry.CD_gen(timeIdx).*(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref);
    Telemetry.Thrust_req(timeIdx) = Telemetry.Drag(timeIdx);
    
    % finding our climb rate by subtracting our physical power required to move through the air from our physical power available from the propeller
    Telemetry.ClimbRate_fps(timeIdx) = (Telemetry.MaxPower_Avail(timeIdx) - Telemetry.Vfps(timeIdx-1).*Telemetry.Drag(timeIdx))./(Telemetry.Weight(timeIdx-1)); % finding climb rate from prop thrust and drag
    Telemetry.ClimbRate_fpmin(timeIdx) = Telemetry.ClimbRate_fps(timeIdx).*60;

    % calculating fuel burn using prescribed SFC and power consumption
    Telemetry.FuelBurn(timeIdx) = Telemetry.BSFC(timeIdx).*Telemetry.TotalPowerReq_hp(timeIdx); % fuel burn in lbs/hr
    Telemetry.FuelLoad(timeIdx) = Telemetry.FuelLoad(timeIdx-1) - (Telemetry.FuelBurn(timeIdx)./3600)*timeIncrement; % calculating fuel burn in lb/s and multiplying by flight time increment
    Telemetry.Weight(timeIdx) = Telemetry.FuelLoad(timeIdx) + DryWeight;

    % we now go through and update all our flight conditions since we know the change in weight and how far the aircraft must have traveled
    climbAngle = asind((Telemetry.ClimbRate_fps(timeIdx))/(Telemetry.Vfps(timeIdx-1)));
    groundSpd = (Telemetry.Vfps(timeIdx-1))*cosd(climbAngle);
    Telemetry.Altitude(timeIdx) = Telemetry.Altitude(timeIdx-1) + (Telemetry.ClimbRate_fps(timeIdx))*timeIncrement;
    [~, ~, Telemetry.density(timeIdx), Telemetry.sigma(timeIdx)] = ATMOS76(Telemetry.Altitude(timeIdx),0);

    Telemetry.groundCover(timeIdx) = (Telemetry.Vfps(timeIdx-1)).*timeIncrement + Telemetry.groundCover(timeIdx-1); % distance covered, ft
    Telemetry.timeStamp(timeIdx) = timeIncrement + Telemetry.timeStamp(timeIdx-1);
    Telemetry.KEAS(timeIdx) = Telemetry.KEAS(timeIdx-1);
    Telemetry.KTAS(timeIdx) = Telemetry.KEAS(timeIdx)./sqrt(Telemetry.sigma(timeIdx));
    Telemetry.Vfps(timeIdx) = Telemetry.KTAS(timeIdx)./ft2kts;

    timeIdx = timeIdx+1;

end


%%


%% Mission Loiter

loiterEnd = loiterTime*60*60/timeIncrement;
for ii = timeIdx:1:(timeIdx+loiterEnd)

    % Telemetry.MaxPower(timeIdx) = (maxPower + internalLosses).*Telemetry.sigma(timeIdx-1) - internalLosses; % max power available out of the engine, derated for altitude (ft-lb/s)

    % calculating lift coefficient, drag coefficient, and resultant required total thrust for the current weight/altitude/airspeed
    Telemetry.CL_req(timeIdx) = Telemetry.Weight(timeIdx-1)./(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref);
    Telemetry.CD_gen(timeIdx) = interp1(TrimPolar.CLtot(1:stall_idx),TrimPolar.CDtot(1:stall_idx),Telemetry.CL_req(timeIdx));
    Telemetry.Thrust_req(timeIdx) = Telemetry.CD_gen(timeIdx).*(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref); % lbs

    % calculating the thrust required from EACH propeller (since there are multiple)
    Telemetry.PropThrust(timeIdx) = Telemetry.Thrust_req(timeIdx)./spinningprops; % dividing required thrust by number of props
    % this function calculates the matching RPM and required power consumption for the propeller operating at the provided airspeed/altitude/thrust, using constructed coefficient curves
    [Telemetry.AdvanceJ(timeIdx), Telemetry.RPM(timeIdx), Telemetry.Power(timeIdx), Telemetry.Eta(timeIdx), Telemetry.CT(timeIdx), Telemetry.CP(timeIdx)] = Propeller_Mejzlik32x18(Telemetry.Vfps(timeIdx-1), Telemetry.density(timeIdx-1), Telemetry.PropThrust(timeIdx));

    % calculating the total power required by multiplying the individual propeller power consumption by the number of props, and then adding in the electrical power draw
    Telemetry.TotalPowerReq(timeIdx) = Telemetry.Power(timeIdx).*spinningprops + Power_electric; % total power in ft-lb/s
    Telemetry.TotalPowerReq_hp(timeIdx) = Telemetry.TotalPowerReq(timeIdx).*ftlb2hp; % total power in hp

    Telemetry.ClimbRate_fps(timeIdx) = 0; % bookkeeping climb rate
    Telemetry.ClimbRate_fpmin(timeIdx) = Telemetry.ClimbRate_fps(timeIdx).*60;

    % calculating fuel burn by finding throttle for required power and outputting associated SFC
    [Telemetry.Torque(timeIdx), Telemetry.BSFC(timeIdx)] = Engine_NW230(Telemetry.sigma(timeIdx-1),RPMset,Telemetry.TotalPowerReq_hp(timeIdx)./W2hp); % BSFC in lb/hp/hr

    Telemetry.FuelBurn(timeIdx) = Telemetry.BSFC(timeIdx).*Telemetry.TotalPowerReq_hp(timeIdx); % fuel burn in lbs/hr
    Telemetry.FuelLoad(timeIdx) = Telemetry.FuelLoad(timeIdx-1) - (Telemetry.FuelBurn(timeIdx)./3600)*timeIncrement; % calculating fuel burn in lb/s and multiplying by flight time increment
    Telemetry.Weight(timeIdx) = Telemetry.FuelLoad(timeIdx) + DryWeight;

    % we now go through and update all our flight conditions since we know the change in weight and how far the aircraft must have traveled
    Telemetry.Altitude(timeIdx) = Telemetry.Altitude(timeIdx-1);
    [~, ~, Telemetry.density(timeIdx), Telemetry.sigma(timeIdx)] = ATMOS76(Telemetry.Altitude(timeIdx),0);

    % [BestE,BestR] = FindBestAirspeeds(Telemetry.Weight(timeIdx),Telemetry.density(timeIdx),Telemetry.sigma(timeIdx),TrimPolar.CLtot,TrimPolar.CDtot,Sref);
    % Telemetry.KEAS(timeIdx) = BestR.V_KEAS;
    % Telemetry.KTAS(timeIdx) = BestR.V_KTAS;
    % Telemetry.Vfps(timeIdx) = BestR.V_fps_TAS;

    Telemetry.groundCover(timeIdx) = (Telemetry.Vfps(timeIdx-1)).*timeIncrement + Telemetry.groundCover(timeIdx-1); % distance covered, ft
    Telemetry.timeStamp(timeIdx) = timeIncrement + Telemetry.timeStamp(timeIdx-1);
    Telemetry.KEAS(timeIdx) = Telemetry.KEAS(timeIdx-1);
    Telemetry.KTAS(timeIdx) = Telemetry.KEAS(timeIdx)./sqrt(Telemetry.sigma(timeIdx));
    Telemetry.Vfps(timeIdx) = Telemetry.KTAS(timeIdx)./ft2kts;

    % if Telemetry.FuelLoad(timeIdx) <= 3
    %     break
    % end

    timeIdx = timeIdx+1;
end


%%


%% Descend to Launch Alt

% while Telemetry.Altitude > launchAlt
for ii = timeIdx:1:(timeIdx+200)

    Telemetry.TotalPowerReq(timeIdx) = Power_electric;
    Telemetry.TotalPowerReq_hp(timeIdx) = Telemetry.TotalPowerReq(timeIdx).*ftlb2hp;
    % to simulate an engine at idle, we need to find the point where the prop absorbs no power, i.e. Power = 0
    [Telemetry.AdvanceJ(timeIdx), Telemetry.RPM(timeIdx), Telemetry.PropThrust(timeIdx), Telemetry.Eta(timeIdx), Telemetry.CT(timeIdx), Telemetry.CP(timeIdx)] = Propeller_Mejzlik32x18_Pwr(Telemetry.Vfps(timeIdx-1), Telemetry.density(timeIdx-1), 0);
    Telemetry.Thrust(timeIdx) = Telemetry.PropThrust(timeIdx).*spinningprops; % multiplying generated thrust by number of props

    % calculating lift coefficient and drag coefficient for flight at the current weight/altitude/airspeed
    Telemetry.CL_req(timeIdx) = Telemetry.Weight(timeIdx-1)./(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref); % finding CL using weight
    Telemetry.CD_gen(timeIdx) = interp1(TrimPolar.CLtot(1:stall_idx),TrimPolar.CDtot(1:stall_idx),Telemetry.CL_req(timeIdx));
    Telemetry.Drag(timeIdx) = Telemetry.CD_gen(timeIdx).*(0.5.*Telemetry.density(timeIdx-1).*(Telemetry.Vfps(timeIdx-1).^2).*Sref);

    % finding our descent rate by subtracting our physical power required to move through the air from our physical power available from the propeller (i.e. thrust generated)
    Telemetry.ClimbRate_fps(timeIdx) = Telemetry.Vfps(timeIdx-1).*(Telemetry.Thrust(timeIdx) - Telemetry.Drag(timeIdx))./Telemetry.Weight(timeIdx-1); % finding descent rate from prop thrust and drag
    Telemetry.ClimbRate_fpmin(timeIdx) = Telemetry.ClimbRate_fps(timeIdx).*60;

    % calculating fuel burn by finding throttle for required power and outputting associated SFC
    [Telemetry.Torque(timeIdx), Telemetry.BSFC(timeIdx)] = Engine_NW230(Telemetry.sigma(timeIdx-1),RPMset,Telemetry.TotalPowerReq_hp(timeIdx)./W2hp); % BSFC in lb/hp/hr

    % calculating fuel burn using prescribed SFC and power consumption
    Telemetry.FuelBurn(timeIdx) = Telemetry.BSFC(timeIdx).*Telemetry.TotalPowerReq_hp(timeIdx); % fuel burn in lbs/hr
    Telemetry.FuelLoad(timeIdx) = Telemetry.FuelLoad(timeIdx-1) - (Telemetry.FuelBurn(timeIdx)./3600)*timeIncrement; % calculating fuel burn in lb/s and multiplying by flight time increment
    Telemetry.Weight(timeIdx) = Telemetry.FuelLoad(timeIdx) + DryWeight;

    climbAngle = asind((Telemetry.ClimbRate_fps(timeIdx))/(Telemetry.Vfps(timeIdx-1)));
    groundSpd = (Telemetry.Vfps(timeIdx-1))*cosd(climbAngle);

    % we now go through and update all our flight conditions since we know the change in weight and how far the aircraft must have traveled
    Telemetry.Altitude(timeIdx) = Telemetry.Altitude(timeIdx-1) + (Telemetry.ClimbRate_fps(timeIdx))*timeIncrement;
    [~, ~, Telemetry.density(timeIdx), Telemetry.sigma(timeIdx)] = ATMOS76(Telemetry.Altitude(timeIdx),0);

    % [BestE,BestR] = FindBestAirspeeds(Telemetry.Weight(timeIdx),Telemetry.density(timeIdx),Telemetry.sigma(timeIdx),TrimPolar.CLtot,TrimPolar.CDtot,Sref);
    % Telemetry.KEAS(timeIdx) = BestE.V_KEAS;
    % Telemetry.KTAS(timeIdx) = BestE.V_KTAS;
    % Telemetry.Vfps(timeIdx) = BestE.V_fps_TAS;

    Telemetry.groundCover(timeIdx) = (Telemetry.Vfps(timeIdx-1)).*timeIncrement + Telemetry.groundCover(timeIdx-1); % distance covered, ft
    Telemetry.timeStamp(timeIdx) = timeIncrement + Telemetry.timeStamp(timeIdx-1);
    Telemetry.KEAS(timeIdx) = Telemetry.KEAS(timeIdx-1);
    Telemetry.KTAS(timeIdx) = Telemetry.KEAS(timeIdx)./sqrt(Telemetry.sigma(timeIdx));
    Telemetry.Vfps(timeIdx) = Telemetry.KTAS(timeIdx)./ft2kts;

    % this will stop the simulation if we reach our launch altitude, i.e. land
    if Telemetry.Altitude(timeIdx) <= launchAlt
        break
    end
    timeIdx = timeIdx+1;
end

%%


%% Plotting

figure;
hold on; grid on; grid minor
plot(Telemetry.timeStamp./60,Telemetry.Altitude,'LineWidth',1.5)
xlabel('Time (min)')
ylabel('Altitude (ft)')
title('Altitude Over Time')

% figure;
% hold on; grid on; grid minor
% plot(Telemetry.timeStamp./60,(Telemetry.PropThrust)*spinningprops,'.','LineWidth',1.5)
% xlabel('Time (min)')
% ylabel('Thrust Required (lb)')
% title('Output Thrust Required')
% 
% figure;
% hold on; grid on; grid minor
% plot(Telemetry.timeStamp./60,Telemetry.TotalPowerReq_hp,'.','LineWidth',1.5)
% xlabel('Time (min)')
% ylabel('Power Required (hp)')
% title('Total Engine Output Power Required')
% 
% figure;
% hold on; grid on; grid minor
% plot(Telemetry.timeStamp./60,Telemetry.FuelBurn,'.','LineWidth',1.5)
% xlabel('Time (min)')
% ylabel('Fuel Burn Rate (lb/hr)')
% title('Fuel Burn Rate')
% 
% figure;
% hold on; grid on; grid minor
% plot(Telemetry.FuelBurn,Telemetry.Altitude,'.','LineWidth',1.5)
% xlabel('Fuel Burn Rate (lb/hr)')
% ylabel('Altitude (ft)')
% title('Fuel Burn Rate')

% figure;
% hold on; grid on; grid minor
% plot(Telemetry.timeStamp./60,Telemetry.DescentRate_fps,'.','LineWidth',1.5)
% % ylim([0 120])
% xlabel('Time (min)')
% ylabel('Descent Rate (ft/s)')
% title('Vehicle Climb/Descent Rates')

figure;
hold on; grid on; grid minor
plot(Telemetry.timeStamp./60,Telemetry.ClimbRate_fps*60,'.','LineWidth',1.5)
% ylim([0 120])
xlabel('Time (min)')
ylabel('Climb Rate (ft/min)')
title('Vehicle Climb/Descent Rates')


% figure;
% hold on; grid on; grid minor
% plot3(Telemetry.timeStamp./60,Telemetry.ClimbRate_fps*60,'.','LineWidth',1.5)
% % ylim([0 120])
% xlabel('Time (min)')
% ylabel('Climb Rate (ft/min)')
% title('Vehicle Climb/Descent Rates')

% figure;
% hold on; grid on; grid minor
% plot(Telemetry.timeStamp./60,Telemetry.FuelLoad,'.','LineWidth',1.5)
% ylim([0 120])
% xlabel('Time (min)')
% ylabel('Total Fuel (lb)')
% title('Total Fuel Load')

% figure;
% hold on; grid on; grid minor
% plot(Telemetry.timeStamp./3600,Telemetry.Altitude,'LineWidth',1.5)
% xlabel('Time (hr)')
% ylabel('Altitude (ft)')
% title('Altitude Over Time')

figure;
hold on; grid on; grid minor
plot(Telemetry.timeStamp./3600,Telemetry.FuelLoad,'.','LineWidth',1.5)
ylim([0 140])
xlabel('Time (hr)')
ylabel('Total Fuel (lb)')
title('Total Fuel Load')


%% Save Data


fprintf('Total Flight Time, hrs')
(timeIdx-1)*timeIncrement/60/60

fprintf('Total Distance Traveled, nm')
Telemetry.groundCover(end)*ft2nm

fprintf('End Fuel, lbs')
Telemetry.FuelLoad(end)

load('ChartsData.mat')

ChartsData.E1p6W.MaxPwr7.PL65.Alt2k.V70 = Telemetry;
% ChartsData.MaxPwr11.PL65.Alt8k.V80.loiter_time = loiterTime;
save('ChartsData.mat','ChartsData')


