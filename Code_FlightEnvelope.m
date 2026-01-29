clear all; close all; clc;

ft2kts = 0.592484;
W2ftlb = 550/746; % 550 ft-lb/s = 746 W
W2hp = 1/746; % 1 hp = 746 W
ftlb2hp = 1/550;

colors = [0 0.4470 0.7410;
         0.8500 0.3250 0.0980;
         0.9290 0.6940 0.1250;
         0.4940 0.1840 0.5560;
         0.4660 0.6740 0.1880;
         0.3010 0.7450 0.9330;
         0.6350 0.0780 0.1840];

%% Aircraft Geometry

load('TrimPolar.mat')

Sref = 5522/144;
cref = 20/12;
bref = 216/12;

MGTOW = 320; % lbs
DryW = 210; % lbs
% Payload = 40; % lbs, Alticam AC-14 plus extra goodies
% Fuel = MGTOW - (230 + 40); % lbs
% Weight = MGTOW - (Fuel/2); % lbs
Weight = MGTOW; % lbs

CL_stall = TrimPolar.CLtot(21);
CL_max = TrimPolar.CLtot(23);
stall_idx = 21;

% NW230 Engine
% maxPower = 11600; % Watts
% SFC_s = 5.56; % lb/hr
maxPower = 10000; % Watts
SFC_s = 5.56; % lb/hr

% Previous Engine
% maxPower = 11000; % Watts
% BSFC = 0.422; % lb/hr/hp
% SFC_s = (BSFC/3600)*(ftlb2hp); % lb/s/ft-lb/s

% TrimPolar.CDtot = TrimPolar.CDtot;

Power_electric = 500; % Watts, base avionics
spinningprops = 2; % number of propellers used in forward flight

%% Aero Model Plots

% figure;
% hold on; grid on; grid minor;
% plot(TrimPolar.Alpha,(TrimPolar.CLtot./TrimPolar.CDtot),'LineWidth',1.5)
% ylim([0 20])
% xlabel('\alpha (deg)')
% ylabel('L/D')
% title('L/D of Raider 330, Unpowered')
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(TrimPolar.Alpha,TrimPolar.CLtot,'LineWidth',1.5)
% xlabel('\alpha (deg)')
% ylabel('C_L')
% title('Lift of Raider 330, Unpowered')
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(TrimPolar.Alpha,TrimPolar.CDtot,'LineWidth',1.5)
% xlabel('\alpha (deg)')
% ylabel('C_D')
% title('Drag of Raider 330, Unpowered')
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(TrimPolar.Alpha,TrimPolar.Cmtot,'LineWidth',1.5)
% xlabel('\alpha (deg)')
% ylabel('C_m')
% title('Pitch of Raider 330, Unpowered')


% figure;
% hold on; grid on; grid minor;
% plot(TrimPolar.CDtot,TrimPolar.CLtot,'LineWidth',1.5)
% xlim([0 0.2])
% xlabel('C_D')
% ylabel('C_L')
% title('CL vs CD of Raider 330, Unpowered')

%%

%% Day Conditions

altitudes = [0:1000:10000,10200:200:30000]; % in ft
[~, ~, densities, sigma] = ATMOS76(altitudes,0);

V_fps = [60:0.5:72,73:1:79,80:2:274]; % in ft/s
% V_fps = [70:1:79,80:2:100]; % in ft/s

%%

%% Calculate Lift & Thrust

% calculating CL, CD, and therefore thrust required for vehicle at altitude
for jj = 1:length(altitudes)
    for ii = 1:length(V_fps)
        CL_req(jj,ii) = Weight./(0.5.*densities(jj).*(V_fps(ii).^2).*Sref); % finding CL using weight, lbs & ft
        CD_gen(jj,ii) = interp1(TrimPolar.CLtot(1:stall_idx),TrimPolar.CDtot(1:stall_idx),CL_req(jj,ii)); % finding CD using trim polar
        Thrust_req(jj,ii) = CD_gen(jj,ii).*(0.5.*densities(jj).*(V_fps(ii).^2).*Sref); % finding thrust from drag coeff, lbs
        Endur_metric(jj,ii) = (CL_req(jj,ii).^(3/2))./CD_gen(jj,ii); % 
        Range_metric(jj,ii) = CL_req(jj,ii)./CD_gen(jj,ii);
    end
    V_fps_EAS(jj,:) = V_fps.*sqrt(sigma(jj));
    V_fps_TAS(jj,:) = V_fps;
    V_KEAS(jj,:) = V_fps_EAS(jj,:).*ft2kts;
    V_KTAS(jj,:) = V_fps_TAS(jj,:).*ft2kts;

    [BestE.Endur, BestE.idx] = max(Endur_metric(1,:));
    [BestR.Range, BestR.idx] = max(Range_metric(1,:));

    BestE.V_fps_EAS(jj) = V_fps_EAS(1,BestE.idx); %best endurance airspeed at this altitude
    BestR.V_fps_EAS(jj) = V_fps_EAS(1,BestR.idx);
    BestE.V_KEAS(jj) = V_KEAS(1,BestE.idx);
    BestR.V_KEAS(jj) = V_KEAS(1,BestR.idx);

    BestE.V_fps_TAS(jj) = BestE.V_fps_EAS(jj)./sqrt(sigma(jj));
    BestR.V_fps_TAS(jj) = BestR.V_fps_EAS(jj)./sqrt(sigma(jj));
    BestE.V_KTAS(jj) = BestE.V_KEAS(jj)./sqrt(sigma(jj));
    BestR.V_KTAS(jj) = BestR.V_KEAS(jj)./sqrt(sigma(jj));
end


    

    

%% Lift & Thrust Plots

% figure;
% hold on; grid on; grid minor
% plot(V_KEAS(1,:),Endur_metric(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% % plot(V_KEAS_interp,Endur_metric_interp,'LineWidth',1.5,'DisplayName','Interp')
% plot(V_KEAS(3,:),Endur_metric(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KEAS(5,:),Endur_metric(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KEAS(7,:),Endur_metric(7,:),'LineWidth',1.5,'DisplayName','6k ft')
% plot(V_KEAS(9,:),Endur_metric(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KEAS(11,:),Endur_metric(11,:),'LineWidth',1.5,'DisplayName','10k ft')
% % plot([40 120],[1.59 1.59],'LineWidth',1.5,'HandleVisibility','off')
% xlim([30 130])
% % ylim([0 2.5])
% xlabel('Airspeed (KTAS)')
% ylabel('C_L')
% title('Required C_L and Thrust at Airspeed')
% legend('show','location','best')


% figure;
% subplot(2,1,1)
% hold on; grid on; grid minor
% plot(V_KTAS(1,:),CL_req(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(3,:),CL_req(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KTAS(5,:),CL_req(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(7,:),CL_req(7,:),'LineWidth',1.5,'DisplayName','6k ft')
% plot(V_KTAS(9,:),CL_req(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KTAS(11,:),CL_req(11,:),'LineWidth',1.5,'DisplayName','10k ft')
% % plot([40 120],[1.59 1.59],'LineWidth',1.5,'HandleVisibility','off')
% xlim([30 130])
% ylim([0 2.5])
% %xlabel('Airspeed (KTAS)')
% ylabel('C_L')
% title('Required C_L and Thrust at Airspeed')
% legend('show','location','best')


% figure;
% subplot(2,1,2)
% hold on; grid on; grid minor
% plot(V_KTAS(1,:),Thrust_req(1,:),'LineWidth',1.5) %,'DisplayName','0 ft'
% plot(V_KTAS(3,:),Thrust_req(3,:),'LineWidth',1.5) %,'DisplayName','2k ft'
% plot(V_KTAS(5,:),Thrust_req(5,:),'LineWidth',1.5) %,'DisplayName','4k ft'
% plot(V_KTAS(7,:),Thrust_req(7,:),'LineWidth',1.5) %,'DisplayName','6k ft'
% plot(V_KTAS(9,:),Thrust_req(9,:),'LineWidth',1.5) %,'DisplayName','8k ft'
% plot(V_KTAS(11,:),Thrust_req(11,:),'LineWidth',1.5) %,'DisplayName','10k ft'
% xlim([30 130])
% ylim([10 50])
% xlabel('Airspeed (KTAS)')
% ylabel('Thrust Required (lb)')
% % legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor
% plot(V_KEAS(1,:),Thrust_req(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(3,:),Thrust_req(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KEAS(5,:),Thrust_req(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KEAS(7,:),Thrust_req(7,:),'LineWidth',1.5,'DisplayName','6k ft')
% plot(V_KEAS(9,:),Thrust_req(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KEAS(11,:),Thrust_req(11,:),'LineWidth',1.5,'DisplayName','10k ft')
% xlim([30 130])
% xlabel('Airspeed (KEAS)')
% ylabel('Thrust Required (lb)')
% legend('show','location','best')


% figure;
% hold on; grid on; grid minor
% plot(V_KTAS(1,:),CD_gen(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(2,:),CD_gen(2,:),'LineWidth',1.5,'DisplayName','5k ft, TAS')
% plot(V_KTAS(3,:),CD_gen(3,:),'LineWidth',1.5,'DisplayName','10k ft, TAS')
% plot(V_KEAS(1,:),CD_gen(1,:),'--','LineWidth',1.5,'DisplayName','EAS')
% xlim([40 120])
% xlabel('Airspeed (KTAS)')
% ylabel('C_D')
% legend('show','location','best')

%%

%% Calculate Power & Max Power

AdvanceJ = [];
RPM = [];
Power = [];
Eta = [];
CT = [];
CP = [];

% calculating RPM and power required (i.e. power CONSUMED by the motors)
for jj = 1:length(altitudes)
    for ii = 1:length(V_fps)
        if isnan(Thrust_req(jj,ii))
            PropThrust = 0;
            Power(jj,ii) = 20000;
        else
            PropThrust = Thrust_req(jj,ii)./spinningprops; % dividing required thrust by number of props
            % finding the power that must be supplied to the motor to generate this amount of thrust at given airspeed + altitude
            [AdvanceJ(jj,ii), RPM(jj,ii), Power(jj,ii), Eta(jj,ii), CT(jj,ii), CP(jj,ii)] = Propeller_Mejzlik32x18(V_fps(ii), densities(jj), PropThrust); 

            Endurance(jj,ii) = (Eta(jj,ii)./(SFC_s*spinningprops)).*Endur_metric(jj,ii).*((2*densities(jj).*Sref).^(1/2)).*(DryW^(-0.5)-(MGTOW^(-0.5)));
            Range(jj,ii) = (Eta(jj,ii)./(SFC_s*spinningprops)).*Range_metric(jj,ii).*log(MGTOW/DryW);
        end
        TotalPowerReq(jj,ii) = Power(jj,ii).*spinningprops; % total power in ft-lb/s
        TotalPowerReq_W(jj,ii) = TotalPowerReq(jj,ii).*(745.7/550); % conversion to Watts through hp

       [Engine.MaxPower_out(jj), Engine.Torque_out(jj), Engine.BSFC_out(jj)] = Engine_NW230(sigma(jj),5250);
    end
end

figure;
hold on; grid on; grid minor;
plot(Engine.MaxPower_out,sigma,'--','LineWidth',1.5,'DisplayName','Max Power Avail')
plot(TotalPowerReq_W(:,52),sigma,'LineWidth',1.5,'DisplayName','Power Req')
xlabel('Engine Power (W)')
ylabel('Density Ratio')
xlim([0 10000])
legend('show','location','best')

internalLosses = maxPower.*0.13; % assumes 13% of power generated is lost due to internal efficiencies (Watts)
maxPower_altitude_W = ((maxPower + internalLosses).*sigma - internalLosses); % max power available, derated for altitude (Watts)
% maxMotorPower = (maxPower_altitude_W - Power_electric); % calculating the maximum power available to the motor-prop at each altitude



%% Prop Model & Power Plots

% figure;
% hold on; grid on; grid minor;
% % ylim([0.4 1])
% plot(V_KEAS(1,:),Endurance(1,:)./3600,'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),Endurance(5,:)./3600,'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KEAS(9,:),Endurance(9,:)./3600,'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KEAS(13,:),Endurance(13,:)./3600,'LineWidth',1.5,'DisplayName','12000 ft')
% plot(V_KEAS(17,:),Endurance(17,:)./3600,'LineWidth',1.5,'DisplayName','16000 ft')
% plot(V_KEAS(21,:),Endurance(21,:)./3600,'LineWidth',1.5,'DisplayName','20000 ft')
% plot(V_KEAS(29,:),Endurance(29,:)./3600,'LineWidth',1.5,'DisplayName','24000 ft')
% plot(V_KEAS(33,:),Endurance(33,:)./3600,'LineWidth',1.5,'DisplayName','26000 ft')
% xlabel('Airspeed (KEAS)')
% ylabel('Endurance (hrs)')
% legend('show','location','best')
% 

figure;
hold on; grid on; grid minor;
% ylim([0.4 1])
plot(V_KTAS(1,:),Endurance(1,:)./3600,'LineWidth',1.5,'DisplayName','0 ft')
plot(V_KTAS(5,:),Endurance(5,:)./3600,'LineWidth',1.5,'DisplayName','4000 ft')
plot(V_KTAS(9,:),Endurance(9,:)./3600,'LineWidth',1.5,'DisplayName','8000 ft')
plot(V_KTAS(13,:),Endurance(13,:)./3600,'LineWidth',1.5,'DisplayName','12000 ft')
plot(V_KTAS(17,:),Endurance(17,:)./3600,'LineWidth',1.5,'DisplayName','16000 ft')
plot(V_KTAS(21,:),Endurance(21,:)./3600,'LineWidth',1.5,'DisplayName','20000 ft')
plot(V_KTAS(29,:),Endurance(29,:)./3600,'LineWidth',1.5,'DisplayName','24000 ft')
plot(V_KTAS(33,:),Endurance(33,:)./3600,'LineWidth',1.5,'DisplayName','26000 ft')
xlabel('Airspeed (KTAS)')
ylabel('Endurance (hrs)')
legend('show','location','best')


figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),AdvanceJ(1,:),'color',colors(1,:),'LineWidth',1.5,'DisplayName','0k ft')
% plot(V_KTAS(2,2:end),AdvanceJ(2,2:end),'color',colors(2,:),'LineWidth',1.5,'DisplayName','1k ft')
% plot(V_KTAS(3,3:end),AdvanceJ(3,3:end),'color',colors(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KTAS(4,4:end),AdvanceJ(4,4:end),'color',colors(4,:),'LineWidth',1.5,'DisplayName','3k ft')
% plot(V_KTAS(5,5:end),AdvanceJ(5,5:end),'color',colors(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(6,6:end),AdvanceJ(6,6:end),'color',colors(6,:),'LineWidth',1.5,'DisplayName','5k ft')
% ylim([0.6 1])
% xlabel('Airspeed (KEAS)')
% ylabel('Advance Ratio J')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),CT(1,:),'color',colors(1,:),'LineWidth',1.5,'DisplayName','0k ft')
% plot(V_KTAS(2,2:end),CT(2,2:end),'color',colors(2,:),'LineWidth',1.5,'DisplayName','1k ft')
% plot(V_KTAS(3,3:end),CT(3,3:end),'color',colors(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KTAS(4,4:end),CT(4,4:end),'color',colors(4,:),'LineWidth',1.5,'DisplayName','3k ft')
% plot(V_KTAS(5,5:end),CT(5,5:end),'color',colors(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(6,6:end),CT(6,6:end),'color',colors(6,:),'LineWidth',1.5,'DisplayName','5k ft')
% % ylim([0 1])
% xlabel('Airspeed (KEAS)')
% ylabel('C_T, Single Prop')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% scatter(AdvanceJ(1,11:end),CT(1,11:end),'DisplayName','0k ft')
% % scatter(AdvanceJ(2,12:end),CT(2,12:end),'DisplayName','1k ft')
% scatter(AdvanceJ(3,13:end),CT(3,13:end),'DisplayName','2k ft')
% % scatter(AdvanceJ(4,14:end),CT(4,14:end),'DisplayName','3k ft')
% scatter(AdvanceJ(5,15:end),CT(5,15:end),'DisplayName','4k ft')
% % scatter(AdvanceJ(6,16:end),CT(6,16:end),'DisplayName','5k ft')
% xlim([0 1])
% ylim([-0.05 0.15])
% xlabel('Advance Ratio J')
% ylabel('C_T, Single Prop')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),CP(1,:),'color',colors(1,:),'LineWidth',1.5,'DisplayName','0k ft')
% plot(V_KTAS(2,2:end),CP(2,2:end),'color',colors(2,:),'LineWidth',1.5,'DisplayName','1k ft')
% plot(V_KTAS(3,3:end),CP(3,3:end),'color',colors(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KTAS(4,4:end),CP(4,4:end),'color',colors(4,:),'LineWidth',1.5,'DisplayName','3k ft')
% plot(V_KTAS(5,5:end),CP(5,5:end),'color',colors(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(6,6:end),CP(6,6:end),'color',colors(6,:),'LineWidth',1.5,'DisplayName','5k ft')
% % ylim([0 1])
% xlabel('Airspeed (KEAS)')
% ylabel('C_P, Single Prop')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),RPM(1,:),'color',colors(1,:),'LineWidth',1.5,'DisplayName','0k ft')
% plot(V_KTAS(2,2:end),RPM(2,2:end),'color',colors(2,:),'LineWidth',1.5,'DisplayName','1k ft')
% plot(V_KTAS(3,3:end),RPM(3,3:end),'color',colors(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot(V_KTAS(4,4:end),RPM(4,4:end),'color',colors(4,:),'LineWidth',1.5,'DisplayName','3k ft')
% plot(V_KTAS(5,5:end),RPM(5,5:end),'color',colors(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(6,6:end),RPM(6,6:end),'color',colors(6,:),'LineWidth',1.5,'DisplayName','5k ft')
% % ylim([0 1])
% xlabel('Airspeed (KEAS)')
% ylabel('RPM')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% ylim([0.4 1])
% plot(V_KEAS(1,:),Eta(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),Eta(5,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KEAS(9,:),Eta(9,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KEAS(13,:),Eta(13,:),'LineWidth',1.5,'DisplayName','12000 ft')
% xlabel('Airspeed (KEAS)')
% ylabel('Propeller Efficiency \eta')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% ylim([0.4 1])
% plot(V_KTAS(1,:),Eta(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(5,:),Eta(5,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KTAS(9,:),Eta(9,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KTAS(13,:),Eta(13,:),'LineWidth',1.5,'DisplayName','12000 ft')
% xlabel('Airspeed (KTAS)')
% ylabel('Propeller Efficiency \eta')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% ylim([0 8000])
% plot(V_KEAS(1,:),TotalPowerReq_W(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),TotalPowerReq_W(5,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KEAS(9,:),TotalPowerReq_W(9,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KEAS(13,:),TotalPowerReq_W(13,:),'LineWidth',1.5,'DisplayName','12000 ft')
% xlabel('Airspeed (KEAS)')
% ylabel('Power Required (Watts)')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),TotalPowerReq(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(5,:),TotalPowerReq(5,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KTAS(9,:),TotalPowerReq(9,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KTAS(13,:),TotalPowerReq(13,:),'LineWidth',1.5,'DisplayName','12000 ft')
% ylim([0 25000])
% xlabel('Airspeed (KTAS)')
% ylabel('Power Required (lb-ft/s)')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),TotalPowerReq(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),TotalPowerReq(5,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KEAS(9,:),TotalPowerReq(9,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KEAS(13,:),TotalPowerReq(13,:),'LineWidth',1.5,'DisplayName','12000 ft')
% ylim([0 25000])
% xlabel('Airspeed (KEAS)')
% ylabel('Power Required (lb-ft/s)')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),TotalPowerReq(1,:).*ftlb2hp,'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),TotalPowerReq(5,:).*ftlb2hp,'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KEAS(9,:),TotalPowerReq(9,:).*ftlb2hp,'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KEAS(13,:),TotalPowerReq(13,:).*ftlb2hp,'LineWidth',1.5,'DisplayName','12000 ft')
% plot([30 110],[22 22],'r--','LineWidth',1.5,'DisplayName','Max Power')
% ylim([0 30])
% xlabel('Airspeed (KEAS)')
% ylabel('Power Required (hp)')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),TotalPowerReq_W(1,:),'color',colors(1,:),'LineWidth',1.5,'DisplayName','0k ft')
% plot([30 130],[maxPower_altitude_W(1) maxPower_altitude_W(1)],'--','color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(V_KTAS(2,2:end),TotalPowerReq_W(2,2:end),'color',colors(2,:),'LineWidth',1.5,'DisplayName','1k ft')
% plot([30 130],[maxPower_altitude_W(2) maxPower_altitude_W(2)],'--','color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(V_KTAS(3,3:end),TotalPowerReq_W(3,3:end),'color',colors(3,:),'LineWidth',1.5,'DisplayName','2k ft')
% plot([30 130],[maxPower_altitude_W(3) maxPower_altitude_W(3)],'--','color',colors(3,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(V_KTAS(4,4:end),TotalPowerReq_W(4,4:end),'color',colors(4,:),'LineWidth',1.5,'DisplayName','3k ft')
% plot([30 130],[maxPower_altitude_W(4) maxPower_altitude_W(4)],'--','color',colors(4,:),'LineWidth',1.5,'HandleVisibility','off')
% ylim([0 18000])
% xlabel('Airspeed (KEAS)')
% ylabel('Power Required (W)')
% legend('show','location','best')

figure;
hold on; grid on; grid minor;
plot(V_KTAS(1,:),TotalPowerReq_W(1,:)/1000,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
plot([30 130],([maxPower_altitude_W(1) maxPower_altitude_W(1)] - Power_electric)/1000,'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
plot(V_KTAS(6,:),TotalPowerReq_W(6,:)/1000,'color',colors(2,:),'LineWidth',1.5,'DisplayName','4k ft, P req')
plot([30 130],([maxPower_altitude_W(5) maxPower_altitude_W(5)] - Power_electric)/1000,'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','4k ft, P avail')
plot(V_KTAS(11,:),TotalPowerReq_W(11,:)/1000,'color',colors(3,:),'LineWidth',1.5,'DisplayName','8k ft, P req')
plot([30 130],([maxPower_altitude_W(9) maxPower_altitude_W(9)] - Power_electric)/1000,'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','8k ft, P avail')
plot(V_KTAS(16,:),TotalPowerReq_W(16,:)/1000,'color',colors(4,:),'LineWidth',1.5,'DisplayName','12k ft, P req')
plot([30 130],([maxPower_altitude_W(13) maxPower_altitude_W(13)] - Power_electric)/1000,'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','12k ft, P avail')
% plot(V_KTAS(21,:),TotalPowerReq_W(21,:)/1000,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot([30 130],([maxPower_altitude_W(21) maxPower_altitude_W(21)] - Power_electric)/1000,'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
ylim([0 14])
xlim([40 120])
xlabel('Airspeed (KTAS)')
ylabel('Power (kW)')
title('Shaft Power Available vs Required')
legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),maxPower_altitude_W(1) - TotalPowerReq_W(1,:),'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(6,:),maxPower_altitude_W(6) - TotalPowerReq_W(6,:),'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft')
% plot(V_KTAS(11,:),maxPower_altitude_W(11) - TotalPowerReq_W(11,:),'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KTAS(16,:),maxPower_altitude_W(16) - TotalPowerReq_W(16,:),'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft')
% plot(V_KTAS(21,:),maxPower_altitude_W(21) - TotalPowerReq_W(21,:),'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft')
% ylim([0 18000])
% xlim([30 120])
% xlabel('Airspeed (KTAS)')
% ylabel('Excess Power (W)')
% title('Excess Power at Altitude')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,85:end),TotalPowerReq_W(1,85:end).*W2hp,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
% plot([30 130],[maxPower_altitude_W(1) maxPower_altitude_W(1)].*W2hp,'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
% plot(V_KTAS(6,94:end),TotalPowerReq_W(6,94:end).*W2hp,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P req')
% plot([30 130],[maxPower_altitude_W(6) maxPower_altitude_W(6)].*W2hp,'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P avail')
% plot(V_KTAS(11,99:end),TotalPowerReq_W(11,99:end).*W2hp,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P req')
% plot([30 130],[maxPower_altitude_W(11) maxPower_altitude_W(11)].*W2hp,'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P avail')
% plot(V_KTAS(16,102:end),TotalPowerReq_W(16,102:end).*W2hp,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P req')
% plot([30 130],[maxPower_altitude_W(16) maxPower_altitude_W(16)].*W2hp,'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P avail')
% plot(V_KTAS(21,106:end),TotalPowerReq_W(21,106:end).*W2hp,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot([30 130],[maxPower_altitude_W(21) maxPower_altitude_W(21)].*W2hp,'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
% % ylim([0 18])
% xlim([30 120])
% xlabel('Airspeed (KTAS)')
% ylabel('Power (hp)')
% title('Power Available and Required at Altitude')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,85:end),TotalPowerReq_W(1,85:end).*W2ftlb,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
% plot([30 130],[maxPower_altitude_W(1) maxPower_altitude_W(1)].*W2ftlb,'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
% plot(V_KTAS(6,94:end),TotalPowerReq_W(6,94:end).*W2ftlb,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P req')
% plot([30 130],[maxPower_altitude_W(6) maxPower_altitude_W(6)].*W2ftlb,'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P avail')
% plot(V_KTAS(11,99:end),TotalPowerReq_W(11,99:end).*W2ftlb,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P req')
% plot([30 130],[maxPower_altitude_W(11) maxPower_altitude_W(11)].*W2ftlb,'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P avail')
% plot(V_KTAS(16,102:end),TotalPowerReq_W(16,102:end).*W2ftlb,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P req')
% plot([30 130],[maxPower_altitude_W(16) maxPower_altitude_W(16)].*W2ftlb,'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P avail')
% plot(V_KTAS(21,106:end),TotalPowerReq_W(21,106:end).*W2ftlb,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot([30 130],[maxPower_altitude_W(21) maxPower_altitude_W(21)].*W2ftlb,'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
% ylim([0 14000])
% xlim([30 120])
% xlabel('Airspeed (KTAS)')
% ylabel('Power (ft-lb)')
% title('Power Available and Required at Altitude')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),[maxPower_altitude_W(1) - TotalPowerReq_W(1,:)].*W2ftlb,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(6,:),[maxPower_altitude_W(6) - TotalPowerReq_W(6,:)].*W2ftlb,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft')
% plot(V_KTAS(11,:),[maxPower_altitude_W(11) - TotalPowerReq_W(11,:)].*W2ftlb,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KTAS(16,:),[maxPower_altitude_W(16) - TotalPowerReq_W(16,:)].*W2ftlb,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft')
% plot(V_KTAS(21,:),[maxPower_altitude_W(21) - TotalPowerReq_W(21,:)].*W2ftlb,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft')
% xlim([30 120])
% ylim([0 10000])
% xlabel('Airspeed (KTAS)')
% ylabel('Excess Power (ft-lb)')
% title('Excess Power at Altitude')
% legend('show','location','best')
% 
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),[maxPower_altitude_W(1) - TotalPowerReq_W(1,:)].*W2hp,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(6,:),[maxPower_altitude_W(6) - TotalPowerReq_W(6,:)].*W2hp,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft')
% plot(V_KTAS(11,:),[maxPower_altitude_W(11) - TotalPowerReq_W(11,:)].*W2hp,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KTAS(16,:),[maxPower_altitude_W(16) - TotalPowerReq_W(16,:)].*W2hp,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft')
% plot(V_KTAS(21,:),[maxPower_altitude_W(21) - TotalPowerReq_W(21,:)].*W2hp,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft')
% xlim([30 120])
% ylim([0 25])
% xlabel('Airspeed (KTAS)')
% ylabel('Excess Power (hp)')
% title('Excess Power at Altitude')
% legend('show','location','best')


% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),TotalPowerReq(1,:).*ftlb2hp,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot([30 130],[maxPower_altitude_W(1).*W2hp maxPower_altitude_W(1).*W2hp],'--','color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(V_KEAS(5,:),TotalPowerReq(5,:).*ftlb2hp,'color',colors(2,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot([30 130],[maxPower_altitude_W(5).*W2hp maxPower_altitude_W(5).*W2hp],'--','color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(V_KEAS(9,:),TotalPowerReq(9,:).*ftlb2hp,'color',colors(3,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot([30 130],[maxPower_altitude_W(9).*W2hp maxPower_altitude_W(9).*W2hp],'--','color',colors(3,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(V_KEAS(13,:),TotalPowerReq(13,:).*ftlb2hp,'color',colors(4,:),'LineWidth',1.5,'DisplayName','12000 ft')
% plot([30 130],[maxPower_altitude_W(13).*W2hp maxPower_altitude_W(13).*W2hp],'--','color',colors(4,:),'LineWidth',1.5,'HandleVisibility','off')
% ylim([0 24])
% xlabel('Airspeed (KEAS)')
% ylabel('Power Required (hp)')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(maxPower_altitude_W.*W2hp,altitudes./1000,'LineWidth',1.5)
% xlabel('Max Power Available (hp)')
% ylabel('Altitude (kft)')




%%

%% Calculate Rates of Climb

maxMotorPower = (Engine.MaxPower_out - Power_electric); % calculating the maximum power available to the motor-prop at each altitude

for jj = 1:length(altitudes)
    for ii = 1:length(V_fps)
        % calculating prop efficiency assuming maximum power available is delivered to the motor/prop
        [Max.AdvanceJ(jj,ii), Max.RPM(jj,ii), Max.PropThrust(jj,ii), Max.Eta(jj,ii), Max.CT(jj,ii), Max.CP(jj,ii)] = Propeller_Mejzlik32x18_Pwr(V_fps(ii), densities(jj), maxMotorPower(jj)*W2ftlb);

        Max.PowerAvailable(jj,ii) = maxMotorPower(jj).*W2ftlb.*Max.Eta(jj,ii); % max power available from the propeller in ft-lbs

        ClimbRate_fps(jj,ii) = (Max.PowerAvailable(jj,ii) - V_fps(ii).*Thrust_req(jj,ii))./(Weight); % finding climb rate from maximum power available and required thrust
        RateofClimb(jj,ii) = ClimbRate_fps(jj,ii).*60;

    end
end


%%

%% Maximum Power Plots

figure;
hold on; grid on; grid minor;
plot(maxPower_altitude_W,altitudes./1000,'LineWidth',1.5,'DisplayName','Shaft (Electrical)')
plot(Engine.MaxPower_out,altitudes./1000,'--','LineWidth',1.5,'DisplayName','Max Dyno Power')
plot(Max.PowerAvailable(:,55)./W2ftlb,altitudes./1000,'LineWidth',1.5,'DisplayName','Propulsive (Physical)')
plot(TotalPowerReq_W(:,55),altitudes./1000,'LineWidth',1.5,'DisplayName','Required')
xlim([0 12000])
xlabel('Power Available (W)')
ylabel('Altitude (kft)')
title('Maximum Power Available at Altitude')
legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),Max.PropThrust(1,:)*2,'LineWidth',1.5,'DisplayName','Available')
% plot(V_KEAS(1,:),Thrust_req(7,:),'LineWidth',1.5,'DisplayName','Required')
% % ylim([0 2000])
% xlabel('Airspeed (KEAS)')
% ylabel('Thrust')
% title('Thrust Available vs Required')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_fps,Thrust_req(1,:).*V_fps,'LineWidth',1.5,'DisplayName','T*V Required')
% plot(V_fps,Max.PowerAvailable(1,:),'LineWidth',1.5,'DisplayName','Pavail')
% % ylim([0 20000])
% xlabel('Airspeed (ft/s)')
% ylabel('Power (ft-lb/s)')
% title('Power Available vs Required')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),Thrust_req(1,:).*V_fps,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
% plot(V_KEAS(1,:),Max.PowerAvailable(1,:),'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
% plot(V_KEAS(6,:),Thrust_req(6,:).*V_fps,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P req')
% plot(V_KEAS(6,:),Max.PowerAvailable(6,:),'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P avail')
% plot(V_KEAS(11,:),Thrust_req(11,:).*V_fps,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P req')
% plot(V_KEAS(11,:),Max.PowerAvailable(11,:),'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P avail')
% plot(V_KEAS(16,:),Thrust_req(16,:).*V_fps,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P req')
% plot(V_KEAS(16,:),Max.PowerAvailable(16,:),'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P avail')
% plot(V_KEAS(21,:),Thrust_req(21,:).*V_fps,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot(V_KEAS(21,:),Max.PowerAvailable(21,:),'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
% xlim([30 130])
% ylim([0 12000])
% xlabel('Airspeed (KEAS)')
% ylabel('Power (ft-lb)')
% title('Power Available vs Required')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),Thrust_req(1,:).*V_fps,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
% plot(V_KTAS(1,:),Max.PowerAvailable(1,:),'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
% plot(V_KTAS(6,:),Thrust_req(6,:).*V_fps,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P req')
% plot(V_KTAS(6,:),Max.PowerAvailable(6,:),'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P avail')
% plot(V_KTAS(11,:),Thrust_req(11,:).*V_fps,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P req')
% plot(V_KTAS(11,:),Max.PowerAvailable(11,:),'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P avail')
% plot(V_KTAS(16,:),Thrust_req(16,:).*V_fps,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P req')
% plot(V_KTAS(16,:),Max.PowerAvailable(16,:),'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P avail')
% plot(V_KTAS(21,:),Thrust_req(21,:).*V_fps,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot(V_KTAS(21,:),Max.PowerAvailable(21,:),'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
% xlim([30 130])
% ylim([0 12000])
% xlabel('Airspeed (KTAS)')
% ylabel('Power (ft-lb/s)')
% title('Power Available vs Required')
% legend('show','location','best')


figure;
hold on; grid on; grid minor;
plot(V_KTAS(1,1:77),Thrust_req(1,1:77).*V_fps(1:77)./W2ftlb./1000,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
plot(V_KTAS(1,:),Max.PowerAvailable(1,:)./W2ftlb./1000,'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
plot(V_KTAS(5,1:76),Thrust_req(5,1:76).*V_fps(1:76)./W2ftlb./1000,'color',colors(2,:),'LineWidth',1.5,'DisplayName','4k ft, P req')
plot(V_KTAS(5,:),Max.PowerAvailable(5,:)./W2ftlb./1000,'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','4k ft, P avail')
plot(V_KTAS(9,1:74),Thrust_req(9,1:74).*V_fps(1:74)./W2ftlb./1000,'color',colors(3,:),'LineWidth',1.5,'DisplayName','8k ft, P req')
plot(V_KTAS(9,:),Max.PowerAvailable(9,:)./W2ftlb./1000,'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','8k ft, P avail')
plot(V_KTAS(21,1:71),Thrust_req(21,1:71).*V_fps(1:71)./W2ftlb./1000,'color',colors(4,:),'LineWidth',1.5,'DisplayName','12k ft, P req')
plot(V_KTAS(21,:),Max.PowerAvailable(21,:)./W2ftlb./1000,'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','12k ft, P avail')
% plot(V_KTAS(21,:),Thrust_req(21,:).*V_fps./W2ftlb/1000,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot(V_KTAS(21,:),Max.PowerAvailable(21,:)./W2ftlb/1000,'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
xlim([40 120])
ylim([0 14])
xlabel('Airspeed (KTAS)')
ylabel('Power (kW)')
title('Propeller Output Power Available vs Required')
legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),Thrust_req(1,:).*V_fps.*ftlb2hp,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P req')
% plot(V_KTAS(1,:),Max.PowerAvailable(1,:).*ftlb2hp,'--','color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft, P avail')
% plot(V_KTAS(6,:),Thrust_req(6,:).*V_fps.*ftlb2hp,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P req')
% plot(V_KTAS(6,:),Max.PowerAvailable(6,:).*ftlb2hp,'--','color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft, P avail')
% plot(V_KTAS(11,:),Thrust_req(11,:).*V_fps.*ftlb2hp,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P req')
% plot(V_KTAS(11,:),Max.PowerAvailable(11,:).*ftlb2hp,'--','color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft, P avail')
% plot(V_KTAS(16,:),Thrust_req(16,:).*V_fps.*ftlb2hp,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P req')
% plot(V_KTAS(16,:),Max.PowerAvailable(16,:).*ftlb2hp,'--','color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft, P avail')
% plot(V_KTAS(21,:),Thrust_req(21,:).*V_fps.*ftlb2hp,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P req')
% plot(V_KTAS(21,:),Max.PowerAvailable(21,:).*ftlb2hp,'--','color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft, P avail')
% xlim([30 130])
% ylim([0 22])
% xlabel('Airspeed (KTAS)')
% ylabel('Power (hp)')
% title('Power Available vs Required')
% legend('show','location','best')
% 
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),ClimbRate_fps(1,:).*Weight,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(6,:),ClimbRate_fps(6,:).*Weight,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft')
% plot(V_KTAS(11,:),ClimbRate_fps(11,:).*Weight,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KTAS(16,:),ClimbRate_fps(16,:).*Weight,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft')
% plot(V_KTAS(21,:),ClimbRate_fps(21,:).*Weight,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft')
% xlim([30 130])
% ylim([0 12000])
% xlabel('Airspeed (KTAS)')
% ylabel('Excess Power (ft-lb/s)')
% title('Excess Power at Altitude')
% legend('show','location','best')
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),ClimbRate_fps(1,:).*Weight./W2ftlb/1000,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(6,:),ClimbRate_fps(6,:).*Weight./W2ftlb/1000,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft')
% plot(V_KTAS(11,:),ClimbRate_fps(11,:).*Weight./W2ftlb/1000,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KTAS(16,:),ClimbRate_fps(16,:).*Weight./W2ftlb/1000,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft')
% plot(V_KTAS(21,:),ClimbRate_fps(21,:).*Weight./W2ftlb/1000,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft')
% xlim([30 130])
% ylim([0 14])
% xlabel('Airspeed (KTAS)')
% ylabel('Excess Power (kW)')
% title('Excess Power at Altitude')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),ClimbRate_fps(1,:).*Weight.*ftlb2hp,'color',colors(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(6,:),ClimbRate_fps(6,:).*Weight.*ftlb2hp,'color',colors(2,:),'LineWidth',1.5,'DisplayName','5k ft')
% plot(V_KTAS(11,:),ClimbRate_fps(11,:).*Weight.*ftlb2hp,'color',colors(3,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KTAS(16,:),ClimbRate_fps(16,:).*Weight.*ftlb2hp,'color',colors(4,:),'LineWidth',1.5,'DisplayName','15k ft')
% plot(V_KTAS(21,:),ClimbRate_fps(21,:).*Weight.*ftlb2hp,'color',colors(5,:),'LineWidth',1.5,'DisplayName','20k ft')
% xlim([30 130])
% ylim([0 22])
% xlabel('Airspeed (KTAS)')
% ylabel('Excess Power (hp)')
% title('Excess Power at Altitude')
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(V_fps,Thrust_req(1,:).*V_fps.*ftlb2hp,'LineWidth',1.5,'DisplayName','T*V Required')
% plot(V_fps,Max.PowerAvailable(1,:).*ftlb2hp,'LineWidth',1.5,'DisplayName','Pavail')
% % ylim([0 20000])
% xlabel('Airspeed (ft/s)')
% ylabel('Power (hp)')
% title('Power Available vs Required')
% legend('show','location','best')


%% Rate of Climb Plots


% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),RateofClimb(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(7,:),RateofClimb(7,:),'LineWidth',1.5,'DisplayName','6k ft')
% plot(V_KEAS(11,:),RateofClimb(11,:),'LineWidth',1.5,'DisplayName','10k ft')
% plot(V_KEAS(13,:),RateofClimb(13,:),'LineWidth',1.5,'DisplayName','12k ft')
% plot(V_KEAS(17,:),RateofClimb(17,:),'LineWidth',1.5,'DisplayName','16k ft')
% plot(V_KEAS(21,:),RateofClimb(21,:),'LineWidth',1.5,'DisplayName','20k ft')
% plot(V_KEAS(29,:),RateofClimb(29,:),'LineWidth',1.5,'DisplayName','24k ft')
% plot(V_KEAS(33,:),RateofClimb(33,:),'LineWidth',1.5,'DisplayName','26k ft')
% plot([30 130],[100 100],'r--','LineWidth',1.5,'DisplayName','Service Ceiling')
% ylim([0 1200])
% xlabel('Airspeed (KEAS)')
% ylabel('Rate of Climb (ft/min)')
% title('Rate of Climb at Altitude')
% legend('show','location','best')

rcplot_title = append('Rate of Climb, Engine Pmax = ',num2str(maxPower),' W, Electric Draw = ',num2str(Power_electric),' W');

figure;
hold on; grid on; grid minor;
plot(V_KTAS(1,:),RateofClimb(1,:),'LineWidth',1.5,'DisplayName','0 ft')
plot(V_KTAS(5,:),RateofClimb(5,:),'LineWidth',1.5,'DisplayName','4k ft')
plot(V_KTAS(9,:),RateofClimb(9,:),'LineWidth',1.5,'DisplayName','8k ft')
plot(V_KTAS(13,:),RateofClimb(21,:),'LineWidth',1.5,'DisplayName','12k ft')
plot(V_KTAS(21,:),RateofClimb(56,:),'LineWidth',1.5,'DisplayName','20k ft')
% plot([20 120],[1000 1000],'--','LineWidth',1.5,'DisplayName','1000 fpm')
plot([20 120],[500 500],'--','LineWidth',1.5,'DisplayName','500 fpm')
plot([20 120],[100 100],'r--','LineWidth',1.5,'DisplayName','100 fpm')
ylim([0 800])
xlabel('Airspeed (KTAS)')
ylabel('Rate of Climb (ft/min)')
title(rcplot_title)
legend('show','location','northeast')

fpm0 = interp1(RateofClimb(21,70:110),V_KTAS(21,70:110),0);
fpm100 = interp1(RateofClimb(21,70:110),V_KTAS(21,70:110),100);
fpm500 = interp1(RateofClimb(21,70:110),V_KTAS(21,70:110),500);
fpm1000 = interp1(RateofClimb(21,70:110),V_KTAS(21,70:110),1000);


%%
% load('ClimbData.mat');
% 
% Assemble.MGTOW = MGTOW;
% Assemble.V_KTAS = V_KTAS;
% Assemble.V_fps = V_fps_TAS;
% Assemble.Altitudes = altitudes;
% Assemble.ClimbRate_fps = ClimbRate_fps;
% Assemble.RateofClimb_fpm = RateofClimb;
% 
% ClimbData.Weight360 = Assemble;
% 
% save('ClimbData.mat','ClimbData')


% BestE_360.Preq = [1160,1210,1260,1390];
% BestR_360.Preq = [1020,1065,1100,1230];

BestE_210.speed = [39.1,41.5,44.1,53.6];
BestR_210.speed = [48.6,51.5,54.8,66.5];

BestE_330.speed = [48.6,51.5,54.8,66.5];
BestR_330.speed = [61.6,65.4,69.5,84.4];

BestE_360.speed = [50.95,54,57.5,69.8];
BestR_360.speed = [64,67.9,72.3,87.6];

idxalt = [1,5,9,21];

for ii = 1:4
    BestE_330.Preq(ii) = interp1(V_KTAS(idxalt(ii),:),V_fps(idxalt(ii)).*Thrust_req(idxalt(ii),:),BestE_330.speed(ii))./W2ftlb;
    BestR_330.Preq(ii) = interp1(V_KTAS(idxalt(ii),:),V_fps(idxalt(ii)).*Thrust_req(idxalt(ii),:),BestR_330.speed(ii))./W2ftlb;

    BestE_210.Preq(ii) = interp1(V_KTAS(idxalt(ii),:),V_fps(idxalt(ii)).*Thrust_req(idxalt(ii),:),BestE_210.speed(ii))./W2ftlb;
    BestR_210.Preq(ii) = interp1(V_KTAS(idxalt(ii),:),V_fps(idxalt(ii)).*Thrust_req(idxalt(ii),:),BestR_210.speed(ii))./W2ftlb;

    BestE_360.Preq(ii) = interp1(V_KTAS(idxalt(ii),:),V_fps(idxalt(ii)).*Thrust_req(idxalt(ii),:),BestE_360.speed(ii))./W2ftlb;
    BestR_360.Preq(ii) = interp1(V_KTAS(idxalt(ii),:),V_fps(idxalt(ii)).*Thrust_req(idxalt(ii),:),BestR_360.speed(ii))./W2ftlb;
end


% BestE_330.Preq = [1160,1210,1260,1390];

% BestR_330.Preq = [1020,1065,1100,1230];

% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),V_fps(1).*Thrust_req(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(5,:),V_fps(5).*Thrust_req(5,:),'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KTAS(9,:),V_fps(9).*Thrust_req(9,:),'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KTAS(21,:),V_fps(21).*Thrust_req(21,:),'LineWidth',1.5,'DisplayName','20000 ft')
% plot(BestE_360.speed,BestE_360.Preq,'x--','LineWidth',1.5,'DisplayName','VBE')
% plot(BestR_360.speed,BestR_360.Preq,'x--','LineWidth',1.5,'DisplayName','VBR')
% % ylim([0 2000])
% xlim([30 130])
% xlabel('Airspeed (KTAS)')
% ylabel('Power Required (ft-lb/s)')
% title('Power Required for SLF, W = 360 lb')
% legend('show','location','best')

% pwrplot_title = append('Power Required for SLF, W = ',num2str(Weight),' lb');
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),V_fps(1).*Thrust_req(1,:)./W2ftlb,'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(5,:),V_fps(5).*Thrust_req(5,:)./W2ftlb,'LineWidth',1.5,'DisplayName','4000 ft')
% plot(V_KTAS(9,:),V_fps(9).*Thrust_req(9,:)./W2ftlb,'LineWidth',1.5,'DisplayName','8000 ft')
% plot(V_KTAS(21,:),V_fps(21).*Thrust_req(21,:)./W2ftlb,'LineWidth',1.5,'DisplayName','20000 ft')
% plot(BestE_360.speed,BestE_360.Preq,'x--','LineWidth',1.5,'DisplayName','VBE')
% plot(BestR_360.speed,BestR_360.Preq,'x--','LineWidth',1.5,'DisplayName','VBR')
% ylim([500 3000])
% xlim([40 120])
% xlabel('Airspeed (KTAS)')
% ylabel('Power Required (W)')
% title(pwrplot_title)
% legend('show','location','northwest')

%% Calculate Max/Service Airspeed

halfset = (length(V_fps))/2;

for jj = 1:length(altitudes)
    for ii = 2:length(V_fps)
        % if isnan(RateofClimb(jj,ii))
        %     RateofClimb(jj,ii) = 0;
        % else
        %     end

        if RateofClimb(jj,ii) > 0
            maxV_fps_TAS(jj) = interp1(RateofClimb(jj,((ii+2):end)),V_fps_TAS(jj,((ii+2):end)),0,'linear','extrap');
            maxV_fps_EAS(jj) = interp1(RateofClimb(jj,((ii+2):end)),V_fps_EAS(jj,((ii+2):end)),0,'linear','extrap');
            break
        else
        end
    end
        
    for ii = 2:length(V_fps)
        if RateofClimb(jj,ii) > 100
            servminV_fps_TAS(jj) = interp1(RateofClimb(jj,((ii+2):end)),V_fps_TAS(jj,((ii+2):end)),100,'linear','extrap');
            servminV_fps_EAS(jj) = interp1(RateofClimb(jj,((ii+2):end)),V_fps_EAS(jj,((ii+2):end)),100,'linear','extrap');

            serviceV_fps_TAS(jj) = interp1(RateofClimb(jj,((ii+2):end)),V_fps_TAS(jj,((ii+2):end)),100,'linear','extrap');
            serviceV_fps_EAS(jj) = interp1(RateofClimb(jj,((ii+2):end)),V_fps_EAS(jj,((ii+2):end)),100,'linear','extrap');
            break
        else
        end
    end
    
    stallV_fps_TAS(jj) = interp1(CL_req(jj,:),V_fps_TAS(jj,:),CL_stall,'spline','extrap');
    stallV_fps_EAS(jj) = interp1(CL_req(jj,:),V_fps_EAS(jj,:),CL_stall,'spline','extrap');
    stallV_KTAS(jj) = stallV_fps_TAS(jj).*ft2kts;
    stallV_KEAS(jj) = stallV_fps_EAS(jj).*ft2kts;

end

maxV_KTAS = maxV_fps_TAS.*ft2kts;
maxV_KEAS = maxV_fps_EAS.*ft2kts;
servminV_KTAS = servminV_fps_TAS.*ft2kts;
servminV_KEAS = servminV_fps_EAS.*ft2kts;
serviceV_KTAS = serviceV_fps_TAS.*ft2kts;
serviceV_KEAS = serviceV_fps_EAS.*ft2kts;
   

%% Plot Flight Envelope

abs_idx = length(maxV_KEAS);
abs_ceiling = altitudes(abs_idx);
service_idx = length(serviceV_KEAS);
service_ceiling = altitudes(service_idx);

% envelope_title = append('Raider 330 Flight Envelope, Engine Pmax = ',num2str(maxPower),' W, Electric Draw = ',num2str(Power_electric),' W');

% figure;
% hold on; grid on; grid minor;
% plot([stallV_KEAS(1), stallV_KEAS(abs_idx)],[altitudes(1), altitudes(abs_idx)]./1000,'color',colors(1,:),'LineWidth',1.5,'DisplayName','Absolute Ceiling')
% plot([stallV_KEAS(1) stallV_KEAS(service_idx)],[altitudes(1) altitudes(service_idx)]/1000,'color',colors(2,:),'LineWidth',1.5,'DisplayName','Service Ceiling')
% 
% plot([BestR.V_KEAS(1) BestR.V_KEAS(1)],[altitudes(1) altitudes(abs_idx)]./1000,'color',colors(3,:),'LineWidth',1.5,'DisplayName','Max Range')
% plot([BestE.V_KEAS(1) BestE.V_KEAS(1)],[altitudes(1) altitudes(abs_idx)]./1000,'color',colors(4,:),'LineWidth',1.5,'DisplayName','Max Endurance')
% 
% plot(maxV_KEAS,altitudes(1:abs_idx)/1000,'color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off')
% plot(serviceV_KEAS,altitudes(1:service_idx)/1000,'color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off')
% 
% plot([stallV_KEAS(abs_idx) maxV_KEAS(end)],[altitudes(abs_idx) altitudes(abs_idx)]./1000,'color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off')
% plot([stallV_KEAS(service_idx) serviceV_KEAS(end)],[altitudes(service_idx) altitudes(service_idx)]./1000,'color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off')
% xlim([30 130])
% ylim([0 25])
% xlabel('Airspeed (KEAS)')
% ylabel('Altitude (kft)')
% title(envelope_title)
% legend('show','location','northeast')

fix = [(altitudes(abs_idx)), (BestR.V_KTAS(abs_idx))];

figure;
hold on; grid on; grid minor;
plot(stallV_KTAS(1:length(maxV_KTAS)),altitudes(1:abs_idx)/1000,'color',colors(1,:),'LineWidth',1.5,'DisplayName','Absolute Ceiling')
plot(stallV_KTAS(1:service_idx),altitudes(1:service_idx)/1000,'color',colors(2,:),'LineWidth',1.5,'DisplayName','Service Ceiling')

plot(BestE.V_KTAS(1:abs_idx-0),altitudes(1:abs_idx-0)/1000,'color',colors(4,:),'LineWidth',1.5,'DisplayName','Max Endurance')
plot(BestR.V_KTAS(1:(abs_idx-4)),altitudes(1:(abs_idx-4))/1000,'color',colors(3,:),'LineWidth',1.5,'DisplayName','Max Range')
% plot([BestR.V_KTAS(abs_idx-1) fix(2)],[altitudes(abs_idx-1) fix(1)]./1000,'color',colors(3,:),'LineWidth',1.5,'HandleVisibility','off')

plot(maxV_KTAS,altitudes(1:abs_idx)/1000,'color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off')
plot(serviceV_KTAS,altitudes(1:service_idx)/1000,'color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off')

plot([stallV_KTAS(abs_idx) maxV_KTAS(end)],[altitudes(abs_idx) altitudes(abs_idx)]./1000,'color',colors(1,:),'LineWidth',1.5,'HandleVisibility','off')
plot([stallV_KTAS(service_idx) serviceV_KTAS(end)],[altitudes(service_idx) altitudes(service_idx)]./1000,'color',colors(2,:),'LineWidth',1.5,'HandleVisibility','off')

xlim([40 120])
ylim([0 25])
xlabel('Airspeed (KTAS)')
ylabel('Altitude (kft)')
title('Raider 330 Predicted Flight Envelope, W = 330 lb')
legend('show','location','northeast')

%% Save Off Best Climb Rates


for jj = 1:1:length(altitudes)
    [Climbs.BestRate(jj), Climbs.idx(jj)] = max(RateofClimb(jj,:));
    Climbs.BestRate_VKEAS(jj) = V_KEAS(jj,Climbs.idx(jj));
    Climbs.BestRate_VKTAS(jj) = V_KTAS(jj,Climbs.idx(jj));

    for ii = 1:length(V_KEAS(jj,:))
        if RateofClimb(jj,ii) < 0
        RateofClimb(jj,ii) = 0;
        end
    end
    ClimbAngle(jj,:) = asind(RateofClimb(jj,:)/60./V_fps_EAS(jj,:));

    [Climbs.BestAngle(jj), Climbs.idxA(jj)] = max(ClimbAngle(jj,:));
    Climbs.BestAngle_VKEAS(jj) = V_KEAS(jj,Climbs.idxA(jj));
    Climbs.BestAngle_VKTAS(jj) = V_KTAS(jj,Climbs.idxA(jj));
end

% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),RateofClimb(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),RateofClimb(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KEAS(9,:),RateofClimb(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KEAS(13,:),RateofClimb(13,:),'LineWidth',1.5,'DisplayName','12k ft')
% plot(V_KEAS(17,:),RateofClimb(17,:),'LineWidth',1.5,'DisplayName','16k ft')
% plot(V_KEAS(21,:),RateofClimb(21,:),'LineWidth',1.5,'DisplayName','20k ft')
% plot(V_KEAS(29,:),RateofClimb(29,:),'LineWidth',1.5,'DisplayName','24k ft')
% plot(V_KEAS(33,:),RateofClimb(33,:),'LineWidth',1.5,'DisplayName','26k ft')
% plot([30 130],[100 100],'r--','LineWidth',1.5,'DisplayName','Service Ceiling')
% xlim([30 130])
% ylim([0 1200])
% xlabel('Airspeed (KEAS)')
% ylabel('Rate of Climb (ft/min)')
% legend('show','location','best')
% 
% figure;
% hold on; grid on; grid minor;
% plot(V_KEAS(1,:),ClimbAngle(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KEAS(5,:),ClimbAngle(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KEAS(9,:),ClimbAngle(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KEAS(13,:),ClimbAngle(13,:),'LineWidth',1.5,'DisplayName','12k ft')
% plot(V_KEAS(17,:),ClimbAngle(17,:),'LineWidth',1.5,'DisplayName','16k ft')
% plot(V_KEAS(21,:),ClimbAngle(21,:),'LineWidth',1.5,'DisplayName','20k ft')
% plot(V_KEAS(29,:),ClimbAngle(29,:),'LineWidth',1.5,'DisplayName','24k ft')
% plot(V_KEAS(33,:),ClimbAngle(33,:),'LineWidth',1.5,'DisplayName','26k ft')
% plot(Climbs.BestAngle_VKEAS(1:37),Climbs.BestAngle(1:37),'--','LineWidth',1.5,'DisplayName','Best Angle')
% % plot([40 120],[100 100],'r--','LineWidth',1.5,'DisplayName','Service Ceiling')
% % ylim([0 2000])
% xlim([30 120])
% xlabel('Airspeed (KEAS)')
% ylabel('Angle of Climb (deg)')
% legend('show','location','best')


p1 = polyfit(altitudes(1:abs_idx),Climbs.BestRate_VKTAS(1:abs_idx),2);
climbrate_VKTAS = polyval(p1, altitudes(1:abs_idx));

p1 = polyfit(altitudes(1:abs_idx),Climbs.BestRate_VKEAS(1:abs_idx),2);
climbrate_VKEAS = polyval(p1, altitudes(1:abs_idx));

p2 = polyfit(altitudes(1:abs_idx),Climbs.BestAngle_VKTAS(1:abs_idx),2);
climbangle_VKTAS = polyval(p2, altitudes(1:abs_idx));

p2 = polyfit(altitudes(1:abs_idx),Climbs.BestAngle_VKEAS(1:abs_idx),2);
climbangle_VKEAS = polyval(p2, altitudes(1:abs_idx));

% load('ClimbData.mat');
% 
% ClimbData.W200.BestRate.Airspeed_KTAS = climbrate_VKTAS;
% ClimbData.W200.BestRate.Airspeed_KEAS = climbrate_VKEAS;
% ClimbData.W200.BestRate.Altitudes = altitudes(1:abs_idx);
% ClimbData.W200.BestRate.ClimbRate = Climbs.BestRate(1:abs_idx);
% 
% ClimbData.W200.BestAngle.Airspeed_KTAS = climbangle_VKTAS;
% ClimbData.W200.BestAngle.Airspeed_KEAS = climbangle_VKEAS;
% ClimbData.W200.BestAngle.ClimbRate = Climbs.BestAngle(1:abs_idx);
% 
% ClimbData.W200.Altitudes = altitudes(1:abs_idx);
% 
% save('ClimbData.mat','ClimbData');
% envelope_title = append('Raider 330 Best Climb Airspeeds, W = ',num2str(Weight),' lbs, ',num2str(spinningprops),' Propulsors');

% figure;
% hold on; grid on; grid minor;
% plot(ClimbData.W330.BestRate.Airspeed_KEAS,ClimbData.W330.Altitudes./1000,'LineWidth',1.5,'DisplayName','Best Rate')
% plot(ClimbData.W330.BestAngle.Airspeed_KEAS,ClimbData.W330.Altitudes./1000,'LineWidth',1.5,'DisplayName','Best Angle')
% % ylim([0 30])
% xlabel('Airspeed (KEAS)')
% ylabel('Altitude (kft)')
% title(envelope_title)
% legend('show','location','best')
% 
% 
% figure;
% hold on; grid on; grid minor;
% plot(ClimbData.W330.BestRate.Airspeed_KTAS,ClimbData.W330.Altitudes./1000,'LineWidth',1.5,'DisplayName','Best Rate')
% plot(ClimbData.W330.BestAngle.Airspeed_KTAS,ClimbData.W330.Altitudes./1000,'LineWidth',1.5,'DisplayName','Best Angle')
% % ylim([0 30])
% xlabel('Airspeed (KTAS)')
% ylabel('Altitude (kft)')
% title(envelope_title)
% legend('show','location','best')

% figure;
% hold on; grid on; grid minor;
% plot(Climbs.BestAngle_VKTAS,Climbs.BestAngle,'LineWidth',1.5,'DisplayName','TAS')
% plot(Climbs.BestAngle_VKEAS,Climbs.BestAngle,'LineWidth',1.5,'DisplayName','EAS')
% xlabel('Airspeed (kts)')
% ylabel('Climb Angle (deg)')
% title('Best Climb Angle')
% legend('show','location','best')


% figure;
% hold on; grid on; grid minor;
% plot(Climbs.BestRate,altitudes/1000,'LineWidth',1.5)
% xlabel('Best Rate of Climb (ft/min)')
% ylabel('Altitude (kft)')


% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),RateofClimb(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(5,:),RateofClimb(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(9,:),RateofClimb(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KTAS(13,:),RateofClimb(13,:),'LineWidth',1.5,'DisplayName','12k ft')
% plot(V_KTAS(17,:),RateofClimb(17,:),'LineWidth',1.5,'DisplayName','16k ft')
% plot(V_KTAS(21,:),RateofClimb(21,:),'LineWidth',1.5,'DisplayName','20k ft')
% plot(V_KTAS(29,:),RateofClimb(29,:),'LineWidth',1.5,'DisplayName','24k ft')
% plot(V_KTAS(33,:),RateofClimb(33,:),'LineWidth',1.5,'DisplayName','26k ft')
% plot([40 120],[100 100],'r--','LineWidth',1.5,'DisplayName','Service Ceiling')
% ylim([0 2000])
% xlabel('Airspeed (KTAS)')
% ylabel('Rate of Climb (ft/min)')
% legend('show','location','best')



% figure;
% hold on; grid on; grid minor;
% plot(V_KTAS(1,:),ClimbAngle(1,:),'LineWidth',1.5,'DisplayName','0 ft')
% plot(V_KTAS(5,:),ClimbAngle(5,:),'LineWidth',1.5,'DisplayName','4k ft')
% plot(V_KTAS(9,:),ClimbAngle(9,:),'LineWidth',1.5,'DisplayName','8k ft')
% plot(V_KTAS(13,:),ClimbAngle(13,:),'LineWidth',1.5,'DisplayName','12k ft')
% plot(V_KTAS(17,:),ClimbAngle(17,:),'LineWidth',1.5,'DisplayName','16k ft')
% plot(V_KTAS(21,:),ClimbAngle(21,:),'LineWidth',1.5,'DisplayName','20k ft')
% plot(V_KTAS(29,:),ClimbAngle(29,:),'LineWidth',1.5,'DisplayName','24k ft')
% plot(V_KTAS(33,:),ClimbAngle(33,:),'LineWidth',1.5,'DisplayName','26k ft')
% % plot([40 120],[100 100],'r--','LineWidth',1.5,'DisplayName','Service Ceiling')
% % ylim([0 2000])
% xlabel('Airspeed (KTAS)')
% ylabel('Angle of Climb (deg)')
% legend('show','location','best')

