function [BestE,BestR] = FindBestAirspeeds(Weight,density,sigma,CL,CD,Sref)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
ft2kts = 0.592484;

Vfps_vec = [50:0.25:72,73:1:79,80:2:200];

for ii = 1:length(Vfps_vec)
        CL_req(ii) = Weight./(0.5.*density.*(Vfps_vec(ii).^2).*Sref); % finding CL range using weight, lbs & ft
        CD_gen(ii) = interp1(CL(1:21),CD(1:21),CL_req(ii)); % finding CD using trim polar
        Thrust_req(ii) = CD_gen(ii).*(0.5.*density.*(Vfps_vec(ii).^2).*Sref); % finding thrust from drag coeff, lbs
        Endur_metric(ii) = (CL_req(ii).^(3/2))./CD_gen(ii);
        Range_metric(ii) = CL_req(ii)./CD_gen(ii);
end

V_fps_EAS = Vfps_vec.*sqrt(sigma);
V_fps_TAS = Vfps_vec;
V_KEAS = V_fps_EAS.*ft2kts;
V_KTAS = V_fps_TAS.*ft2kts;

[BestE.Endur, BestE.idx] = max(Endur_metric);
[BestR.Range, BestR.idx] = max(Range_metric);

BestE.V_fps_EAS = V_fps_EAS(BestE.idx);
BestR.V_fps_EAS = V_fps_EAS(BestR.idx);
BestE.V_KEAS = V_KEAS(BestE.idx);
BestR.V_KEAS = V_KEAS(BestR.idx);

BestE.V_fps_TAS = BestE.V_fps_EAS./sqrt(sigma);
BestR.V_fps_TAS = BestR.V_fps_EAS./sqrt(sigma);
BestE.V_KTAS = BestE.V_KEAS./sqrt(sigma);
BestR.V_KTAS = BestR.V_KEAS./sqrt(sigma);

end