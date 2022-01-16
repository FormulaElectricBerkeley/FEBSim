function EffTime = EfficiencyCalc(OLResults, EffGraph,numMotors, lapsim)
    %Script to calculate overal car power consumption given OptimumLap 
    %results. Factors in motor innefficiency map that OL does not support.



    %Parameters
    %OLResults: Matrix from OptimumLap with columns, Time, RPM, Torque, and Brake Power
    %EffGraph: Matrix made from motor curves with columns, RPM, Torque, and Efficiency
    %numMotors: Number of Motors
    %lapsim: Results from FEB trackLapModel

    %FEB trackLapModel import (comment code out if using OptimumLap Results)
    OLResults = cat(2, lapsim(:,1), lapsim(:,2)/(.254)*3.54*2*pi/60, lapsim(:,5)./4, zeros(length(lapsim),1));
        
    EffRPM = EffGraph(:,1);
    EffTorque = EffGraph(:,2);
    Eff = EffGraph(:,3);
    
    OLTime = OLResults(:,1);        %time (s)
    OLRPM = OLResults(:,2);         %angular velocity (rad/s)
    OLTorque = OLResults(:,3);      %Torque (Nm)
    OLCharging = OLResults(:,4);    %Battery Power (Kw)
    
    RegenEff = .9;                  %Drivetrain Efficiency for regen    
    TrackLength = 1.069;            %Length of One Lap (km)
    InvEff = 0.97;                  %Inverter Efficiency


    %scatteredInterpolant uses bicubic interpolation and linear extrapolation
    InterpolEff = scatteredInterpolant(EffRPM/20,EffTorque,Eff,"natural","linear");
    EffTime = InterpolEff(OLRPM*9.5493/20,OLTorque);
    
    
    %Interpolating/Extrapolating eff at each time using biharmonic method
    %{
    count = 1:length(OLResults);
    EffTime = zeros(length(count),1);
    for n = count
        EffTime(n) = griddata(RPM,Torque,Eff,OLResults(n,2),OLResults(n,3)/numMotors,"v4");
    end
    %}
    
    
    %TimeStep
    TimeStep = zeros(length(OLTime),1);
    for n = 1:(length(OLTime)-1)
        TimeStep(n) = OLTime(n+1) - OLTime(n);
    end
    
    %Battery Efficiency
    OLCharging = OLCharging.*EffTime/100*RegenEff*InvEff/2;
    
    %Tot Energy Calc
    Energylap = sum(OLRPM.*OLTorque./(EffTime/100).*TimeStep/InvEff-OLCharging)/(10^6);
    Energytot = Energylap*(22/TrackLength);
    
    disp("    Charging tot: " + sum(OLCharging)/(10^6)*22/TrackLength*0.2777);
    disp("    MJ: " + Energytot + newline + "    kWh: " + Energytot*0.27777);
end