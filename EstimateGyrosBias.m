% MTRN4010 Project 2 - Part A: Gyroscope Bias Estimation via Optimization
% Estimates 3D gyroscope bias using fminsearch over a 5-second trajectory.
% Mode 1 (A1): complete attitude measurements.
% Mode 2 (A2): attitude measurements with missing (NaN) components.
%
% z5610741
% Darshan Komala Sreeramu
%
% Reference: https://au.mathworks.com/help/matlab/ref/fminsearch.html
% Reference: ZYX Euler kinematic equations - MTRN4010 Lectures / Project 1

function [ok, BiasVector, extra] = EstimateGyrosBias(Data, Mode)
    ok= -1;
    BiasVector=[];
    extra =[];
    
    % Cost function based on mode
    if Mode== 1 % A1
        costFn= @(b) costA1(b,Data);
    else % A2
        costFn =@(b) costA2(b ,Data);
    end
    
    % Initial guess - zero bias
    initBias= [ 0,0, 0];
    
    % Nelder-Mead optimizer
    opts =optimset('TolX',1e-6, 'TolFun',1e-6,'MaxIter',2000, 'MaxFunEvals',5000);
    [ bOpt,~, exitflag] =fminsearch(costFn,initBias, opts);
    
    if exitflag >= 0
        ok = 1;
    end
    
    BiasVector =bOpt(:);%3x1     
    fullAtt= propagateAttitude(Data,BiasVector);
    extra.EstimatedFullAttitude =fullAtt;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost =costA1(bias,Data )
    % A1: all attitude components are valid.
    fullAtt= propagateAttitude(Data,bias );%(degrees)
    predicted =fullAtt(:,Data.sampleIndexesA);
    measured=double(Data.MeasuredAttitudes);
    
    diff =predicted -measured;
    cost= sum( diff(: ).^2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost= costA2(bias, Data)
    %Same as A1 but skip NaN entries. Only penalise valid components.
    fullAtt= propagateAttitude( Data, bias);
    predicted =fullAtt( : , Data.sampleIndexesA);
    measured=double( Data.MeasuredAttitudes);
    
    validMask= ~isnan(measured );
    if sum(validMask( :) )== 0
        cost =0;
        return;
    end
    
    diff= predicted(validMask ) -measured(validMask);
    cost =sum( diff.^2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function fullAtt=propagateAttitude( Data, bias)
    % Integrates bias-compensated gyros from InitialAttitude
    % Reference: Project-1 PartB
    kr=pi/180; 
    kd= 180/pi;
    bias =double(bias( :) );
    
    gyros =double( Data.GyrosNoisyMesurements);
    tt= double(Data.timestamps );
    Ng=Data.numOfGyroscopesSamples;
    att= double( Data.InitialAttitude(: ))*kr;
    
    fullAtt= zeros(3,Ng);
    fullAtt( :, 1)= att* kd;
    
    % Subtract bias from all gyro samples at once
    gyrosCorrected= gyros-bias;
    for i= 2:Ng
        dt= tt(i) -tt(i-1);
        w =gyrosCorrected(:,i-1)* kr;%rad/s
    
        phi=att(1 );
        theta= att( 2);
        cosTheta =cos(theta);
        if abs( cosTheta)< 1e-6
            cosTheta =sign(cosTheta) *1e-6;
        end
    
        % ZYX Euler kinematic eqns
        phiDot=w(1) +(w(2)* sin(phi) + w( 3) *cos(phi))*tan( theta);
        thetaDot =w(2)*cos(phi) -w(3)* sin(phi);
        psiDot = (w(2)*sin(phi)+ w(3 )* cos(phi)) /cosTheta;
    
        att =att+ dt *[phiDot ; thetaDot;psiDot];
        fullAtt(:,i) = att* kd;
    end
end
