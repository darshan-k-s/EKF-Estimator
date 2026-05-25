% P2PartD_Main.m
% MTRN4010 Project 2: Part D
% z5610741
% Darshan Komala Sreeramu
function P2PartD_Main()
main1();
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function main1()
    FFF= zeros(20,1);
    FFF(1) =1;% run flag
    FFF(2)=1 ;% paused flag

    Api=xAPI4010v06();

    % Load dataset%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    s ='.\dataSets\HH04\';
    [ok,info ] =Api.LoadDataSet(s );
    if (ok< 0), return;end
    disp(info );
    fprintf('Dataset: %d events, duration=[%.2fs]\n',info.numEvents,info.Duration );


    % Inject artificial gyro bias
    Api.SetFictitiousExtraGyrosBiasses([-1 ;1.5;-1]);%deg/s

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    % Oscilloscopes for live monitoring 
    [~,hhOAtt]= Api.IniScopesNChannels(3,33,500,[-100,100], 'EKF Attitude: Yours (red) vs API (blue)', {'Roll (deg)','Pitch (deg)','Yaw (deg)'}); 
    cAttX =1; 
    set(hhOAtt,'LineWidth', 2,'Color',[0 0 1] );%API blue 
    [~,hhOBias ] =Api.IniScopesNChannels(3,35 ,800,[-3,3 ],'Gyro Biases (deg/s) - Yours', {'bx','by','bz'}); 
    cBiasX=1; set(hhOBias,'LineWidth',2 ); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    % Full-history figures
    figure(40); clf(); subplot(311); 
    hRollAPI =plot(0,0 ,'b-', 'LineWidth',2 ); 
    hold on; 
    hRollOurs= plot(0, 0,'r--','LineWidth', 1.5); ylabel('Roll (deg)'); grid on; 
    legend('API','Yours' ,'Location', 'best'); 
    title('Part D: EKF Attitude - Yours (red dashed) vs API (blue)'); 
    
    subplot(312 ); 
    hPitchAPI= plot(0, 0,'b-','LineWidth', 2); 
    hold on; 
    hPitchOurs=plot( 0,0,'r--', 'LineWidth',1.5);ylabel('Pitch (deg)' ); grid on; 
    
    subplot(313); 
    hYawAPI=plot(0, 0,'b-','LineWidth', 2);hold on; 
    hYawOurs= plot( 0,0,'r--','LineWidth',1.5 ); 
    ylabel('Yaw (deg) '); xlabel(' Time (s)');grid on; figure(41);clf() ; 
    
    subplot(311);
    hErrRoll=plot(0, 0,'m','LineWidth', 1.5);ylabel('Roll err(deg)'); grid on; 
    title('Part D: Attitude Error (Yours - API)'); 
    
    subplot( 312);
    hErrPitch = plot( 0,0,'m','LineWidth',1.5 ); ylabel('Pitch err(deg)');grid on; 
    
    subplot(313); 
    hErrYaw= plot(0, 0,'m','LineWidth', 1.5);ylabel('Yaw err(deg)' ); 
    xlabel('Time (s)' );grid on; 
    figure(42); 
    clf(); 
    
    subplot(311);
    hBxApi =plot(0,0, 'b-','LineWidth' ,2);
    hold on; 
    hBx= plot( 0,0,'r--', 'LineWidth',1.5);ylabel('bx (deg/s)'); 
    legend( 'API', 'Yours','Location','best' ); 
    title('Part D: Estimated Gyro Biases-Yours(red dashed) vs API(blue)' );grid on; 
    
    subplot(312);
    hByApi =plot(0, 0,'b-','LineWidth', 2 );hold on; 
    hBy=plot(0, 0,'r--','LineWidth', 1.5);ylabel('by (deg/s)' ); grid on; 
    
    subplot(313);
    hBzApi=plot(0, 0,'b-','LineWidth' ,2);hold on; 
    hBz =plot( 0,0 ,'r--' ,'LineWidth',1.5 );
    ylabel('bz (deg/s)'); xlabel('Time(s)');grid on ; 
    
    % Image figures 
    figure(21);clf() ; 
    
    subplot(211);hiRGB=image(0 );
    axis( [0,320 ,0, 240]);hTitleRGB= title('Last RGB frame'); 
    ca =gca();ca.XDir= 'reverse'; 
    hold on; 
    a=160;b= 230;
    plot([a, b,b,a ,a],[a,a ,b, b,a],'r' ,'LineWidth',3 );hold off; 
    subplot(212);
    hiDepth =imagesc(0);axis([0,320, 0,240] );title('Last Depth frame'); 
    ca =gca();ca.XDir='reverse';

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Control GUI 
    hh =MkMenuInFigure(21);
    gyrosLast= [0;0;0];
    lastT= 0;
    t0=0 ;
    firstPredDone =false;

    fprintf('EKF running. Press On/Off to start.\n' );

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % EKF state initialisation
    varianceNoiseGyros=(1.0* pi/ 180)^2;
    stdNormalMeasurement =0.05;
    
    %Augmented state:[roll ;pitch;yaw; bx;by;bz](rad, rad/s)
    
    Xa= [0;0; 0;0;0 ;0];
    stds0 =[10 ;10;20;10;10; 10] *pi/180;
    Pa=diag(stds0.^2);
    Xb= [0; 0;0; 0;0 ;0];
    Pb =diag(stds0.^2);


    % History buffers
    apiAttHist=zeros(3, 0);
    ourBiasHist =zeros( 3,0);
    apiBiasHist =zeros(3,0 );
    timeHist = [];
    myAttHist= zeros(3,0);

    nSamp=0;

    toDeg =180/pi;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MAIN EVENT LOOP
    while(FFF(1))
        drawnow limitrate;
        if (FFF(2)), pause(0.4);continue; end

        r =Api.getMeasurement();
        if isempty(r), break;end


        t= double(r.t)* 0.0001;
        dts =t- lastT;
        lastT = t;

        % PREDICTION STEP
        if dts> 0 && firstPredDone
            [Xb, Pb] =myPredictionStep(Xb,Pb, dts, gyrosLast,varianceNoiseGyros,Api);
            [Xa, Pa]= Api.PredictionStepPartD(Xa, Pa,dts, gyrosLast , varianceNoiseGyros,[0,0,0 ,0, 0,0]);
        end

        if((r.t- t0)> 1500), pause(0.02); t0 =r.t; FFF(10)=1; end

        switch(r.ID)
            case 1 % IMU
                firstPredDone =true;
                gyrosLast= r.Data(4:6);
                cAttX =Api.PushDataInScopeChannels(hhOAtt, cAttX, Xb(1:3)*toDeg);
                nSamp= nSamp +1;
                timeHist(nSamp)= t;
                ourBiasHist(:,nSamp)=Xb(4:6) * toDeg;
                apiBiasHist(:,nSamp)=Xa(4:6)* toDeg;
                myAttHist(:,nSamp) =Xb(1:3) *toDeg;
                apiAttHist(:,nSamp)= Xa(1:3)* toDeg;
                

                if mod(nSamp,150) ==0
                    drawnow;
                end
              continue;

            case 2  % Depth
                hiDepth.CData=r.Data;
                [~, xx,yy,zz ] =Api.Get3DPointsFromROI([160, 230],[160 ,230],r.Data,1);
                [zz,xx]= Api.Rota2D(zz,xx, 20*pi/ 180);
                [okP,vn,~,~ , ~] =Api.ApproxPlaneFromPoints(xx,yy, zz, 20) ;
                fprintf('okP=%d |t=%.2f | vn=[%.2f,%.2f,%.2f]\n',t, okP,vn);
                if okP >0
                    [Xb, Pb,~] =  myUpdateStep(Xb, Pb, vn,stdNormalMeasurement);
                    [Xa,Pa, ~] =Api.ExeUpdatePartD(Xa,Pa, vn, stdNormalMeasurement);
                end
                cBiasX =Api.PushDataInScopeChannels(hhOBias, cBiasX, Xb(4:6)*toDeg);
                if nSamp> 0
                    tt =timeHist(1:nSamp );
                    hRollOurs.XData   = tt;hRollOurs.YData= myAttHist(1,1:nSamp );
                    hPitchOurs.XData = tt; hPitchOurs.YData =myAttHist(2,1:nSamp);
                    hYawOurs.XData = tt;hYawOurs.YData= myAttHist(3,1:nSamp);
                    hRollAPI.XData = tt; hRollAPI.YData= apiAttHist(1,1:nSamp);
                    hPitchAPI.XData= tt; hPitchAPI.YData = apiAttHist(2, 1:nSamp);
                    hYawAPI.XData= tt; hYawAPI.YData =apiAttHist(3,1:nSamp);
                    err =myAttHist(:,1:nSamp) -apiAttHist(:,1:nSamp);
                    hErrRoll.XData= tt; hErrRoll.YData= err(1,: );
                    hErrPitch.XData =tt; hErrPitch.YData = err(2,:);
                    hErrYaw.XData= tt; hErrYaw.YData=err(3,:);
                    hBx.XData= tt; hBx.YData= ourBiasHist(1, 1:nSamp);
                    hBy.XData =tt; hBy.YData =ourBiasHist(2,1:nSamp);
                    hBz.XData=tt;hBz.YData= ourBiasHist(3,1:nSamp);
                    hBxApi.XData= tt; hBxApi.YData= apiBiasHist(1,1:nSamp);
                    hByApi.XData =tt;hByApi.YData =apiBiasHist(2,1:nSamp);
                    hBzApi.XData= tt; hBzApi.YData=apiBiasHist(3, 1:nSamp);
                end
                continue;

            case 3 % RGB
                hiRGB.CData =r.Data;
                if(FFF(10))
                    FFF(10)= 0;
                    set(hTitleRGB,'string', sprintf('RGB at t=[%.3fs]',t)) ;
                end
                continue;



            case 250
                PausePlayback();
                lastT = 0;
                firstPredDone = false;
                fprintf('Jump in time.\n'); t0 = r.t;
                gyrosLast = [0;0;0]; 
                continue;

            case 255
                fprintf('End of dataset.\n'); 
                break;
        end
    end
    delete(hh);
    disp('BYEEEEEEEEE') ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function PausePlayback()
        FFF(2) = 1 ;
        disp('Paused. Press On/Off to continue.');
    end

    function MyCallbackEND(~,~ )
        FFF(1)= 0;
    end

    function MyCallbackTogglePause(~,~)
        FFF(2) =1 -FFF(2 );
        fprintf('flagPaused=[%d]\n',FFF(2) );
    end

    function MyCallbackJmpTo0(~,~)
        Api.JmpToTime(0);
        Xb =zeros(6,1); 
        Xa= zeros(6,1);
        stds0=[10;10;20;2;2;2]* pi/180;
        Pb = diag(stds0.^2);
        Pa= diag(stds0.^2 );
        gyrosLast =zeros(3,1); 
        lastT= 0;
        firstPredDone=false ;
        cAttX =1; 
        cBiasX = 1;
        nSamp = 0;
        timeHist= []; 
        myAttHist = zeros(3,0); apiAttHist = zeros(3,0);
        ourBiasHist = zeros(3,0); apiBiasHist =zeros(3,0);
    end

    function hhOut= MkMenuInFigure(ThisFigure)
        figure(ThisFigure);
        currY= 1; hy = 20;px = 10; hyb = hy*1.1 ; ddx =60;
      
        hhOut(4)= CreateMyButton('On/Off',[px,currY,ddx,hy], @MyCallbackTogglePause); 
        currY = currY+hyb;
        hhOut(3)= CreateMyButton('END',  [px,currY,ddx,hy], @MyCallbackEND); 
        currY = currY+hyb;
        hhOut(2)=CreateMyButton('Go to t0',[px,currY,ddx,hy], @MyCallbackJmpTo0);

        set(hhOut(4), 'BackgroundColor', [0,  1, 0.2]);
        set(hhOut(3),'BackgroundColor', [1,0,0.2]);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % EKF PREDICTION STEP(C1/C5)
    function [Xnew,Pnew] = myPredictionStep(X, P, dt, w, varW, ApiRef)
        bx= X(4); by= X(5);bz= X(6);
        phi = X(1);theta = X(2);

        wx= w(1) -bx;
        wy= w(2) - by;
        wz=w(3) - bz;

        ct= cos(theta);
        if abs(ct) < 1e-4, ct =sign(ct + 1e-10) *1e-4; end

        dPhi  =wx + (wy*sin(phi) + wz*cos(phi)) * tan(theta);
        dTheta=wy*cos(phi) -wz*sin(phi);
        dPsi= (wy*sin(phi)+ wz*cos(phi)) /ct;

        Xnew = X;
        Xnew(1) =X(1) + dt * dPhi;
        Xnew(2) = X(2) +dt *dTheta;
        Xnew(3)= X(3) + dt * dPsi;
        % biases unchanged

        [Jx,~] = ApiRef.EvaluateAugStateEquationJacobians(X,w, dt);

        Ju =zeros(6,3);
        Ju(1,1)= dt;
        Ju(1,2)= dt * sin(phi) * tan(theta);
        Ju(1,3)= dt * cos(phi) *tan(theta);
        Ju(2,2)= dt * cos(phi);
        Ju(2,3)= dt *(-sin(phi));
        Ju(3,2)= dt * sin(phi) / ct;
        Ju(3,3)= dt * cos(phi) /ct;

        Q= Ju *(varW * eye(3))* Ju';
        Pnew= Jx * P * Jx' +Q;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    % EKF UPDATE STEP(C3/C4)
    function [Xnew,Pnew, okUpdate] =myUpdateStep(X, P,vn, stdC)
        Xnew= X; Pnew =P; okUpdate= 0;
        phi = X(1);theta = X(2); psi =X(3);
        attDeg= [phi, theta, psi] *180/pi;
    
        % Direct geometric check first
        vn= vn(:) / norm(vn);
        tol = 35; %deg
        cosT =cos(tol * pi/180);
    
        if vn(3)> cosT
            plane =1; % floor
        elseif vn(3) < -cosT
            plane= 5; %ceiling
        else
            plane =InferWhichWall(vn, attDeg, tol); % walls need attitude
        end

    
        fprintf('myUpdateStep: plane=%d\n',plane);
        if plane ==0, return; end

        switch plane
            case 1 % FLOOR [0;0;+1]
                h= [ -sin(theta);
                       sin(phi)*cos(theta);
                       cos(phi)*cos(theta) ];
                H =zeros(3,6);
                H(1,2)= -cos(theta);
                H(2,1) = cos(phi)*cos(theta);
                H(2,2)= -sin(phi)*sin(theta);
                H(3,1)= -sin(phi)*cos(theta);
                H(3,2)= -cos(phi)*sin(theta);

            case 2 % WALL1 [-1;0;0]
                h =[ -cos(theta)*cos(psi);
                      -(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi));
                      -(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) ];
                H =zeros(3,6);
                H(1,2)=  sin(theta)*cos(psi);
                H(1,3)= cos(theta)*sin(psi);
                H(2,1)= -(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
                H(2,2)= -(sin(phi)*cos(theta)*cos(psi));
                H(2,3) = (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi));
                H(3,1) = (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi));
                H(3,2) = -(cos(phi)*cos(theta)*cos(psi));
                H(3,3)= (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) );

            otherwise
                return;
        end

        okUpdate= plane;
        
        z = vn(:);
        R = (stdC^2) *eye(3);

        y = z - h;
        S = H * P * H' + R;
        K = (P * H') /S;
        Xnew= X + K * y;
        Pnew = (eye(6) - K*H) *P;
        
    end

end  % main1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h =CreateMyButton(strBla,position, MyCallback)
    h= uicontrol('Style','pushbutton','String',strBla, 'Position',position ,'Callback',MyCallback);
end