function TestMyVectorInference()

% for testing your plane identifier.
% JEG/MTRN4010

clc();
MaxErrorAttitudeDeg=15;     % maximum discrepancy between assumed and actual attitude    
toleranceDiffVectoDeg=25;   % tolerance of discrepancy in directions of compared vectors. 

   kr=pi/180;

    amaxDeg = 85;   %  range of attitude angles being randomly tested in this test program
    % from [-amaxDeg to +amaxDeg] for roll, pitch, yaw. (in degrees)

    nG = [0; 0; 1] ;     % actual vector in GCF.
    % you can change this one, to choose any case you may like to try
    %nG = [+1; 0; 0] ;   
    %nG = [ 0;-1; 0] ;   
    %nG = [ 0;+1; 0] ;   
    %nG = [ 0; 0;-1] ;   
    %nG = [ 0; 0;+1] ;   


    
    % if your detector is working well, it should always infer this vector.
    % except in cases in which the discrepancy were larger than the specified tolerance.


    % here, we try, randomly, for different attitudes and for different
    % discrepancies betwwen assumed and actual attitude.
    for i=1:100,
        
        Ad = (2*rand(3,1)-1)*amaxDeg;   % some random attitude. 
        % It will be our "actual attitude".

        Noise1 = (2*rand(3,1)-1);
        errorAttitudeRad = (MaxErrorAttitudeDeg*kr)*Noise1;
        % some random errors for the "assumned attitude".
        
        Ar = Ad*kr;                   % "actual  attitude", in radians
        ArNoisy =Ar+errorAttitudeRad; % "assumed attitude", in radians.
        R = RotationMatrix3D(Ar);
        nL = R'*nG;                 % this the actual normal vector we measure locally.
    
        
        % function to test(e.g., you may use your function). 
        % [ID,s,angleDiffDeg]=InferUsefulGlobalVector(nL,ArNoisy,toleranceDiffVectoDeg) ; 
        % we lie to the function, providing the "assumed attitude"
        % see [note1] about InferUsefulGlobalVector()
        ThisOne = InferWhichWall(nL, ArNoisy, toleranceDiffVectoDeg);
        ID = ThisOne;
        % Map your ID to a name string
        names = {'none','floor','wall1','door','wall2','ceil'};
        s = names{ThisOne + 1};
        % Compute angleDiffDeg manually for printing
        knownNormals = [0,0,+1; -1,0,0; 0,+1,0; 0,-1,0; 0,0,-1];
        R = RotationMatrix3D(ArNoisy);
        nGCF = R * nL; nGCF = nGCF/norm(nGCF);
        if ThisOne > 0
            nRef = knownNormals(ThisOne,:)';
            angleDiffDeg = acos(max(-1,min(1,dot(nGCF,nRef)))) * (180/pi);
        else
            angleDiffDeg = Inf;
        end

        % you should adapt these argument/parameters to the convention and structure  used in your function
        % (the one specified in the item of Project2)

        % input arguments of InferUsefulGlobalVector()
           % nL : input vector (seen locally)
           % ArNoisy : our (not accurate) assumed platform attitude (in radians).
           % toleranceDiffVectoDeg: maximum accepted discrepancy in directions of associated vectors (in degrees)
            

        % output
        % ID : ID of global normal vector associated to our local vector nL. 
        % ID<1 means NO association.
        % s  : string specifying the associated / inferred global vector.
        % angleDiffDeg : angle between predicted global vector and the inferred one (in degrees)   
        % if the association was successful, then : angleDiffDeg < toleranDiffVectoDeg
        
        % We give, to the inference function, the the noisy/assumed attitude (not the actual one), 
        % as it would happen in reality.

        fprintf('ID=[%d],%s. [%.1fdeg]\n',ID,s,angleDiffDeg);

    end
    

end    


%note1:

% this function "InferUsefulGlobalVector()" considers all canonical vectors
% '[0;0;+1]','[0;0;-1]',  '[-1;0;0]','[+1;0;0]','[0;-1;0]','[0;+1;0]';
%  these vectors do have IDs [1,2,3,4,5,6] 

% Your ID convenction (required in the related item of Project2) may be different).

% in our "natural language"
% '[0;0;+1] (floor)','[0;0;-1] (ceil)', '[-1;0;0](wall1)','[+1;0;0]','[0;-1;0](wall2)','[0;+1;0](door)';

%------------------------------------------------------------------