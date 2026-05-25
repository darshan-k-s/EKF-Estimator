% InferWhichWall.m
% Part B
% z5610741
% Darshan Komala Sreeramu
%
% Known normals in GCF:
%   Floor   : [0; 0; 1]   (plane=1)
%   Wall 1  : [-1; 0; 0]  (plane=2)
%   Door    : [0; 1; 0]    (plane=3)
%   Wall 2  : [0; -1; 0]  (plane=4)
%   Ceiling : [0; 0; -1]  (plane=5)

function plane = InferWhichWall(normalVec, currAttGuess, tol)
    knownNorms =[ 0,  0,  1;
                  -1,  0,  0;
                   0,  1,  0;
                   0, -1,  0;
                   0,  0, -1]';

    phi= currAttGuess(1);  % roll  (rad)
    theta =currAttGuess(2);   % pitch (rad)
    psi = currAttGuess(3);  % yaw   (rad)

    R_z =[cos(psi), -sin(psi),  0;
      sin(psi),  cos(psi),  0;
      0,         0,         1];

    R_y= [cos(theta),  0,  sin(theta);
      0,           1,  0;
     -sin(theta),  0,  cos(theta)];
    R_x =[1,    0,         0;
          0,    cos(phi), -sin(phi);
          0,    sin(phi),  cos(phi)];

    R= R_z * R_y * R_x;

    globalNormal= R *normalVec(:);

    bestAngle= inf;
    plane = 0;
    bestPlane = 0;


    for i= 1:5
        refN= knownNorms(:, i);
        dotProd = max(-1, min(1, dot(globalNormal, refN)));
        angleDeg =acos(dotProd) * (180/pi);
        if angleDeg <tol && angleDeg <bestAngle
            bestAngle= angleDeg;
            bestPlane = i;
        end
    end

    plane= bestPlane;
end
