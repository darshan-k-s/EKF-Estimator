% MTRN4010 Project 2 - Part A Demo Runner
% z5610741 - Darshan Komala Sreeramu

function P2PartA_Main()
    addpath(fileparts(mfilename('fullpath')));

    fprintf('========== PART A1: Complete Attitude Data ==========\n');
    RunTest(1);

    fprintf('\n========== PART A2: Missing Attitude Data (NaN) ==========\n');
    RunTest(2);
end

% ----------------------------------------------------------
function RunTest(mode)
    data = GenerateSyntheticDataForProject2_A(mode);
    PrintAvailableAttitudes(data);

    ForValidation = data.ForValidation;
    data.ForValidation = [];

    tic0 = tic();
    [ok, B, ~] = EstimateGyrosBias(data, mode);
    dtp = toc(tic0);

    if ok > 0
        fprintf('Processing time = %.1f ms', dtp * 1000);
        if dtp < 0.25
            fprintf('  [OK - under 250ms]\n');
        else
            fprintf('  [WARNING - exceeds 250ms limit!]\n');
        end
        fprintf('\n--- Results ---\n');
        fprintf('Estimated biases : [%.4f]  [%.4f]  [%.4f]  deg/s\n', B(1), B(2), B(3));
        fprintf('Actual biases    : [%.4f]  [%.4f]  [%.4f]  deg/s\n', ...
            ForValidation.UnknownBias(1), ForValidation.UnknownBias(2), ForValidation.UnknownBias(3));
        err = abs(B(:) - ForValidation.UnknownBias(:));
        fprintf('Errors           : [%.4f]  [%.4f]  [%.4f]  deg/s\n', err(1), err(2), err(3));
        if all(err < 0.15)
            fprintf('[PASS] All components within 0.15 deg/s tolerance.\n');
        else
            fprintf('[FAIL] One or more components exceed tolerance.\n');
        end
    else
        fprintf('[ERROR] Estimator returned ok <= 0.\n');
    end
end

% ----------------------------------------------------------
function PrintAvailableAttitudes(r)
    fprintf('Available attitude samples (%d total):\n', r.numOfAttitudeSamples);
    for u = 1:r.numOfAttitudeSamples
        fprintf('  [%02d] t=%.3fs  RPY=[%+.2f  %+.2f  %+.2f] deg\n', ...
            u, r.TimesMeasuredAttitudes(u), r.MeasuredAttitudes(:,u));
    end
    fprintf('\n');
end