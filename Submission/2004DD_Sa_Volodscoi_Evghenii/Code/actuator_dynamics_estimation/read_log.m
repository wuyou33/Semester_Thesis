function B = read_log(filename)

%% Experiment comments:
% Experiment 1: thrust: 36000, 42000, 30000, 48000, 0 
% Experiment 2: thrust: 36000 roll: 5000, -5000, 10000, -10000, 0 (byte
% misalignment, throw first byte away!)


%%

fileID = fopen(filename);
A = fread(fileID);

if strcmp(filename,'experiment_roll_changes.log')
    A = A(2:end);
end

fclose(fileID);

%%

B = zeros(floor(length(A)/2),1);

for n=1:2:length(A)
    if (n==length(A)) 
        break; 
    else
        B((n+1)/2) = swapbytes(typecast(uint8(A(n:(n+1))), 'uint16'));
    end
end

% unit of B: Hz (two blades!)
