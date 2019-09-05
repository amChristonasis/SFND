% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;

% Generate random noise
s=randn(Ns,1);

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
plot(s);

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
T = 4;
% 1b. Guard Cells 
G = 1;

% Offset : Adding room above noise threshold for desired SNR 
offset=3;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T))     
    
    % 2. - 5. Determine the noise threshold by measuring it within the training cells
    sum = 0;
    for j = 1:T
        sum = sum + s(i+T-j); % + s(i+T+j+2*G);
    end
    for k = 1:G
        sum = sum + s(i+T+G-k); % + s(i+T+G+k);
    end
    threshold = sum/((G+T));
    offset = 1.0;
    append(threshold_cfar, offset + threshold);
    % 6. Measuring the signal within the CUT
    append(signal_cfar, s(i+T+G+1));
    
    % 8. Filter the signal above the threshold
    if signal_cfar[i] < threshold_cfar[i] ;
        signal_cfar[i] = 0;
    end
    signal_cfar = [signal_cfar, {signal}];
end




% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')