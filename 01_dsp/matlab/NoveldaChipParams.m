function [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(chipSet, PGen, Sampler)
% Look up table to extract all Novelda pulse parameters depending on the
% function [fc, bw, bwr, vp, n, bw_hz, pwr_dBm, fs_hz] = NoveldaChipParams(chipSet, PGen, Sampler)
%
%
% Inputs:
%   chipSet:    'X1-IPG0' = NVA6100, medium-band pgen
%               'X1-IPG1' = NVA6100, low-band pgen
%               'X2' = NVA6201/2
%               'X4' = X4 SoC
%
%   PGen:       0 = slow, 1 = nominal, 3 = fast, for 'X1'
%               0-11 (CF = 5.3 GHz to 9.1 GHz), for 'X2'
%               0 or 3: CF = 7.29 GHz (ETSI/FCC Compliant), 1 or 4: CF = 8.748 GHz (KCC/FCC Compliant) for 'X4'
%
%   Sampler:    '4mm' = 39 GS/s, '8mm' = 20 GS/s, or '4cm' = 3.8 GS/s, for 'X1'
%               '4mm' only for 'X2' or just omit value
%               '6mm' only for 'X4' or just omit value
%
% Outputs:  
%   fc      = center frequency
%   bw      = fractional bandwidth
%   bwr     = dB down for bandwidth extents
%   vp      = instantaneous output voltage peak amplitude
%   n       = frameSize, i.e. number of samplers
%   bw_hz   = bandwidth in hertz
%   pwr_dBm = mean/average output power in dBm (i.e. true RMS)
%   fs_hz   = sampling rate in Hz
%
%
% Update 9/26/2017, Added X4 support
%
switch lower(chipSet)
    
    case 'x1-ipg0'
        %-------------------NVA6100, Medium-Band PGen--------------------%%
        bwr = 12;                   % bwr dB down from normalized peak setting for fractional bandwidth (changed to 6 dB power spectra bw, looks better)
        n = 512;                    % # of samplers, i.e. length of frame
        
        % Pulse Generator
        switch PGen
            case 0
                fL = 660e6;             % -10 dB low cutoff
                fH = 7145e6;            % -10 dB high cutoff
                vp = 0.47;              % 470 mV instantaneous output amplitude (peak voltage)    
            case 1
                fL = 845e6;             % -10 dB low cutoff
                fH = 9550e6;            % -10 dB high cutoff
                vp = 0.45;              % 450 mV instantaneous output amplitude (peak voltage)  
            case 2
                fL = 1060e6;            % -10 dB low cutoff
                fH = 10410e6;           % -10 dB high cutoff
                vp = 0.37;              % 370 mV instantaneous output amplitude (peak voltage)  
            otherwise
                error('For X1 platform, PGen is an integer input, 0 = slow, 1 = nominal, or 2 = fast...');
        end
        
        % Sampling Rate
        switch lower(Sampler)
            case '4mm'
                fs_hz = 39e9;       % mean system sampling rate for 4mm
            case '8mm'
                fs_hz = 20e9;       % mean system sampling rate for 8mm
            case '4cm'
                fs_hz = 3.8e9;      % mean system sampling rate for 4 cm
            otherwise
                % Default, mean system sampling rate for 4mm
                fs_hz = 39e9;       
                fprintf('For X1 platform, Sampler as string input must be, 4mm, 8mm, or 4cm...\n');
        end
        
        % Center frequency, bandwidth in hertz, and fractional bandwidth
        fc = (fH + fL) / 2;
        bw_hz = fH - fL;
        bw = bw_hz / fc;
        
        % Mean/Average output power in dBm (i.e. true RMS), Nominal,
        % function of PRF @ 50 MHz
        pwr_dBm = -19;      
        
    case 'x1-ipg1'
        %---------------------NVA6100, Low-Band PGen---------------------%%
        bwr = 12;                   % bwr dB down from normalized peak setting for fractional bandwidth (change to 6 dB power spectra bw, looks better)
        n = 512;                    % # of samplers, i.e. length of frame
        vp = 0.5;                   % 500 mV instantaneous output amplitude (peak voltage)  
       
        % Pulse Generator
        switch PGen
            case 0
                fL = 435e6;         % -10 dB low cutoff
                fH = 3165e6;        % -10 dB high cutoff
            case 1
                fL = 450e6;         % -10 dB low cutoff
                fH = 3555e6;        % -10 dB high cutoff
            case 2
                fL = 485e6;         % -10 dB low cutoff
                fH = 4065e6;        % -10 dB high cutoff
            otherwise
                error('For X1 platform, PGen is an integer input, 0 = slow, 1 = nominal, or 2 = fast...');
        end
        
        % Sampling Rate
        switch lower(Sampler)
            case '4mm'
                fs_hz = 39e9;       % mean system sampling rate for 4mm
            case '8mm'
                fs_hz = 20e9;       % mean system sampling rate for 8mm
            case '4cm'
                fs_hz = 3.8e9;      % mean system sampling rate for 4 cm
            otherwise
                % Default, mean system sampling rate for 4mm
                fs_hz = 39e9;       
                fprintf('For X1 platform, Sampler as string input must be, 4mm, 8mm, or 4cm...\n');
        end
        
        % Center frequency, bandwidth in hertz, and fractional bandwidth
        fc = (fH + fL) / 2;
        bw_hz = fH - fL;
        bw = bw_hz / fc;
        
        % Mean/Average output power in dBm (i.e. true RMS), Nominal,
        % function of PRF @ 50 MHz
        pwr_dBm = -14;
        
    case 'x2'
        %%-----------------------------NVA6201---------------------------%%
        bwr = 10;                   % bwr dB down from normalized peak setting for fractional bandwidth (5 dB power spectra bw)
        n = 256;                    % # of samplers, i.e. length of frame
        fs_hz = 39e9;               % mean system sampling rate, 39 GS/s
        
        % Pulse Generator
        switch PGen
            case 0
                % PGSelect = 0;
                fc = 5.3e9;         % center frequency
                bw_hz = 1.75e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~1.75 GHz @ 5.3 GHz)
                vp = 0.69 / 2;      % 690 mV instanteous output voltage peak-to-peak
                pwr_dBm = -10.7;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 1
                % PGSelect = 1;
                fc = 5.4e9;         % center frequency
                bw_hz = 1.80e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~1.80 GHz @ 5.4 GHz)
                vp = 0.69 / 2;      % 690 mV instanteous voltage peak-to-peak 
                pwr_dBm = -10.8;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 2
                % PGSelect = 2;
                fc = 5.7e9;         % center frequency
                bw_hz = 1.85e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~1.85 GHz @ 5.7 GHz)
                vp = 0.72 / 2;      % 720 mV instanteous voltage peak-to-peak 
                pwr_dBm = -11.2;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 3
                % PGSelect = 3;
                fc = 6.1e9;         % center frequency
                bw_hz = 2.05e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.05 GHz @ 6.1 GHz)
                vp = 0.71 / 2;      % 710 mV instanteous voltage peak-to-peak
                pwr_dBm = -11.8;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 4
                % PGSelect = 4;
                fc = 6.4e9;         % center frequency
                bw_hz = 2.15e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.15 GHz @ 6.4 GHz)
                vp = 0.72 / 2;      % 720 mV instanteous voltage peak-to-peak 
                pwr_dBm = -12.0;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 5
                % PGSelect = 5;
                fc = 6.8e9;         % center frequency
                bw_hz = 2.30e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.30 GHz @ 6.8 GHz)
                vp = 0.69 / 2;      % 690 mV instanteous voltage peak-to-peak
                pwr_dBm = -12.6;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 6
                % PGSelect = 6;
                fc = 7.3e9;         % center frequency
                bw_hz = 2.35e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.35 GHz @ 7.3 GHz)
                vp = 0.65 / 2;      % 650 mV instanteous voltage peak-to-peak 
                pwr_dBm = -13.3;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 7
                % PGSelect = 7;
                fc = 7.7e9;         % center frequency
                bw_hz = 2.50e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.50 GHz @ 7.7 GHz)
                vp = 0.62 / 2;      % 620 mV instanteous output voltage peak-to-peak 
                pwr_dBm = -14.0;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 8
                % PGSelect = 8;
                fc = 7.8e9;         % center frequency
                bw_hz = 2.50e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.50 GHz @ 7.8 GHz)
                vp = 0.62 / 2;      % 620 mV instanteous output voltage peak-to-peak 
                pwr_dBm = -14.0;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 9
                % PGSelect = 9;
                fc = 8.2e9;         % center frequency
                bw_hz = 2.65e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~2.65 GHz @ 8.2 GHz)
                vp = 0.57 / 2;      % 570 mV instanteous output voltage peak-to-peak
                pwr_dBm = -14.8;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 10
                % PGSelect = 10
                fc = 8.8e9;         % center frequency
                bw_hz = 3.10e9;     % bandwidth in hertz
                bw = bw_hz / fc;    % fractional bandwidth (~3.10 GHz @ 8.8 GHz)
                vp = 0.54 / 2;      % 540 mV instanteous output voltage peak-to-peak
                pwr_dBm = -16.4;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            case 11
                % PGSelect = 11
                fc = 9.1e9;         % center frequency
                bw_hz = 3.10e9;     % bandwidth in hertz....not sure about this?
                bw = bw_hz / fc;    % fractional bandwidth (~3.10 GHz @ 8.8 GHz)
                vp = 0.52 / 2;      % 540 mV instanteous output voltage peak-to-peak
                pwr_dBm = -16.7;    % mean/average output power in dBm (i.e. true RMS, function of PRF @ 100 MHz)
            otherwise
                error('For X2 platform, PGen is an integer input, >=0 and <= 11...');
        end
        
    case 'x4'
    %------------------X4, Impulse Radar Transceiver SoC-----------------%%
    bwr = 10;                       % bwr dB down from normalized peak setting for fractional bandwidth (5 dB power spectra bw)
    n = 1536;                       % # of samplers, i.e. length of frame
    fs_hz = 23.328e9;               % mean system sampling rate, 23.328 GS/s

    % Pulse Generator
    switch PGen
        case {0, 3}
            % ETSI/FCC        
            fc = 7.29e9;        % center frequency
            bw_hz = 1.4e9;      % bandwidth in hertz....not sure about this?
            bw = bw_hz / fc;    % fractional bandwidth (~3.10 GHz @ 8.8 GHz)
            vp = 0.31;          % 310 mV instanteous output voltage peak
            pwr_dBm = -14;      % mean/average output power in dBm (i.e. true RMS, function of PRF @ 60.75 MHz)
        case {1, 4}
            % KCC/FCC
            fc = 8.748e9;       % center frequency
            bw_hz = 1.5e9;      % bandwidth in hertz....not sure about this?
            bw = bw_hz / fc;    % fractional bandwidth (~3.10 GHz @ 8.8 GHz)
            vp = 0.31;          % 310 mV instanteous output voltage peak
            pwr_dBm = -14;      % mean/average output power in dBm (i.e. true RMS, function of PRF @ 60.75 MHz)
        otherwise
            error('For X4 platform, PGen is an integer input, 0 = 7.29 GHz CF (ETSI/FCC), 1 = 8.748 CF (KCC/FCC)..');
    end     
        
    otherwise
        error('For chipSet, input is a string, "X1-IPG0", "X1-IPG1", "X2", or "X4"...');
end
