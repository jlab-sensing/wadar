clear;

dataDir = 'D:\\XethruData\\';
startFile = 'ExpData20180913T235019.mat';
% startFile = 'ExpData20180916T141907.mat';
% startFile = 'ExpData20180917T204033';

dopplerIndExpected = 25600;  6401;  10505; 
for expFileOffset = 91:95 %19:26
    filenames = dir( sprintf('%s/Exp*.mat',dataDir));
    ExpStartFileIdx = strmatch(startFile,{filenames(:).name}) -1 + expFileOffset;
    filename = filenames(ExpStartFileIdx).name
    radarData = load([dataDir '/' filename]);

    nRx = length(radarData.frameTotCell);
    radarDataOversampleForMaxIdxAllRx = cell(nRx,1);
    radarDataOversampleForMaxIdx_cplx_1i_allRx = cell(nRx,1);
    radarDataOversampleForMaxIdx_cplx_other1i_allRx = cell(nRx,1);
    radarDataOversampleForMaxIdx_sin_allRx = cell(nRx,1);
    for iRx = 1:nRx
        frameTot = radarData.frameTotCell{iRx};
        
        shiftRadTot = linspace(0*pi, 2*pi, 197);
        radarDataOversampleForMaxIdx = zeros(size(frameTot,1), length(shiftRadTot));
        radarDataOversampleForMaxIdx_cplx_1i = radarDataOversampleForMaxIdx;
        radarDataOversampleForMaxIdx_cplx_other1i = radarDataOversampleForMaxIdx;
        radarDataOversampleForMaxIdx_sin = radarDataOversampleForMaxIdx;
        for iAngShift = 1:length(shiftRadTot)
            cdma_sig = sign(sin(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShift)));
            correlatedFrameTmp = frameTot*cdma_sig.'; %sum(frameTot.*cdma_sig_rep,2);
            radarDataOversampleForMaxIdx(:,iAngShift) = correlatedFrameTmp;
            
            cdma_sig_cplx_1i = (exp(1i*(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShift))));
            cdma_sig_cplx_other1i = (exp(-1i*(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShift))));
            cdma_sig_sin = (sin(1*(2*pi*(dopplerIndExpected-1)*(0:size(frameTot,2)-1)/size(frameTot,2) + shiftRadTot(iAngShift))));
            
            radarDataOversampleForMaxIdx_cplx_1i(:,iAngShift) = frameTot*cdma_sig_cplx_1i.';
            radarDataOversampleForMaxIdx_cplx_other1i(:,iAngShift) = frameTot*cdma_sig_cplx_other1i.';
            radarDataOversampleForMaxIdx_sin(:,iAngShift) = frameTot*cdma_sig_sin.';
        end
        radarDataOversampleForMaxIdxAllRx{iRx} = radarDataOversampleForMaxIdx;
        radarDataOversampleForMaxIdx_cplx_1i_allRx{iRx} = radarDataOversampleForMaxIdx_cplx_1i;
        radarDataOversampleForMaxIdx_cplx_other1i_allRx{iRx} = radarDataOversampleForMaxIdx_cplx_other1i;
        radarDataOversampleForMaxIdx_sin_allRx{iRx} = radarDataOversampleForMaxIdx_sin;
        
    end
    fileExtension = filename;
%     save(sprintf('D:\\PhaseLimitedXethruData%d\\PhaseLimited%s',dopplerIndExpected,fileExtension),'-v7.3','shiftRadTot','radarDataOversampleForMaxIdxAllRx',...
%         'radarDataOversampleForMaxIdx_cplx_1i_allRx', 'radarDataOversampleForMaxIdx_cplx_other1i_allRx', 'radarDataOversampleForMaxIdx_sin_allRx');
    save(sprintf('D:\\PhaseLimitedXethruData\\PhaseLimited%s',fileExtension),'-v7.3','shiftRadTot','radarDataOversampleForMaxIdxAllRx',...
        'radarDataOversampleForMaxIdx_cplx_1i_allRx', 'radarDataOversampleForMaxIdx_cplx_other1i_allRx', 'radarDataOversampleForMaxIdx_sin_allRx');

end




