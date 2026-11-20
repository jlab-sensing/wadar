function T = AnalyzeTripleTagScans(localDataPath, patterns, tagHz, outDir, emptyPrefix)
% T = AnalyzeTripleTagScans(localDataPath, patterns, tagHz, outDir, emptyPrefix)
%
% Locates each backscatter tag in a set of radar captures using the
% standard pipeline (ProcessFrames -> slow-time FFT -> TagIndex ->
% TagLocateCWT / max -> TagSNR). Tag scans and empty (no-tag) scans can be
% processed together, so the tag peaks can be compared against the
% empty-scan baseline. Cross-check the results with validate_triple_tag.py,
% which is an independent implementation.
%
% Inputs:
%   localDataPath: capture folder relative to pwd, starting with '/',
%                  e.g. '/data/calibrate' (same convention as ProcessFrames)
%   patterns:      cell array of dir() patterns, e.g.
%                  {'tag_pos*.frames', 'empty_pos*.frames'} (default {'*.frames'})
%   tagHz:         tag frequencies in Hz, top to bottom (default [105 80 64])
%   outDir:        folder for the CSV, plots and .mat file ('' = no output files)
%   emptyPrefix:   file-name prefix of the no-tag scans (default 'empty')
%
% Outputs:
%   T: table with, per capture and tag, the peak bin (max and CWT), the SNR
%      from TagSNR, the peak-vs-median prominence (dB) and the peak value.
%
% Notes:
%   - Range bin size in air is c / (2 * fs), about 0.42 cm for Chipotle.
%   - TagIndex needs tagHz < frameRate. Captures where that does not hold
%     (e.g. a 100 fps capture with a 105 Hz tag) are skipped with a warning;
%     validate_triple_tag.py handles aliasing and can process them.
%   - TagSNR is 10*log10 of a magnitude ratio (half of a power dB value).
%
% Example (from 01_dsp/matlab):
%   T = AnalyzeTripleTagScans('/data/calibrate', ...
%       {'tag_pos*.frames', 'empty_pos*.frames'}, [105 80 64], 'results/calibrate');

arguments
    localDataPath = '/data'
    patterns = {'*.frames'}
    tagHz = [105 80 64]
    outDir = ''
    emptyPrefix = 'empty'
end

if ischar(patterns) || isstring(patterns), patterns = cellstr(patterns); end
nTags = numel(tagHz);
c = 299792458;

names = {};
for k = 1:numel(patterns)
    files = dir(fullfile(pwd, localDataPath, patterns{k}));
    names = [names, {files.name}]; %#ok<AGROW>
end
names = unique(names, 'stable');
if isempty(names)
    error('No captures match the patterns in %s', fullfile(pwd, localDataPath));
end

n = numel(names);
keep = true(n, 1);
prof = cell(n, nTags);
pkMax = nan(n, nTags); pkCWT = nan(n, nTags); snr = nan(n, nTags);
prom = nan(n, nTags); pkVal = nan(n, nTags);
fs = nan(n, 1); fps = nan(n, 1);

for i = 1:n
    fs(i) = readSampleRate(fullfile(pwd, localDataPath, names{i}));
    [~, bb, fr] = ProcessFrames(localDataPath, names{i});
    fps(i) = fr;
    N = width(bb);
    if any(tagHz / fr * N + 2 > N)
        warning('Skipping %s: frame rate %d fps is too low for TagIndex with tags at %s Hz.', ...
            names{i}, fr, mat2str(tagHz));
        keep(i) = false;
        continue
    end
    FT = fft(bb, N, 2);
    for t = 1:nTags
        [p, fidx] = TagIndex(FT, fr, tagHz(t));
        prof{i, t} = p;
        try
            pkCWT(i, t) = TagLocateCWT(p, false);
        catch
            pkCWT(i, t) = NaN;
        end
        [pkVal(i, t), pkMax(i, t)] = max(p);
        snr(i, t) = TagSNR(FT, fidx, pkMax(i, t));
        prom(i, t) = 20 * log10(pkVal(i, t) / median(p));
    end
    fprintf('%s done\n', names{i});
end

names = names(keep); prof = prof(keep, :); fs = fs(keep); fps = fps(keep);
pkMax = pkMax(keep, :); pkCWT = pkCWT(keep, :); snr = snr(keep, :);
prom = prom(keep, :); pkVal = pkVal(keep, :);
binCm = c ./ (2 * fs) * 100;

T = table(string(names(:)), fps, binCm, 'VariableNames', {'capture', 'fps', 'binCm'});
for t = 1:nTags
    tag = sprintf('%g', tagHz(t));
    T.(['max' tag]) = pkMax(:, t);
    T.(['cwt' tag]) = pkCWT(:, t);
    T.(['snr' tag]) = snr(:, t);
    T.(['prom' tag]) = prom(:, t);
    T.(['pk' tag]) = pkVal(:, t);
end
for t = 1:nTags-1
    T.(sprintf('diff%g_%g', tagHz(t), tagHz(t+1))) = pkMax(:, t+1) - pkMax(:, t);
end
disp(T)

if isempty(outDir), return; end
if ~exist(outDir, 'dir'), mkdir(outDir); end
writetable(T, fullfile(outDir, 'triple_tag_results.csv'));

isEmpty = startsWith(string(names), emptyPrefix);
cols = lines(nTags);

fig = figure('Position', [0 0 1400 300 * nTags], 'Visible', 'off');
for t = 1:nTags
    subplot(nTags, 1, t); hold on;
    for i = find(isEmpty(:))', plot(prof{i, t}, 'Color', [0.6 0.6 0.6]); end
    for i = find(~isEmpty(:))', plot(prof{i, t}, 'Color', cols(t, :)); end
    title(sprintf('%g Hz tag: tag scans (color) vs empty scans (gray)', tagHz(t)));
    xlabel('Range bin'); ylabel('|FFT| at tag freq'); grid on; xlim([1 512]);
end
exportgraphics(fig, fullfile(outDir, 'profiles_tag_vs_empty.png'));
close(fig);

ti = find(~isEmpty(:))';
if ~isempty(ti)
    fig = figure('Position', [0 0 1400 200 * numel(ti)], 'Visible', 'off');
    for k = 1:numel(ti)
        subplot(numel(ti), 1, k); hold on;
        for t = 1:nTags, plot(prof{ti(k), t} / max(prof{ti(k), t}), 'Color', cols(t, :)); end
        title(names{ti(k)}, 'Interpreter', 'none'); xlim([1 512]); grid on;
        if k == 1, legend(compose('%g Hz', tagHz)); end
    end
    xlabel('Range bin');
    exportgraphics(fig, fullfile(outDir, 'profiles_normalized_per_tag_scan.png'));
    close(fig);
end

save(fullfile(outDir, 'triple_tag_profiles.mat'), 'prof', 'names', 'tagHz', 'fs', 'fps');

end

function fs = readSampleRate(fileName)
% Sampling rate from the frameLogger header (X1/Chipotle: double after 7 int32s)
fid = fopen(fileName, 'r');
fread(fid, 7, 'int32');
fs = fread(fid, 1, 'double');
fclose(fid);
end
