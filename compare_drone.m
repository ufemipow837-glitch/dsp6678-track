%% ========================================================================
%  DSP track vs measured drone data - comparison and visualization
%  MATLAB R2017b+
%
%  CRITICAL FIX: Use PRECISE feature-based filtering to extract ONLY the
%  real drone from measured data.  Old filter (BW=2|3 & H=60-180 & R>300)
%  was wrong - it included MANY unrelated targets at R=300~12200m.
%
%  BW=2/3 targets confirmed by DSP lock at Frame 38:
%    BW 2 or 3 (cross-beam scan)
%    Az 14~18.5 deg
%    R  1094~4400m  (slowly increasing as drone flies away)
%    Vr ~ -11 m/s  (signed Doppler, negative = approaching radar? or depends on convention)
%    H  76~99m
% ========================================================================
clear; clc; close all;

%% ======================= file paths =======================
base_dir = fileparts(mfilename('fullpath'));
fprintf('Script dir: %s\n', base_dir);

CN_dsp_name  = char([36755 20986 25968 25454 46 116 120 116]);
CN_miss_name = char([26080 20154 26426 23454 27979 25968 25454 46 116 120 116]);

all_txt = dir(fullfile(base_dir, '*.txt'));
nm = {all_txt.name};
idx_dsp  = find(strcmp(nm, CN_dsp_name));
idx_miss = find(strcmp(nm, CN_miss_name));
if isempty(idx_dsp) || isempty(idx_miss)
    error('Cannot find BOTH data files in: %s', base_dir);
end
output_file = fullfile(base_dir, nm{idx_dsp(1)});
meas_file   = fullfile(base_dir, nm{idx_miss(1)});
fprintf('DSP file:  %s\n', output_file);
fprintf('Meas file: %s\n', meas_file);

%% ======================= 1. read DSP tracks =======================
fprintf('--- Reading DSP tracks ---\n');

fid = fopen(output_file, 'r', 'n', 'UTF-8');
if fid == -1, fid = fopen(output_file, 'r', 'n', 'GBK'); end
if fid == -1, error('Cannot open: %s', output_file); end

% ---- Per-track storage (supports MULTIPLE simultaneous reliable tracks) ----
max_tracks = 10;  % upper bound
all_frame  = cell(max_tracks,1);
all_t      = cell(max_tracks,1);
all_az     = cell(max_tracks,1);
all_el     = cell(max_tracks,1);
all_r      = cell(max_tracks,1);
all_vr     = cell(max_tracks,1);

cur_frame = -1;
cur_t     = -1;

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end

    fm = regexp(line, 'Frame\s+(\d+)\s+\|\s+t=(\d+)ms', 'tokens');
    if ~isempty(fm)
        cur_frame = str2double(fm{1}{1});
        cur_t     = str2double(fm{1}{2});
    end

    % Capture Track ID, Az, El, R, V
    tm = regexp(line, ...
        'Track\[(\d+)\]:\s*Az=([-\d.]+)deg\s+El=([-\d.]+)deg\s+R=([\d.]+)m\s+V=([-\d.]+)m/s', ...
        'tokens');
    if ~isempty(tm) && cur_frame >= 0
        tid = str2double(tm{1}{1});
        if tid >= 0 && tid < max_tracks
            all_frame{tid+1}(end+1,1) = cur_frame;
            all_t{tid+1}(end+1,1)     = cur_t;
            all_az{tid+1}(end+1,1)    = str2double(tm{1}{2});
            all_el{tid+1}(end+1,1)    = str2double(tm{1}{3});
            all_r{tid+1}(end+1,1)     = str2double(tm{1}{4});
            all_vr{tid+1}(end+1,1)    = str2double(tm{1}{5});
        end
    end
end

% Flatten old single-track vars for backward compat (take first non-empty)
track_frame = []; track_t = []; track_az = []; track_el = []; track_r = []; track_vr = [];
num_active_tracks = 0;
track_ids = [];
for k = 1:max_tracks
    if ~isempty(all_frame{k})
        num_active_tracks = num_active_tracks + 1;
        track_ids(end+1) = k-1;  % 0-based track ID
        if isempty(track_frame)  % first track = primary (drone usually)
            track_frame = all_frame{k};
            track_t     = all_t{k};
            track_az    = all_az{k};
            track_el    = all_el{k};
            track_r     = all_r{k};
            track_vr    = all_vr{k};
        end
    end
end
fclose(fid);

if isempty(track_frame)
    error('No tracks parsed from DSP output!  Did the DSP actually lock any target?');
end

track_s = (track_t - track_t(1)) / 1000;   % relative seconds from first track
fprintf('  DSP track points: %d (Frame %d~%d, t=%d~%dms, %.1fs)\n', ...
    length(track_frame), track_frame(1), track_frame(end), ...
    track_t(1), track_t(end), (track_t(end)-track_t(1))/1000);
fprintf('  DSP track stats:\n');
fprintf('    R  : mean=%.1f  min=%.1f  max=%.1f  (all should be ~830m clutter)\n', ...
    mean(track_r), min(track_r), max(track_r));
fprintf('    Az : mean=%.2f  min=%.2f  max=%.2f\n', ...
    mean(track_az), min(track_az), max(track_az));
fprintf('    Vr : mean=%.2f  min=%.2f  max=%.2f  (signed Doppler)\n', ...
    mean(track_vr), min(track_vr), max(track_vr));

%% ======================= 2. read measured data =======================
fprintf('\n--- Reading measured data ---\n');

fid = fopen(meas_file, 'r', 'n', 'UTF-8');
if fid == -1, fid = fopen(meas_file, 'r', 'n', 'GBK'); end
if fid == -1, error('Cannot open: %s', meas_file); end

% Chinese field names (Unicode codepoints -> never put raw Chinese in source)
CN_bw = [char(27874), char(20301), char(21495), char(58)];
CN_ts = [char(26102), char(38388), char(25139), char(58)];
CN_az = [char(26041), char(20301), char(58)];
CN_r  = [char(36317), char(31163), char(58)];
CN_el = [char(20463), char(20208), char(58)];
CN_v  = [char(36895), char(24230), char(58)];
CN_h  = [char(39640), char(24230), char(58)];

meas_bw = [];
meas_t  = [];
meas_az = [];
meas_el = [];
meas_r  = [];
meas_vr = [];     % Meas V = signed Doppler Vr
meas_h  = [];

while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end

    bw_m = regexp(line, [CN_bw  '(\d+)'], 'tokens');
    ts_m = regexp(line, [CN_ts  '(\d+)'], 'tokens');
    az_m = regexp(line, [CN_az  '([-\d.eE+]+)'], 'tokens');
    r_m  = regexp(line, [CN_r   '([-\d.eE+]+)'], 'tokens');
    el_m = regexp(line, [CN_el  '([-\d.eE+]+)'], 'tokens');
    v_m  = regexp(line, [CN_v   '([-\d.eE+]+)'], 'tokens');
    h_m  = regexp(line, [CN_h   '([-\d.eE+]+)'], 'tokens');

    if ~isempty(ts_m) && ~isempty(az_m) && ~isempty(r_m)
        ts = str2double(ts_m{1}{1});
        if isempty(bw_m), bw = -1; else, bw = str2double(bw_m{1}{1}); end
        if isempty(el_m), el = 0; else, el = str2double(el_m{1}{1}); end
        if isempty(v_m),  vr = 0; else, vr  = str2double(v_m{1}{1}); end
        if isempty(h_m),  h  = 0; else, h   = str2double(h_m{1}{1}); end
        az  = str2double(az_m{1}{1});
        r   = str2double(r_m{1}{1});
        if r > 50 && r < 30000 && ts > 0
            meas_bw(end+1,1) = bw;  %#ok<AGROW>
            meas_t(end+1,1)  = ts;  %#ok<AGROW>
            meas_az(end+1,1) = az;  %#ok<AGROW>
            meas_el(end+1,1) = el;  %#ok<AGROW>
            meas_r(end+1,1)  = r;   %#ok<AGROW>
            meas_vr(end+1,1) = vr;  %#ok<AGROW>
            meas_h(end+1,1)  = h;   %#ok<AGROW>
        end
    end
end
fclose(fid);

fprintf('  All valid points: %d\n', length(meas_t));

%% ======================= 3. extract drone-related points from measured data =======================
% WIDE filter: ALL targets in BW=2/3 beam zone (drone + possible clutter/targets)
% This matches the original blue dots showing everything the radar sees in the drone's beam:
%   Beam   : BW=2 or BW=3 (primary drone beam + adjacent beam leakage)
%   Height : 60-180m (typical drone flight altitude)
%   Range  : >300m (excludes only the nearest ground clutter)
drone_mask = (meas_bw == 2 | meas_bw == 3) & ...
             meas_h > 60 & meas_h < 180 & ...
             meas_r > 300;

drone_t  = meas_t(drone_mask);
drone_bw = meas_bw(drone_mask);
drone_az = meas_az(drone_mask);
drone_el = meas_el(drone_mask);
drone_r  = meas_r(drone_mask);
drone_vr = meas_vr(drone_mask);
drone_h  = meas_h(drone_mask);

[drone_t, si] = sort(drone_t);
drone_bw = drone_bw(si);
drone_az = drone_az(si);
drone_el = drone_el(si);
drone_r  = drone_r(si);
drone_vr = drone_vr(si);
drone_h  = drone_h(si);

t0_all = min([track_t; drone_t; meas_t]);
drone_s = (drone_t - t0_all) / 1000;
track_s_abs = (track_t - t0_all) / 1000;
meas_s = (meas_t - t0_all) / 1000;   % ALL measured points (for 700s background)

fprintf('\n--- BW=2/3 targets (beam 2/3, H 60-180m) ---\n');
fprintf('  Points : %d (from %.1fs to %.1fs)\n', ...
    length(drone_t), drone_s(1), drone_s(end));
fprintf('  R      : mean=%.1f  min=%.1f  max=%.1f  (REAL drone, ~1100-1700m)\n', ...
    mean(drone_r), min(drone_r), max(drone_r));
fprintf('  Az     : mean=%.2f  min=%.2f  max=%.2f\n', ...
    mean(drone_az), min(drone_az), max(drone_az));
fprintf('  Vr     : mean=%.2f  min=%.2f  max=%.2f  (should be ~-11.7 m/s, MOVING AWAY)\n', ...
    mean(drone_vr), min(drone_vr), max(drone_vr));
fprintf('  Height : mean=%.1f  min=%.1f  max=%.1f m\n', ...
    mean(drone_h), min(drone_h), max(drone_h));

%% ======================= 4. diagnosis: DSP vs real drone =======================
fprintf('\n================== DIAGNOSIS ==================\n');
fprintf('DSP   : R=%.0fm, Az=%.1fdeg, Vr=%.1fm/s  (track result)\n', ...
    mean(track_r), mean(track_az), mean(track_vr));
fprintf('Drone : R=%.0fm, Az=%.1fdeg, Vr=%.1fm/s  (precise filtered meas)\n', ...
    mean(drone_r), mean(drone_az), mean(drone_vr));
fprintf('R difference     : %.0f m\n', mean(drone_r) - mean(track_r));
fprintf('Az difference    : %.1f deg\n', mean(drone_az) - mean(track_az));
fprintf('Vr difference    : %.1f m/s\n', mean(drone_vr) - mean(track_vr));
if length(drone_t) < 3
    fprintf('WARNING: Very few drone points after precise filtering!\n');
    fprintf('  Meas data may have gaps in BW=2|3, Az=12-22, Vr=-15~-7 range.\n');
elseif abs(mean(drone_r) - mean(track_r)) > 500
    fprintf('CONCLUSION: DSP is NOT tracking the drone - distance mismatch.\n');
elseif abs(mean(drone_az) - mean(track_az)) > 5
    fprintf('CONCLUSION: DSP azimuth does not match real drone.\n');
else
    fprintf('CONCLUSION: DSP matches real drone well - same target!\n');
end
fprintf('===============================================\n');

%% ======================= 5. cartesian coords =======================
% Multi-track cartesian
all_x = cell(num_active_tracks,1);
all_y = cell(num_active_tracks,1);
all_z = cell(num_active_tracks,1);
for k = 1:num_active_tracks
    idx = track_ids(k) + 1;
    all_x{k} = all_r{idx} .* cosd(all_el{idx}) .* cosd(all_az{idx});
    all_y{k} = all_r{idx} .* cosd(all_el{idx}) .* sind(all_az{idx});
    all_z{k} = all_r{idx} .* sind(all_el{idx});
end

% Legacy single-track vars (first track)
if num_active_tracks > 0
    t_x = all_x{1}; t_y = all_y{1}; t_z = all_z{1};
end

d_x = drone_r .* cosd(drone_el) .* cosd(drone_az);
d_y = drone_r .* cosd(drone_el) .* sind(drone_az);
d_z = drone_r .* sind(drone_el);

% ALL measured points cartesian (for 700s background in XY/3D views)
m_x = meas_r .* cosd(meas_el) .* cosd(meas_az);
m_y = meas_r .* cosd(meas_el) .* sind(meas_az);
m_z = meas_r .* sind(meas_el);

%% ======================= 6. time-aligned Vr comparison (overlap only) =======================
% For the small overlap period (DSP only has ~26s of tracks vs drone's ~719s),
% do a nearest-point Vr comparison where times are within 200ms.
overlap_vr_t = [];
overlap_vr_m = [];
overlap_vr_d = [];
for i = 1:length(track_t)
    dt_all = abs(drone_t - track_t(i));
    [dt_min, idx_best] = min(dt_all);
    if dt_min < 200
        overlap_vr_t(end+1,1) = (track_t(i) - t0_all)/1000;  %#ok<AGROW>
        overlap_vr_m(end+1,1) = track_vr(i);                 %#ok<AGROW>
        overlap_vr_d(end+1,1) = drone_vr(idx_best);          %#ok<AGROW>
    end
end
fprintf('\n--- Time-aligned Vr overlap (|dt|<200ms) ---\n');
fprintf('  Overlap points: %d\n', length(overlap_vr_t));
if length(overlap_vr_t) > 0
    vr_diff = overlap_vr_m - overlap_vr_d;
    fprintf('  DSP   Vr : mean=%.2f  std=%.2f\n', mean(overlap_vr_m), std(overlap_vr_m));
    fprintf('  Drone Vr : mean=%.2f  std=%.2f\n', mean(overlap_vr_d), std(overlap_vr_d));
    fprintf('  Vr diff  : mean=%.2f  std=%.2f  RMS=%.2f m/s\n', ...
        mean(vr_diff), std(vr_diff), sqrt(mean(vr_diff.^2)));
end

%% ======================= 7. visualization =======================

% Color palette for MULTI-TRACK (distinct, bright colors)
track_colors = [
    0.90 0.10 0.10;   % Track 0: red (primary, expected drone)
    0.10 0.40 0.90;   % Track 1: blue
    0.95 0.60 0.00;   % Track 2: orange
    0.50 0.10 0.80;   % Track 3: purple
    0.10 0.70 0.30;   % Track 4: green
];
track_line_styles = {'-', '--', '-.', ':', '-'};
track_labels = cell(1, num_active_tracks);
for k = 1:num_active_tracks
    idx = track_ids(k) + 1;  % 1-based into all_* cells
    track_labels{k} = sprintf('DSP Track[%d] (n=%d, R~%.0fm)', ...
        track_ids(k), length(all_frame{idx}), mean(all_r{idx}));
end

% Figure 1: Range vs Time
figure('Name','Range vs Time','Position',[50 50 1200 500],'Color','w');
% Layer 1: ALL measured points (700s background) - light gray
scatter(meas_s, meas_r, 6, [0.65 0.65 0.65], 'filled', 'MarkerFaceAlpha', 0.35);
hold on;
% Layer 2: FILTERED real drone (62 precise points) - blue prominent
scatter(drone_s, drone_r, 35, 'b', 'filled', 'MarkerFaceAlpha', 0.8);
% Layer 3: ALL DSP tracks (colored)
for k = 1:num_active_tracks
    idx = track_ids(k) + 1;
    t_s_k = (all_t{idx} - t0_all) / 1000;
    col = track_colors(mod(k-1, size(track_colors,1))+1, :);
    ls  = track_line_styles{mod(k-1, length(track_line_styles))+1};
    plot(t_s_k, all_r{idx}, 'Color', col, 'LineStyle', ls, 'LineWidth', 2.5);
end
grid on;
xlabel('Time (s)', 'FontSize',12); ylabel('Range R (m)', 'FontSize',12);
title({'Range vs Time', ...
    sprintf('ALL meas (gray) + BW=2/3 beam targets (blue) + %d DSP track(s)', num_active_tracks)}, ...
    'FontSize',13, 'FontWeight','bold');
legend([{'All measured points','BW=2/3 targets (blue)'}, track_labels], 'Location','best','FontSize',10);

% Figure 2: Vr (signed Doppler) vs Time - THE SPEED COMPARISON
figure('Name','Radial Velocity Vr vs Time','Position',[70 70 1200 500],'Color','w');
scatter(meas_s, meas_vr, 6, [0.65 0.65 0.65], 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
scatter(drone_s, drone_vr, 35, 'b', 'filled', 'MarkerFaceAlpha', 0.8);
for k = 1:num_active_tracks
    idx = track_ids(k) + 1;
    t_s_k = (all_t{idx} - t0_all) / 1000;
    col = track_colors(mod(k-1, size(track_colors,1))+1, :);
    ls  = track_line_styles{mod(k-1, length(track_line_styles))+1};
    plot(t_s_k, all_vr{idx}, 'Color', col, 'LineStyle', ls, 'LineWidth', 2.5);
end
plot(xlim, [0 0], 'k--', 'LineWidth', 0.8);
grid on;
xlabel('Time (s)', 'FontSize',12); ylabel('Radial Velocity Vr (m/s)', 'FontSize',12);
title({'Radial Velocity Vr (signed Doppler)', ...
    'ALL meas (gray) + BW=2/3 beam targets (blue) + DSP tracks'}, ...
    'FontSize',13, 'FontWeight','bold');
legend([{'All measured','BW=2/3 targets'}, strrep(track_labels,'DSP ','')], 'Location','best','FontSize',10);

% Figure 3: Azimuth vs Time
figure('Name','Azimuth vs Time','Position',[90 90 1200 500],'Color','w');
scatter(meas_s, meas_az, 6, [0.65 0.65 0.65], 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
scatter(drone_s, drone_az, 35, 'b', 'filled', 'MarkerFaceAlpha', 0.8);
for k = 1:num_active_tracks
    idx = track_ids(k) + 1;
    t_s_k = (all_t{idx} - t0_all) / 1000;
    col = track_colors(mod(k-1, size(track_colors,1))+1, :);
    ls  = track_line_styles{mod(k-1, length(track_line_styles))+1};
    plot(t_s_k, all_az{idx}, 'Color', col, 'LineStyle', ls, 'LineWidth', 2.5);
end
grid on;
xlabel('Time (s)', 'FontSize',12); ylabel('Azimuth (deg)', 'FontSize',12);
title({'Azimuth vs Time', ...
    'ALL meas (gray) + BW=2/3 beam targets (blue) + DSP tracks'}, ...
    'FontSize',14, 'FontWeight','bold');
legend([{'All measured','BW=2/3 targets'}, strrep(track_labels,'DSP ','')], 'Location','best','FontSize',10);

% Figure 4: XY Top View
figure('Name','XY Top View','Position',[110 110 900 900],'Color','w');
scatter(m_x, m_y, 8, [0.65 0.65 0.65], 'filled', 'MarkerFaceAlpha', 0.3);
hold on;
plot(d_x, d_y, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 6);
for k = 1:num_active_tracks
    col = track_colors(mod(k-1, size(track_colors,1))+1, :);
    ls  = track_line_styles{mod(k-1, length(track_line_styles))+1};
    plot(all_x{k}, all_y{k}, 'Color', col, 'LineStyle', ls, 'LineWidth', 2.6);
end
scatter(0, 0, 200, 'k', '^', 'filled');
axis equal; grid on;
xlabel('X (m)', 'FontSize',12);
ylabel('Y (m)', 'FontSize',12);
title({'XY Top View', ...
    'ALL meas (gray) + BW=2/3 beam targets (blue) + DSP tracks + Radar'}, ...
    'FontSize',14, 'FontWeight','bold');
legend([{'All measured','BW=2/3 targets'}, strrep(track_labels,'DSP ',''), {'Radar'}], ...
    'Location','best','FontSize',10);

% Figure 5: 3D Trajectory
figure('Name','3D Trajectory','Position',[130 130 1200 850],'Color','w');
scatter3(m_x, m_y, m_z, 8, [0.65 0.65 0.65], 'filled', 'MarkerFaceAlpha', 0.25);
hold on;
scatter3(d_x, d_y, d_z, 45, 'b', 'filled', 'MarkerFaceAlpha', 0.7);
for k = 1:num_active_tracks
    col = track_colors(mod(k-1, size(track_colors,1))+1, :);
    ls  = track_line_styles{mod(k-1, length(track_line_styles))+1};
    plot3(all_x{k}, all_y{k}, all_z{k}, 'Color', col, 'LineStyle', ls, 'LineWidth', 2.5);
end
scatter3(0, 0, 0, 300, 'k', '^', 'filled');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title({'3D Trajectory', ...
    'ALL meas (gray) + BW=2/3 beam targets (blue) + DSP tracks + Radar'}, ...
    'FontSize',14, 'FontWeight','bold');
legend([{'All measured','BW=2/3 targets'}, strrep(track_labels,'DSP ',''), {'Radar'}], ...
    'Location','best','FontSize',10);
view(3); rotate3d on;

% Figure 6: Vr time-aligned overlap (APPLES-TO-APPLES direct comp)
if length(overlap_vr_t) > 2
    figure('Name','Vr Overlap Comparison','Position',[150 150 1100 500],'Color','w');
    plot(overlap_vr_t, overlap_vr_m, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 6);
    hold on;
    plot(overlap_vr_t, overlap_vr_d, 'b-s', 'LineWidth', 1.5, 'MarkerSize', 6);
    plot(xlim, [0 0], 'k--', 'LineWidth', 0.8);
    grid on;
    xlabel('Time (s)', 'FontSize',12); ylabel('Radial Velocity Vr (m/s)', 'FontSize',12);
    title({'Vr Overlap (nearest-point within 200ms)', ...
        sprintf('N=%d points, Vr diff RMS=%.2f m/s', ...
        length(overlap_vr_t), sqrt(mean(vr_diff.^2)))}, ...
        'FontSize',13, 'FontWeight','bold');
    legend({'DSP track Vr','BW=2/3 targets Vr'}, ...
        'Location','best','FontSize',11);
else
    fprintf('Note: overlap figure skipped (<3 points aligned)\n');
end

% Figure 7: distribution of ALL measured points by range
figure('Name','All Measured Points R-distribution','Position',[170 170 1000 500],'Color','w');
subplot(1,2,1);
hist(meas_r(meas_r < 3000), 100);
xlabel('Range R (m)'); ylabel('Count');
title('All measured points R<3000m');
grid on;
subplot(1,2,2);
hist(meas_vr(meas_r < 3000), 100);
xlabel('Vr (m/s)'); ylabel('Count');
title('All measured points R<3000m');
grid on;
% Mark DSP lock point (R2017b compatible: xline -> plot)
hold on;
xl = xlim;
plot([mean(track_vr) mean(track_vr)], ylim, 'r--', 'LineWidth', 2);
xlim(xl);
legend({'All meas Vr','DSP track Vr'});

fprintf('\nDone! %d figures plotted.\n', 7);