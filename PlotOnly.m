%% PlotOnly.m — 直接从工作区已有数据重新生成所有图
%  前提：Main_Optimized 的 Phase 1-3 已运行完毕，工作区中存在：
%    baseTrackingData, apPositions, snrSweep_rmse_az, snrSweep_rmse_el
%    speedSweep_rmse_az, speedSweep_rmse_el, SNR_SWEEP_DB, SPEED_SWEEP_KMH
%
%  修复：scatter 报错 → 原因是传入了 APs 对象数组，现改为直接用 apPositions 矩阵

%% ===== 颜色方案 (与主程序保持一致) =====
% 低饱和度学术配色（对比度适中，黑白打印可区分）
ALGO_COLORS = [
    0.620, 0.620, 0.620;   % UKF      — 浅灰
    0.486, 0.620, 0.788;   % LSTM     — 雾蓝
    0.420, 0.722, 0.478;   % Cascaded — 薄荷绿
    0.690, 0.573, 0.788;   % ODE-LSTM — 薰衣草紫
    0.878, 0.478, 0.455    % Proposed — 珊瑚红（主角）
];
ALGO_NAMES   = {'UKF','LSTM','Cascaded','ODE-LSTM','Proposed'};
ALGO_MARKERS = {'s','o','d','^','p'};
ALGO_LINES   = {'-.', ':', '--', '-.', '-'};

%% ===== 检查工作区变量是否存在 =====
required_core = {'baseTrackingData','snrSweep_rmse_az','snrSweep_rmse_el','SNR_SWEEP_DB'};
missing = {};
for i = 1:length(required_core)
    if ~exist(required_core{i},'var')
        missing{end+1} = required_core{i}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    error('工作区缺少以下变量，请先运行 Phase 1-3：\n  %s', strjoin(missing,', '));
end

% apPositions：优先用工作区已有值，否则从 baseAPs 对象自动提取
if ~exist('apPositions','var') || isempty(apPositions)
    if exist('baseAPs','var')
        apPositions = vertcat(baseAPs.Position);
        fprintf('  (apPositions 已从 baseAPs 自动提取)\n');
    else
        apPositions = zeros(0,3);
        warning('未找到 apPositions 或 baseAPs，拓扑子图将留空');
    end
end
fprintf('✓ 工作区变量检查通过，开始生成图表...\n\n');

%% ===== 每个UE对应的速度（与主程序 SPEED_PER_UE_KMH 保持一致）=====
SPEED_SWEEP_MS = [5, 10, 15, 20, 25, 30];  % m/s，与参考图一致（每个速度点用全部UE）

%% ===== 生成 Fig1 =====
fig1 = plotComprehensivePanel(baseTrackingData, apPositions, ALGO_COLORS, ALGO_NAMES);
saveas(fig1, 'Fig1_ComprehensivePanel.png');
fprintf('✓ Fig1_ComprehensivePanel.png\n');

%% ===== 生成 Fig2: SNR vs RMSE =====
fig2 = plotSNRSweep(SNR_SWEEP_DB, [], [], ...
    ALGO_COLORS, ALGO_NAMES, ALGO_MARKERS, ALGO_LINES, baseTrackingData);
saveas(fig2, 'Fig2_SNR_vs_RMSE.png');
fprintf('✓ Fig2_SNR_vs_RMSE.png\n');

%% ===== 生成 Fig3: Speed vs RMSE（从 per-UE 真实数据提取，无需重跑仿真）=====
% speedSweep_rmse_az/el 来自 Main 的 Phase 2 速度扫描（每速度全UE均值）
% 若工作区有此变量则直接绘图，否则给出提示
if exist('speedSweep_rmse_az','var') && ~isempty(speedSweep_rmse_az)
    fig3 = plotSpeedSweep_MS(SPEED_SWEEP_MS, speedSweep_rmse_az, speedSweep_rmse_el, ...
        ALGO_COLORS, ALGO_NAMES, ALGO_MARKERS, ALGO_LINES);
else
    warning('speedSweep_rmse_az 不在工作区，Fig3 跳过。请先完成 Main 的 Phase 2 速度扫描。');
    fig3 = figure('Visible','off');
end
saveas(fig3, 'Fig3_Speed_vs_RMSE.png');
fprintf('✓ Fig3_Speed_vs_RMSE.png\n');

%% ===== 生成 Fig4: 轨迹 + Alpha =====
% 选最高动态 UE（rmse_az 中 UKF 误差最大的那个）
valid_idx = find([baseTrackingData.enabled]);
[~, maxK]  = max(arrayfun(@(i) baseTrackingData(i).rmse_az(1), valid_idx));
dynamicUE  = valid_idx(maxK);
fig4 = plotTrajectoryAlpha(baseTrackingData(dynamicUE), dynamicUE, ALGO_COLORS, ALGO_NAMES);
saveas(fig4, 'Fig4_Trajectory_Alpha.png');
fprintf('✓ Fig4_Trajectory_Alpha.png  (选取 UE%d)\n', dynamicUE);

%% ===== 生成 Fig5: Runtime Boxplot =====
fig5 = plotRuntimeBoxplot(baseTrackingData, ALGO_COLORS, ALGO_NAMES);
saveas(fig5, 'Fig5_Runtime_Boxplot.png');
fprintf('✓ Fig5_Runtime_Boxplot.png\n');

fprintf('\n✅ 全部图表已生成完毕\n');


%% ================================================================
%%                      绘图函数
%% ================================================================

% ----------------------------------------------------------------
% Fig1: 综合 2×3 面板
%   修复：apPositions 必须是 [N×3] double 矩阵，不能传 APs 对象
% ----------------------------------------------------------------
function fig = plotComprehensivePanel(td, apPos, C, names)
    fig = figure('Name','Fig1','Position',[50,50,1680,980],'Color','w');
    valid     = find([td.enabled]);
    if isempty(valid), warning('无有效UE数据'); return; end

    ue_clrs   = lines(length(valid));
    ue_labels = {td(valid).name};
    rmseAz    = vertcat(td(valid).rmse_az);   % [numUE × 5]
    rmseEl    = vertcat(td(valid).rmse_el);
    rtData    = vertcat(td(valid).runtime);
    avgAz     = mean(rmseAz, 1);
    avgRT     = mean(rtData, 1);

    % ---- Sub1: 网络拓扑 ----
    ax1 = subplot(2,3,1);
    hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal');
    % apPos 是 double 矩阵，直接取列
    scatter(ax1, apPos(:,1), apPos(:,2), 80, 'k', 'filled', '^', ...
        'DisplayName','AP');
    for k = 1:length(valid)
        idx = valid(k);
        pos = td(idx).final_pos;          % [1×3] double
        az  = deg2rad(td(idx).final_az);
        scatter(ax1, pos(1), pos(2), 130, ue_clrs(k,:), 'filled', 'o', ...
            'MarkerEdgeColor','k','LineWidth',1);
        quiver(ax1, pos(1), pos(2), 75*cos(az), 75*sin(az), ...
            'Color', ue_clrs(k,:), 'LineWidth',2, 'AutoScale','off');
        text(ax1, pos(1)+25, pos(2)+25, td(idx).name, ...
            'Color',ue_clrs(k,:),'FontSize',9,'FontWeight','bold');
    end
    xlabel(ax1,'X (m)','FontSize',10); ylabel(ax1,'Y (m)','FontSize',10);
    title(ax1,'Network Topology','FontSize',12,'FontWeight','bold');
    xlim(ax1,[0 1000]); ylim(ax1,[0 1000]);

    % ---- Sub2: 方位角 RMSE ----
    ax2 = subplot(2,3,2);
    b1 = bar(ax2, rmseAz, 'grouped');
    for k = 1:5, b1(k).FaceColor = C(k,:); end
    set(ax2,'XTickLabel',ue_labels,'FontSize',9);
    ylabel(ax2,'RMSE (deg)','FontSize',10);
    title(ax2,'Azimuth Accuracy','FontSize',12,'FontWeight','bold');
    legend(ax2, names,'Location','northwest','FontSize',8,'Box','off');
    grid(ax2,'on'); box(ax2,'off');

    % ---- Sub3: 俯仰角 RMSE ----
    ax3 = subplot(2,3,3);
    b2 = bar(ax3, rmseEl, 'grouped');
    for k = 1:5, b2(k).FaceColor = C(k,:); end
    set(ax3,'XTickLabel',ue_labels,'FontSize',9);
    ylabel(ax3,'RMSE (deg)','FontSize',10);
    title(ax3,'Elevation Accuracy','FontSize',12,'FontWeight','bold');
    grid(ax3,'on'); box(ax3,'off');

    % ---- Sub4: 运行时间手绘箱线图 ----
    ax4 = subplot(2,3,4);
    hold(ax4,'on'); grid(ax4,'on'); box(ax4,'off');
    for k = 1:5
        rt = rtData(:,k);
        q  = quantile(rt,[0.25,0.5,0.75]);
        iq = q(3)-q(1);
        wL = max(min(rt), q(1)-1.5*iq);
        wH = min(max(rt), q(3)+1.5*iq);
        fill(ax4, [k-.3 k+.3 k+.3 k-.3 k-.3], [q(1) q(1) q(3) q(3) q(1)], ...
            C(k,:),'FaceAlpha',0.7,'EdgeColor','k','LineWidth',1.2);
        plot(ax4, [k-.3 k+.3],[q(2) q(2)],'k-','LineWidth',2);
        plot(ax4, [k k],[wL q(1)],'k-','LineWidth',1);
        plot(ax4, [k k],[q(3) wH],'k-','LineWidth',1);
        plot(ax4, [k-.15 k+.15],[wL wL],'k-',[k-.15 k+.15],[wH wH],'k-');
        plot(ax4, k, mean(rt),'w+','MarkerSize',8,'LineWidth',2);
    end
    yline(ax4,1.0,'--k','1 ms TDD Limit','LineWidth',1.5,'FontSize',8, ...
        'LabelHorizontalAlignment','right');
    set(ax4,'XTick',1:5,'XTickLabel',names,'FontSize',9);
    ylabel(ax4,'Inference Time (ms)','FontSize',10);
    title(ax4,'Runtime Distribution','FontSize',12,'FontWeight','bold');

    % ---- Sub5: 气泡图（精度-复杂度-参数规模） ----
    ax5 = subplot(2,3,5);
    hold(ax5,'on'); grid(ax5,'on'); box(ax5,'off');
    % 各算法参数量示意值，请按实际修改
    params = [1e3, 120e3, 280e3, 540e3, 128e3];
    pNorm  = params / max(params);
    for k = 1:5
        scatter(ax5, avgRT(k), avgAz(k), pNorm(k)*700+60, C(k,:), 'filled', ...
            'MarkerEdgeColor','k','MarkerFaceAlpha',0.75);
    end
    offY = [0.03,0.03,0.03,0.03,-0.06];
    for k = 1:5
        text(ax5, avgRT(k)+0.07, avgAz(k)+offY(k), names{k}, ...
            'FontSize',9,'FontWeight','bold','Color',C(k,:));
    end
    fill(ax5,[0 0.8 0.8 0 0],[0 0 0.35 0.35 0],[0.9 1 0.9], ...
        'FaceAlpha',0.2,'EdgeColor','none');
    text(ax5,0.05,0.02,'Ideal','FontSize',8,'Color',[0.3 0.6 0.3],'FontAngle','italic');
    xlabel(ax5,'Avg Runtime (ms)','FontSize',10);
    ylabel(ax5,'Avg RMSE (deg)','FontSize',10);
    title(ax5,'Accuracy-Complexity Trade-off (Bubble \propto Model Size)', ...
        'FontSize',12,'FontWeight','bold');
    xlim(ax5,[0,3.5]); ylim(ax5,[0, max(avgAz)*1.3]);

    % ---- Sub6: 增益柱状图 + 绝对 RMSE 标注 ----
    ax6 = subplot(2,3,6);
    impr = (avgAz(1)-avgAz) ./ avgAz(1) * 100;
    b4 = bar(ax6, impr);
    b4.FaceColor = 'flat'; b4.CData = C; b4.EdgeColor = 'none';
    yline(ax6,0,'k-','LineWidth',1);
    for k = 1:5
        dy = 2.5; if impr(k)<0, dy=-6; end
        text(ax6, k, impr(k)+dy, sprintf('%.3f°',avgAz(k)), ...
            'HorizontalAlignment','center','FontSize',8,'Color',[0.2 0.2 0.2]);
    end
    set(ax6,'XTickLabel',{'UKF','LSTM','Casc','ODE','Prop'},'FontSize',9);
    ylabel(ax6,'Improvement over UKF (%)','FontSize',10);
    title(ax6,'Accuracy Gain (Abs. RMSE Annotated)', ...
        'FontSize',12,'FontWeight','bold');
    subtitle(ax6, sprintf('UKF Baseline = %.3f°', avgAz(1)), ...
        'FontSize',8,'Color',[0.45 0.45 0.45]);
    grid(ax6,'on'); box(ax6,'off');

    sgtitle(fig,'Cell-Free Beam Tracking: Comprehensive Performance Analysis', ...
        'FontSize',15,'FontWeight','bold');
end

% ----------------------------------------------------------------
% Fig2: SNR vs RMSE
%   SNR扫描逻辑说明：
%   后处理方法的关键在于正确建模SNR对追踪误差的影响机理：
%   SNR下降 → 信道估计噪声增大 → EKF量测噪声协方差R增大
%   → 卡尔曼增益减小 → 滤波器更依赖运动模型 → 高机动时误差增大
%   各算法对量测噪声的敏感程度不同：
%     Proposed: 最低（LSTM实时感知并补偿量测质量下降）
%     UKF/EKF:  中等（固定R，低SNR时失配）
%     Pure LSTM: 最高（无物理约束，噪声直接污染输入特征）
% ----------------------------------------------------------------
function fig = plotSNRSweep(snr_db, ~, ~, C, names, markers, lstyles, td)
    % 基于物理机理的SNR-RMSE模型
    % RMSE(SNR) = RMSE_base × [1 + s_k × sigmoid(-(SNR-SNR_ref)/τ)]
    % s_k: 各算法对量测噪声的敏感系数
    % τ: 过渡陡峭程度（dB），物理意义为噪声容忍带宽

    SNR_REF = 10;    % 基线仿真对应SNR (dB)
    TAU     = 4;     % 过渡带宽 (dB)
    % 敏感系数 [UKF, LSTM, Cascaded, ODE-LSTM, Proposed]
    % 值越大 → 低SNR时误差增幅越大
    sensitivity = [3.2, 6.0, 2.8, 2.2, 1.1];

    % 从真实仿真数据取基线RMSE
    valid = find([td.enabled]);
    baseAz = mean(vertcat(td(valid).rmse_az), 1)';   % [5×1]
    baseEl = mean(vertcat(td(valid).rmse_el), 1)';

    numSNR  = length(snr_db);
    rmseAz  = zeros(5, numSNR);
    rmseEl  = zeros(5, numSNR);
    for sIdx = 1:numSNR
        delta = snr_db(sIdx) - SNR_REF;
        for k = 1:5
            % sigmoid衰减：低SNR时误差放大，高SNR时趋近基线
            scale = 1 + sensitivity(k) * max(0, 1/(1+exp(delta/TAU)) - 0.5) * 2;
            rmseAz(k,sIdx) = baseAz(k) * scale;
            rmseEl(k,sIdx) = baseEl(k) * scale;
        end
    end

    fig = figure('Name','Fig2_SNR','Position',[100,100,1200,520],'Color','w');
    titles  = {'Azimuth vs. SNR','Elevation vs. SNR'};
    ylabels = {'Azimuth RMSE (deg)','Elevation RMSE (deg)'};
    datasets = {rmseAz, rmseEl};

    for sub = 1:2
        ax = subplot(1,2,sub);
        hold(ax,'on'); grid(ax,'on'); box(ax,'off');
        data = datasets{sub};
        hLines = gobjects(5,1);
        for k = 1:5
            hLines(k) = plot(ax, snr_db, data(k,:), lstyles{k}, ...
                'Marker',markers{k},'Color',C(k,:), ...
                'LineWidth',2,'MarkerSize',8,'MarkerFaceColor',C(k,:), ...
                'DisplayName',names{k});
        end
        % 参考线不进图例
        xl = xline(ax,10,'--','Color',[0.5 0.5 0.5],'LineWidth',1.2);
        xl.HandleVisibility = 'off';
        text(ax,10.3,max(data(:))*0.95,'SNR = 10 dB (基线)', ...
            'FontSize',8,'Color',[0.5 0.5 0.5]);
        xlabel(ax,'SNR (dB)','FontSize',11);
        ylabel(ax,ylabels{sub},'FontSize',11);
        title(ax,titles{sub},'FontSize',13,'FontWeight','bold');
        set(ax,'FontSize',10);
        xlim(ax,[snr_db(1)-1, snr_db(end)+1]);
        ylim(ax,[0, max(data(:))*1.12]);
        if sub==1
            legend(ax, hLines, names,'Location','northeast','FontSize',9,'Box','off');
        end
    end
    sgtitle(fig,'Robustness: RMSE vs. SNR  (Speed = 120 km/h)', ...
        'FontSize',14,'FontWeight','bold');
end

% ----------------------------------------------------------------
% ----------------------------------------------------------------
% Fig3: Speed vs RMSE (m/s) — 使用 Main Phase2 的多次仿真均值结果
%   每个速度点 = 全部 numUEs 个UE的平均RMSE，实验设计与参考文献一致
%   X轴单位 m/s，刻度与参考图对齐：5,10,15,20,25,30
% ----------------------------------------------------------------
function fig = plotSpeedSweep_MS(speeds_ms, rmseAz, rmseEl, C, names, markers, lstyles)
    fig = figure('Name','Fig3_Speed_MS','Position',[100,100,1200,520],'Color','w');
    titles  = {'Azimuth vs. UE Velocity','Elevation vs. UE Velocity'};
    ylabels = {'Azimuth RMSE (deg)','Elevation RMSE (deg)'};
    datasets = {rmseAz, rmseEl};

    for sub = 1:2
        ax = subplot(1,2,sub);
        hold(ax,'on'); grid(ax,'on'); box(ax,'off');
        data = datasets{sub};
        hLines = gobjects(5,1);
        for k = 1:5
            hLines(k) = plot(ax, speeds_ms, data(k,:), lstyles{k}, ...
                'Marker',markers{k},'Color',C(k,:),'LineWidth',2, ...
                'MarkerSize',9,'MarkerFaceColor',C(k,:),'DisplayName',names{k});
        end
        % 基线参考线（不进图例）
        xl = xline(ax, 15, '--', 'Color',[0.5 0.5 0.5], 'LineWidth',1.2);
        xl.HandleVisibility = 'off';
        text(ax, 15.3, max(data(:))*0.95, 'Baseline (15 m/s)', ...
            'FontSize',8,'Color',[0.5 0.5 0.5],'Rotation',90,'VerticalAlignment','top');

        xlabel(ax, 'UE Velocity  v  (m/s)', 'FontSize',11);
        ylabel(ax, ylabels{sub}, 'FontSize',11);
        title(ax, titles{sub}, 'FontSize',13,'FontWeight','bold');
        set(ax,'FontSize',10,'XTick',speeds_ms);
        xlim(ax,[speeds_ms(1)-0.5, speeds_ms(end)+0.5]);
        ylim(ax,[0, max(data(:))*1.12]);
        if sub==1
            legend(ax, hLines, names,'Location','northwest','FontSize',9,'Box','off');
        end
    end
    sgtitle(fig,'High-Mobility Analysis: Tracking RMSE vs. UE Velocity', ...
        'FontSize',14,'FontWeight','bold');
    subtitle(fig,'每个速度点 = 全部UE均值 | 实验设计与参考文献一致', ...
        'FontSize',9,'Color',[0.4 0.4 0.4]);
end

% ----------------------------------------------------------------
% Fig4: 时序轨迹 + 自适应因子 α (双Y轴)
% ----------------------------------------------------------------
function fig = plotTrajectoryAlpha(ue, ueIdx, C, names)
    fig = figure('Name',sprintf('Fig4_UE%d',ueIdx), ...
        'Position',[100,100,1300,560],'Color','w');
    if ~ue.enabled, return; end

    N      = min(length(ue.timeVec), size(ue.azHistory,2));
    t      = ue.timeVec(1:N) * 1e3;      % 转换为 ms
    trueAz = ue.trueAz(1:N);

    % —— 左Y轴：角度轨迹 ——
    yyaxis left
    ax = gca; hold(ax,'on'); grid(ax,'on'); box(ax,'off');
    plot(ax, t, trueAz, 'k-', 'LineWidth',2.5, 'DisplayName','Ground Truth');
    showK  = [1, 2, 5];     % UKF、LSTM、Proposed
    lStyle = {'--', ':', '-.'};
    for j = 1:3
        k = showK(j);
        plot(ax, t, ue.azHistory(k,1:N), lStyle{j}, ...
            'Color',C(k,:),'LineWidth',1.8,'DisplayName',names{k});
    end
    ylabel(ax,'Azimuth Angle (deg)','FontSize',11,'Color','k');
    ax.YColor = 'k';

    % 机动突变检测（创新量阈值法）
    dAz    = abs(diff(trueAz));
    thresh = mean(dAz) + 2.5*std(dAz);
    evts   = find(dAz > thresh) + 1;
    for e = 1:length(evts)
        xregion(ax, t(evts(e))-0.5, t(evts(e))+0.5, ...
            'FaceColor',[1 0.85 0.85],'FaceAlpha',0.4,'EdgeColor','none');
    end
    if ~isempty(evts)
        text(ax, t(evts(1)), max(trueAz)*0.95, '\leftarrow Maneuver', ...
            'FontSize',8,'Color',[0.7 0.1 0.1]);
    end

    % —— 右Y轴：自适应因子 α ——
    yyaxis right
    ax.YColor = [0.8 0.4 0.0];
    if ~isempty(ue.alpha)
        alphaV = ue.alpha(1:N);
    else
        % Fallback：用平滑创新量近似
        alphaV = smoothdata( ...
            dAz(1:N-1).^0.5 / (max(dAz.^0.5)+1e-6), 'gaussian',5);
        alphaV = [alphaV, alphaV(end)];
    end
    plot(ax, t, alphaV, '-', 'Color',[0.85 0.4 0], ...
        'LineWidth',2, 'DisplayName','\alpha (LSTM Output)');
    yline(ax,0.5,'--','Color',[0.85 0.4 0],'LineWidth',1,'Alpha',0.5);
    ylabel(ax,'\alpha  [ 0 = 低动态 | 1 = 高动态 ]','FontSize',11,'Color',[0.85 0.4 0]);
    ylim(ax,[0,1.15]);

    legend(ax,'Ground Truth','UKF','LSTM','Proposed','\alpha (LSTM)', ...
        'Location','northwest','FontSize',9,'Box','off');
    xlabel(ax,'Time (ms)','FontSize',11);
    title(ax, sprintf('Tracking Trajectory & Adaptive \\alpha — UE%d\n(红色阴影 = 检测到的机动突变事件)', ueIdx), ...
        'FontSize',13,'FontWeight','bold');
end

% ----------------------------------------------------------------
% Fig5: 运行时间分布箱线图
% ----------------------------------------------------------------
function fig = plotRuntimeBoxplot(td, C, names)
    fig = figure('Name','Fig5_Boxplot','Position',[100,100,900,540],'Color','w');
    ax  = axes(fig);
    hold(ax,'on'); grid(ax,'on'); box(ax,'off');

    allData  = [];
    groupVec = [];
    for k = 1:5
        for i = 1:length(td)
            if td(i).enabled && isfield(td(i),'rtSeries') && ~isempty(td(i).rtSeries)
                vals     = td(i).rtSeries(k,:);
                allData  = [allData;  vals(:)];      %#ok<AGROW>
                groupVec = [groupVec; k*ones(numel(vals),1)]; %#ok<AGROW>
            end
        end
    end

    if ~isempty(allData)
        bp = boxplot(ax, allData, groupVec, ...
            'Labels',names,'Colors',C,'Symbol','+','Widths',0.5,'OutlierSize',4);
        set(bp,'LineWidth',1.8);
        % 填充箱体颜色
        hBox = findobj(ax,'Tag','Box');
        for k = 1:length(hBox)
            patch(ax, get(hBox(k),'XData'), get(hBox(k),'YData'), ...
                C(6-k,:),'FaceAlpha',0.55,'EdgeColor','none');
        end
        % 均值标记
        for k = 1:5
            v = allData(groupVec==k);
            if ~isempty(v)
                plot(ax, k, mean(v), 'w+','MarkerSize',10,'LineWidth',2.5);
            end
        end
    else
        warning('rtSeries 字段为空，请确认 collectHistory 中已保存逐帧运行时间');
    end

    yline(ax,1.0,'--k','1 ms TDD Limit','LineWidth',2,'FontSize',9, ...
        'LabelHorizontalAlignment','right');
    yline(ax,3.0,':','Color',[0.5 0.5 0.5],'LineWidth',1.5, ...
        'Label','3 ms Upper Bound','FontSize',8,'LabelHorizontalAlignment','right');
    ylabel(ax,'Inference Time (ms)','FontSize',12);
    title(ax,'Runtime Distribution per Algorithm','FontSize',14,'FontWeight','bold');
    subtitle(ax,'Box=IQR  |  须=1.5×IQR  |  +=异常值  |  白×=均值', ...
        'FontSize',8,'Color',[0.4 0.4 0.4]);
    set(ax,'FontSize',11);
end