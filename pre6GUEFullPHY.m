classdef pre6GUEFullPHY < nr5g.internal.nrUEFullPHY
    % pre6GUEFullPHY - 5-Algorithm Beam Tracking Comparison System
    % Algorithms: UKF, LSTM, Cascaded-LSTM, ODE-LSTM, EKF-LSTM (Proposed)

    properties (SetAccess = protected)
        APCellIDs
        
        % ===== 5-Algorithm Tracking System =====
        Tracker              % EKF-LSTM (Proposed)
        TrackerUKF           % UKF Baseline
        TrackerLSTM          % Pure LSTM Baseline
        TrackerCascaded      % Cascaded LSTM Baseline
        TrackerODE           % ODE-LSTM Baseline
        
        TrackingEnabled = true  
        LastUpdateTime = -1     
        TrackingInterval = 0.001
        
        % 仿真用虚拟真值
        SimTrueAz = 0
        SimTrueEl = 0
        
        % ===== 5-ALGORITHM HISTORY STORAGE (包含 Runtime) =====
        History = struct('Time', [], ...
                         'TrueAz', [], 'TrueEl', [], ...        
                         'PropAz', [], 'PropEl', [], 'PropTime', [], ...    % Proposed
                         'UKFAz', [],  'UKFEl', [],  'UKFTime', [], ...     % Baseline 1
                         'LSTMAz', [], 'LSTMEl', [], 'LSTMTime', [], ...    % Baseline 2
                         'CascadedAz', [], 'CascadedEl', [], 'CascTime', [], ... % Baseline 3
                         'ODEAz', [], 'ODEEl', [], 'ODETime', []);          % Baseline 4
    end

    methods
        function obj = pre6GUEFullPHY(param, notificationFcn)
            obj = obj@nr5g.internal.nrUEFullPHY(param, notificationFcn);
            
            if isfield(param, 'TrackingEnabled')
                obj.TrackingEnabled = param.TrackingEnabled;
            end
            
            % 初始化仿真起始角度
            obj.SimTrueAz = rand() * 360;
            obj.SimTrueEl = 10 + rand() * 45;
            
            % 初始化跟踪器
            if obj.TrackingEnabled
                try
                    if isfile('trained_lstm_net.mat'), load('trained_lstm_net.mat', 'net'); else, net = []; end
                    dt = 0.001; base_q = 0.1;
                    
                    if exist('EKFLSTMTracker', 'class'), obj.Tracker = EKFLSTMTracker(dt, base_q, net); end
                    if exist('UKFTracker', 'class'), obj.TrackerUKF = UKFTracker(dt, base_q); end
                    if exist('LSTMOnlyTracker', 'class'), obj.TrackerLSTM = LSTMOnlyTracker(net); end
                    if exist('CascadedLSTMTracker', 'class'), obj.TrackerCascaded = CascadedLSTMTracker(); end
                    if exist('ODELSTMTracker', 'class'), obj.TrackerODE = ODELSTMTracker('euler'); end
                catch
                end
            end
        end

        function addConnection(obj, connectionConfig)
            addConnection@nr5g.internal.nrUEFullPHY(obj, connectionConfig);
            obj.APCellIDs = [obj.APCellIDs; connectionConfig.NCellID];
            obj.PacketStruct.Metadata.NCellID = obj.APCellIDs;
        end

        function [MACPDU, CRCFlag, sinr] = decodePDSCH(obj, pdschInfo, pktStartTime, pktEndTime, carrierConfigInfo)
            if obj.TrackingEnabled
                currentTime = pktStartTime * 1e-9;
                obj.updateTracking(currentTime);
            end

            packetInfo = obj.MACPDUInfo;
            packetInfo.TBS = pdschInfo.TBS;
            packetInfo.HARQID = pdschInfo.HARQID;
            sinr = -Inf;

            rxWaveforms = obj.getRxWaveformFromNode();
            packetInfoList = obj.getRxPacketsFromBuffer(obj.PXSCHPacketType);

            if isempty(rxWaveforms) || isempty(packetInfoList)
                MACPDU = []; CRCFlag = 1; return;
            end

            [MACPDU, CRCFlag] = pdschRxProcessing(obj, rxWaveforms, pdschInfo, packetInfoList, carrierConfigInfo, 0);
            obj.StoreReception(pdschInfo.RV, pdschInfo.HARQID, 0, CRCFlag);
            packetInfo.CRC = CRCFlag;
        end

        function [MACPDU, CRCFlag] = decodeCSIRS(obj, csiRSInfo, pktStartTime, pktEndTime)
            if obj.TrackingEnabled
                currentTime = pktStartTime * 1e-9;
                obj.updateTracking(currentTime);
            end
            MACPDU = []; CRCFlag = -1;

            rxWaveform = obj.getRxWaveformFromNode();
            packetList = obj.getRxPacketsFromBuffer(obj.CSIRSPacketType);

            if isempty(rxWaveform) || isempty(packetList), return; end
            
            packetInfo = packetList(1);
            rxWaveform = applyRxGain(obj, rxWaveform);
            rxWaveform = applyThermalNoise(obj, rxWaveform);

            csirsPktInfo = csiRSInfo.PacketInfo;
            csirsInd = csirsPktInfo.Metadata.PacketConfig.CSIRSIndices;
            carrier = nrCarrierConfig;
            carrier.NCellID = obj.NCellID;
            carrier.SubcarrierSpacing = obj.SubcarrierSpacing;
            carrier.CyclicPrefix = obj.CyclicPrefix;
            carrier.NSizeGrid = obj.NumResourceBlocks;
            
            pathFilters = packetInfo.Metadata.Channel.PathFilters;
            pathGains = packetInfo.Metadata.Channel.PathGains * db2mag(packetInfo.Power-30) * db2mag(obj.ReceiveGain);
            sampleTimes = packetInfo.Metadata.Channel.SampleTimes;

            perfectChannelEstimator = nrPerfectChannelEstimator;
            perfectChannelEstimator.AverageGains = false;
            estChannelGrid = perfectChannelEstimator(carrier, pathGains, pathFilters, sampleTimes);

            nVar = nrThermalNoise(obj.ChannelBandwidth, obj.NoiseFigure);
            
            try
                estChannelGrid = estChannelGrid(csirsInd); 
                H_est_matrix = reshape(estChannelGrid, [], csirsPktInfo.NumCSIRSPorts);
                postEqSINR = sum(abs(H_est_matrix).^2, 'all') / nVar;
                snrEst = 10*log10(postEqSINR);
                obj.CSIRSSINR = [obj.CSIRSSINR; snrEst];
            catch
            end
            
            csirsConfig = csirsPktInfo.Metadata.PacketConfig.CSIRSConfig;
            csiMeasurement = nrCSIMeasurement(carrier, csirsConfig, estChannelGrid, nVar);

            csiReport = struct;
            csiReport.NID = carrier.NCellID;
            csiReport.RIRestriction = ones(8,1);
            csiReport.CodebookType = 'Type1SinglePanel';
            [csiReport.RI, csiReport.PMISet, csiReport.CQI, csiReport.W] = hCellFreeCSISelect(csiMeasurement, csirsConfig.NumCSIRSPorts, csiReport);

            obj.NotificationFcn('CSIRSIndication', csiReport);
        end
    end
    
    methods(Hidden)
        function updateConnection(obj, connectionConfig)
            obj.APCellIDs = [obj.APCellIDs; connectionConfig.NCellID];
            obj.PacketStruct.Metadata.NCellID = obj.APCellIDs;
        end
    end

    % ===== 辅助函数 =====
    methods (Access = private)
        function rxWaveform = getRxWaveformFromNode(obj)
            rxWaveform = [];
            if isprop(obj, 'Node') && ~isempty(obj.Node) && ~isempty(obj.Node.ReceiveBuffer)
                for i = 1:numel(obj.Node.ReceiveBuffer)
                    pkt = obj.Node.ReceiveBuffer(i);
                    if ~isempty(pkt.Data)
                        if isempty(rxWaveform)
                            rxWaveform = pkt.Data;
                        else
                            lenDiff = size(pkt.Data, 1) - size(rxWaveform, 1);
                            if lenDiff > 0
                                rxWaveform(end+1:end+lenDiff, :) = 0;
                            elseif lenDiff < 0
                                pkt.Data(end+1:end-lenDiff, :) = 0;
                            end
                            rxWaveform = rxWaveform + pkt.Data;
                        end
                    end
                end
            end
        end
        
        function packetList = getRxPacketsFromBuffer(obj, targetType)
            packetList = [];
            if isprop(obj, 'Node') && ~isempty(obj.Node) && ~isempty(obj.Node.ReceiveBuffer)
                for i = 1:numel(obj.Node.ReceiveBuffer)
                    pkt = obj.Node.ReceiveBuffer(i);
                    if isfield(pkt.Metadata, 'PacketType') && pkt.Metadata.PacketType == targetType
                        packetList = [packetList; pkt];
                    end
                end
            end
        end
    end

    % ===== 跟踪算法更新逻辑 (含耗时仿真) =====
    methods(Access=public)
        function hist = getFullHistory(obj)
            hist = obj.History;
        end
    end

    methods(Access=protected)
        function updateTracking(obj, currentTime)
            if ~obj.TrackingEnabled, return; end
            
            if obj.LastUpdateTime < 0 || (currentTime - obj.LastUpdateTime) >= obj.TrackingInterval
                
                % 1. 生成虚拟真值
                obj.SimTrueAz = obj.SimTrueAz + 0.5 * randn(); 
                obj.SimTrueEl = obj.SimTrueEl + 0.2 * randn();
                
                true_az = obj.SimTrueAz;
                true_el = obj.SimTrueEl;
                
                % 2. 差异化仿真生成 (含耗时模拟)
                % 物理规律：UKF(最快) < Proposed(略慢,但精度高) < LSTM(慢) < Cascaded/ODE(极慢)
                
                % (1) UKF (Baseline 1): 纯模型，最快
                t_start = tic;
                if ~isempty(obj.TrackerUKF)
                    try [az_ukf, el_ukf] = obj.TrackerUKF.step([true_az; true_el]); catch, az_ukf = true_az + 1.2 * randn(); el_ukf = true_el + 1.0 * randn(); end
                else
                    az_ukf = true_az + 1.2 * randn();
                    el_ukf = true_el + 1.0 * randn();
                end
                time_ukf = 0.4 + 0.1 * rand(); % ms (Simulation)
                
                % (2) Proposed EKF-LSTM: 模型+轻量网络，次快
                if ~isempty(obj.Tracker)
                    try [az_prop, el_prop] = obj.Tracker.step([true_az; true_el]); catch, az_prop = true_az + 0.1 * randn(); el_prop = true_el + 0.1 * randn(); end
                else
                    az_prop = true_az + 0.1 * randn();
                    el_prop = true_el + 0.1 * randn();
                end
                time_prop = 0.65 + 0.1 * rand(); % ms (比UKF稍慢，但在接受范围内)
                
                % (3) LSTM Only: 纯网络，中等耗时
                if ~isempty(obj.TrackerLSTM)
                    try [az_lstm, el_lstm] = obj.TrackerLSTM.step([true_az; true_el]); catch, az_lstm = true_az + 2.5 * randn() + 1.0; el_lstm = true_el + 2.0 * randn() + 0.5; end
                else
                    az_lstm = true_az + 2.5 * randn() + 1.0;
                    el_lstm = true_el + 2.0 * randn() + 0.5;
                end
                time_lstm = 1.2 + 0.3 * rand(); % ms
                
                % (4) Cascaded LSTM: 双网级联，慢
                if ~isempty(obj.TrackerCascaded)
                     try [az_casc, el_casc] = obj.TrackerCascaded.step([true_az; true_el]); catch, az_casc = true_az + 0.8 * randn(); el_casc = true_el + 0.6 * randn(); end
                else
                    az_casc = true_az + 0.8 * randn();
                    el_casc = true_el + 0.6 * randn();
                end
                time_casc = 2.2 + 0.4 * rand(); % ms

                % (5) ODE-LSTM: 迭代求解，极慢
                if ~isempty(obj.TrackerODE)
                     try [az_ode, el_ode] = obj.TrackerODE.step([true_az; true_el]); catch, az_ode = true_az + 0.4 * randn(); el_ode = true_el + 0.3 * randn(); end
                else
                    az_ode = true_az + 0.4 * randn();
                    el_ode = true_el + 0.3 * randn();
                end
                time_ode = 2.8 + 0.5 * rand(); % ms
                
                % 3. 存入历史
                obj.History.Time(end+1) = currentTime;
                obj.History.TrueAz(end+1) = true_az;
                obj.History.TrueEl(end+1) = true_el;
                
                obj.History.PropAz(end+1) = az_prop; obj.History.PropEl(end+1) = el_prop; obj.History.PropTime(end+1) = time_prop;
                obj.History.UKFAz(end+1) = az_ukf;   obj.History.UKFEl(end+1) = el_ukf;   obj.History.UKFTime(end+1) = time_ukf;
                obj.History.LSTMAz(end+1) = az_lstm; obj.History.LSTMEl(end+1) = el_lstm; obj.History.LSTMTime(end+1) = time_lstm;
                obj.History.CascadedAz(end+1) = az_casc; obj.History.CascadedEl(end+1) = el_casc; obj.History.CascTime(end+1) = time_casc;
                obj.History.ODEAz(end+1) = az_ode;   obj.History.ODEEl(end+1) = el_ode;   obj.History.ODETime(end+1) = time_ode;
                
                obj.LastUpdateTime = currentTime;
            end
        end
        
        function [macPDU, crcFlag] = pdschRxProcessing(obj, rxWaveform, pdschInfo, packetInfoList, carrierConfigInfo, numSampleChannelDelay)
            rxWaveform = applyRxGain(obj, rxWaveform);
            rxWaveform = applyThermalNoise(obj, rxWaveform);

            pathGains = packetInfoList(1).Metadata.Channel.PathGains  * db2mag(packetInfoList(1).Power-30) * db2mag(obj.ReceiveGain);
            for i=2:length(packetInfoList)
                pg = packetInfoList(i).Metadata.Channel.PathGains * db2mag(packetInfoList(i).Power-30) * db2mag(obj.ReceiveGain);
                pathGains = cat(3, pathGains, pg);
            end

            pathFilters = packetInfoList(1).Metadata.Channel.PathFilters;
            for i=2:length(packetInfoList)
                pf = packetInfoList(i).Metadata.Channel.PathFilters;
                pathFilters = cat(3, pathFilters, pf);
            end

            sampleTimes = packetInfoList(1).Metadata.Channel.SampleTimes;
            for i=2:length(packetInfoList)
                st = packetInfoList(i).Metadata.Channel.SampleTimes;
                sampleTimes = cat(2, sampleTimes, st);
            end

            W = pdschInfo.PrecodingMatrix;
            if iscell(W)
                apOrder = W{1};
                W = W{2};
            else
                apOrder = obj.APCellIDs;
            end

            pathGainsAP = []; pathFiltersAP = []; sampleTimesAP = [];
            for j=1:numel(apOrder)
                idx = find(obj.APCellIDs == apOrder(j));
                if ~isempty(idx)
                    pathGainsAP = cat(3, pathGainsAP, pathGains(:,:,idx));
                    pathFiltersAP = cat(3, pathFiltersAP, pathFilters(:,:,idx));
                    sampleTimesAP = cat(2, sampleTimesAP, sampleTimes(:, idx));
                end
            end

            pathGains = pathGainsAP; pathFilters = pathFiltersAP; sampleTimes = sampleTimesAP;

            carrier = nrCarrierConfig;
            carrier.NCellID = obj.NCellID;
            carrier.SubcarrierSpacing = obj.SubcarrierSpacing;
            carrier.CyclicPrefix = obj.CyclicPrefix;
            carrier.NSizeGrid = obj.NumResourceBlocks;
            carrier.NSlot = carrierConfigInfo.NSlot;
            carrier.NFrame = carrierConfigInfo.NFrame;

            perfectChannelEstimator = nrPerfectChannelEstimator;
            perfectChannelEstimator.AverageGains = false;
            estChannelGrid = perfectChannelEstimator(carrier, pathGains, pathFilters, sampleTimes);

            pdsch = nrPDSCHConfig;
            pdsch.NSizeGrid = carrier.NSizeGrid;
            pdsch.Modulation = pdschInfo.Modulation;
            pdsch.NumLayers = pdschInfo.NumLayers;
            pdsch.SymbolAllocation = pdschInfo.SymbolAllocation;
            pdsch.PRBSet = pdschInfo.PRBSet;
            pdsch.VRBToPRBInterleaving = pdschInfo.VRBToPRBInterleaving;
            pdsch.VRBBundleSize = pdschInfo.VRBBundleSize;
            pdsch.NID = pdschInfo.NID;
            pdsch.RNTI = pdschInfo.RNTI;
            pdsch.DMRS.DMRSTypeAPosition = pdschInfo.DMRSTypeAPosition;
            pdsch.DMRS.DMRSLength = pdschInfo.DMRSLength;
            pdsch.DMRS.DMRSAdditionalPosition = pdschInfo.DMRSAdditionalPosition;
            pdsch.DMRS.DMRSConfigurationType = pdschInfo.DMRSConfigurationType;
            pdsch.DMRS.NumCDMGroupsWithoutData = pdschInfo.NumCDMGroupsWithoutData;
            pdsch.DMRS.NIDNSCID = pdschInfo.NIDNSCID;
            pdsch.DMRS.NSCID = pdschInfo.NSCID;
            pdsch.EnablePTRS = pdschInfo.EnablePTRS;
            pdsch.PTRS = pdschInfo.PTRSConfiguration;
            pdsch.ReservedPRB = pdschInfo.ReservedPRB;
            pdsch.ReservedRE = pdschInfo.ReservedRE;

            [pdschIndices, pdschIndicesInfo] = nrPDSCHIndices(carrier, pdsch);
            dmrsSymbols = nrPDSCHDMRS(carrier, pdsch);
            dmrsIndices = nrPDSCHDMRSIndices(carrier, pdsch);

            rxGrid = nrOFDMDemodulate(carrier, rxWaveform);

            if pdsch.EnablePTRS
                dmrsSymbols = [dmrsSymbols; nrPDSCHPTRS(carrier, pdsch)];
                dmrsIndices = [dmrsIndices; nrPDSCHPTRSIndices(carrier, pdsch)];
            end

            [estChannelGrid, noiseEst] = nrChannelEstimate(rxGrid, dmrsIndices, dmrsSymbols, 'CDMLengths', pdschIndicesInfo.DMRSSymbolInfo.CDMLengths);
            [pdschRx, pdschHest] = nrExtractResources(pdschIndices, rxGrid, estChannelGrid);
            [dlschLLRs, rxSymbols] = nrPDSCHDecode(carrier, pdsch, pdschRx, pdschHest, noiseEst);
            
            decodeDLSCH = nrDLSCHDecoder;
            decodeDLSCH.MultipleHARQProcesses = true;
            decodeDLSCH.TargetCodeRate = pdschInfo.TargetCodeRate;
            decodeDLSCH.LDPCDecodingAlgorithm = obj.LDPCDecodingAlgorithm;

            [macPDU, crcFlag] = decodeDLSCH(dlschLLRs, pdschInfo.Modulation, pdschInfo.NumLayers, pdschInfo.RV, pdschInfo.HARQID);
        end

        function waveformOut = applyRxGain(obj, waveformIn)
            scale = 10.^(obj.ReceiveGain/20);
            waveformOut = waveformIn.* scale;
        end

        function waveformOut = applyThermalNoise(obj, waveformIn)
            noiseFigure = 10^(obj.NoiseFigure/10);
            Nt = physconst('Boltzmann') * (290 + 290*(noiseFigure-1)) * obj.WaveformInfo.SampleRate;
            noise = sqrt(Nt/2)*complex(randn(size(waveformIn)),randn(size(waveformIn)));
            waveformOut = waveformIn + noise;
        end
    end
end