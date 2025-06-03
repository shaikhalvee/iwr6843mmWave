%-----------------------------------------------------------------------
function err = sensor_advance_frame_BF(devId, p, Tx1_deg, Tx2_deg, Tx3_deg)
% Thin wrapper around TI's RtttNetClientAPI for *one* AWR1243 device.
%   devId ∈ {1,2,3,4} maps to master & 3 slaves.

    import RtttNetClientAPI.*             

    radarId       = [1 2 4 8];            % hard‑coded by TI
    trigSelect    = [1 2 2 2];            % SW trigger for master, HW for slaves

    %-------------------------------------------------- Profile CONFIG
    lua = sprintf([ ...
        "ar1.ProfileConfig_mult(%d,0,%.3f,%.3f,%.3f,%.3f,0,0,0,0,0,0," + ...
        "%.3f,%.3f,%d,%d,2,1,%d)" ], ...
        radarId(devId), p.Start_Freq_GHz, p.Idle_Time_us, p.Adc_Start_Time_us, ...
        p.Ramp_End_Time_us, p.Slope_MHzperus, p.Tx_Start_Time_us, ...
        p.Samples_per_Chirp, p.Sampling_Rate_ksps, p.Rx_Gain_dB);

    err = RtttNetClient.SendCommand(lua);
    if err~=30000, return; end

    %-------------------------------------------------- Chirp CONFIG
    TXenable = zeros(3,4);
    TXenable(p.Tx_Ant_Arr_BF) = 1;           % map used channels

    for k = 0:p.NumAnglesToSweep-1
        lua = sprintf("ar1.ChirpConfig_mult(%d,%d,%d,0,0,0,0,0,%d,%d,%d)", ...
            radarId(devId), k, k, TXenable(1,devId), TXenable(2,devId), TXenable(3,devId));
        err = RtttNetClient.SendCommand(lua);
        if err~=30000, return; end

        lua = sprintf("ar1.SetPerChirpPhaseShifterConfig_mult(%d,%d,%d,%d,%d,%d)", ...
            radarId(devId), k, k, Tx1_deg(k+1,devId), Tx2_deg(k+1,devId), Tx3_deg(k+1,devId));
        err = RtttNetClient.SendCommand(lua);
        if err~=30000, return; end
    end

    %-------------------------------------------------- Advanced FRAME CONFIG
    lua = sprintf("ar1.AdvanceFrameConfig_mult(%d,%d,1536,0,%d,%d,%d,%d,%d,%d,%d,%d,0, ..." + ...
        "%d,%d,%d,%d,%d,%d,%d,%d,0,0,1,128,8000000,0,1,1,8000000,0,0,1,128,8000000,0,1,1,8000000,%d,%d,0,0,128,256,1,128,256,1,128,1,1,128,1,1)", ...
        radarId(devId), p.numSubFrames, ...
        p.SF1ChirpStartIdx, p.SF1NumChirps, p.SF1NumLoops, p.SF1BurstPeriodicity, ...
        p.SF1ChirpStartIdxOffset, p.SF1NumBurst, p.SF1NumBurstLoops, p.SF1SubFramePeriodicity, ...
        p.SF1ChirpStartIdx, p.SF1NumChirps, p.SF1NumLoops, p.SF1BurstPeriodicity, ...
        p.SF1ChirpStartIdxOffset, p.SF1NumBurst, p.SF1NumBurstLoops, p.SF1SubFramePeriodicity, ...
        p.Num_Frames, trigSelect(devId));

    err = RtttNetClient.SendCommand(lua);

    if err==30000
        fprintf("[Dev %d] configuration OK\n", devId);
    end
end
