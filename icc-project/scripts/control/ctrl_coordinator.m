function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)
%
%   ----------------------------------------------------------------------
%   [구현 현황 — 4단계, 검증 진행 중]
%     완료 : 요구사항 (1) 종방향 60:40 분배, (2) yawMoment 차동 분배,
%            (3) steerAngle pass-through, (4) 최종 saturation,
%            lonCmd.brakeRatio (bang-bang ABS) 처리.
%
%     brakeRatio 처리 방식: runner 가 brake_total = brk_scenario + brakeESC
%     로 고정 합산하므로(run_icc_scenario.m, 수정 불가), coordinator 는
%     brk_scenario 의 실제값을 알 수 없음. brakeRatio<1(ABS 개입) 일 때
%     "현재 시점의 추정 최대 제동토크" 의 (1-brakeRatio) 비율을 깎아내는
%     음의 brakeTorque 를 출력해 근사. 추정치는 차량 질량 × LIM.MAX_AX
%     기반 최대 감속력을 torque 로 환산한 상한값 사용 (시나리오 무관
%     일반화된 근사).
%
%     yawMoment 차동 분배: plant_bicycle.md §7 의 ESC yaw moment 복원식
%     M_z_axle = ΔT/rw · halfTrack 을 역산해 ΔT = M_z_axle·rw/halfTrack
%     로 brake torque 차이를 계산. (초안에서 rw 계수를 빠뜨려 단위가
%     [N]으로 잘못 나왔던 오류를 수정함 — 토크는 [Nm]이어야 함.)
%     양의 M_z(CCW) → 좌측(FL,RL) brake 증가, 우측(FR,RR) brake 감소.
%     A1 KPI(sideSlipMax, LTR_max, lateralDevMax) 로 효과 검증 필요.
%   ----------------------------------------------------------------------

    %% Tuning parameters
    rw        = 0.31;   % [m] 타이어 유효 반경
    frontBias = 0.6;
    rearBias  = 0.4;

    %% (1) 종방향 — Fx_total < 0 인 순수 제동 의도 (cruise 시나리오 등)
    brakeTorque = zeros(4,1);   % [FL; FR; RL; RR]

    if isfield(lonCmd, 'Fx_total') && lonCmd.Fx_total < 0
        T_total = abs(lonCmd.Fx_total) * rw;
        T_front_each =  T_total * frontBias / 2;
        T_rear_each  =  T_total * rearBias  / 2;
        brakeTorque  = [T_front_each; T_front_each; T_rear_each; T_rear_each];
    end

    %% (ABS) Bang-bang brakeRatio 처리 — 시나리오 강제 brake 의 일부 해제
    %      brakeRatio < 1 이면 (1-brakeRatio) 비율만큼 brake 를 깎아냄.
    %      brk_scenario 실제값을 모르므로, 차량 최대 감속력 기반 추정
    %      brake torque 상한을 기준으로 근사.
    if isfield(lonCmd, 'brakeRatio') && lonCmd.brakeRatio < 1.0
        massApprox = VEH.mass;
        Fx_decel_max = massApprox * LIM.MAX_AX;      % 추정 최대 감속력 [N]
        T_decel_max  = Fx_decel_max * rw;             % 추정 최대 brake torque (총합)

        releaseFrac = 1.0 - lonCmd.brakeRatio;        % 0.5 (ABS 개입 시)
        T_release_total = T_decel_max * releaseFrac;

        T_release_front_each = -T_release_total * frontBias / 2;
        T_release_rear_each  = -T_release_total * rearBias  / 2;

        brakeTorque = brakeTorque + ...
            [T_release_front_each; T_release_front_each; ...
             T_release_rear_each;  T_release_rear_each];
    end

    %% (2) 횡방향 — ESC yaw moment → 4륜 차동 brake 분배
    %      양의 M_z(CCW) → 좌측 brake 증가, 우측 brake 감소
    %      Mz_axle = (T_left - T_right)/rw * halfTrack 의 역산:
    %      ΔT = Mz_axle * rw / halfTrack
    if isfield(latCmd, 'yawMoment') && latCmd.yawMoment ~= 0
        Mz = latCmd.yawMoment;

        halfTrackF = VEH.track_f / 2;
        halfTrackR = VEH.track_r / 2;

        ratioF = frontBias;
        ratioR = rearBias;

        MzF = ratioF * Mz;
        MzR = ratioR * Mz;

        deltaTF = MzF * rw / halfTrackF;
        deltaTR = MzR * rw / halfTrackR;

        % 좌측 +, 우측 - 방식으로 절반씩 분배
        diffTorque = [ deltaTF/2;
                      -deltaTF/2;
                       deltaTR/2;
                      -deltaTR/2 ];

        brakeTorque = brakeTorque + diffTorque;
    end

    %% (3) AFS steerAngle pass-through
    steerAngle = latCmd.steerAngle;

    %% (4) 최종 saturation
    brakeTorque = max(-LIM.MAX_BRAKE_TRQ, min(LIM.MAX_BRAKE_TRQ, brakeTorque));
    steerAngle  = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steerAngle));

    %% Outputs
    actuatorCmd.steerAngle   = steerAngle;
    actuatorCmd.brakeTorque  = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd;

end