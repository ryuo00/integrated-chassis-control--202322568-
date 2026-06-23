function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target
%
%   ----------------------------------------------------------------------
%   [설계 v6 — Bang-bang ABS]
%     v5(비례 제어) 실측 결과: 슬립이 -0.005→-0.64 까지 폭주, brake 를
%     줄여도 슬립이 멈추지 않음 (1kHz 고정 스텝 대비 슬립 변화가 너무
%     빨라 부드러운 비례 반응이 따라가지 못함, 또는 타이어가 peak μ 를
%     넘어 falling-friction 영역에 진입).
%
%     해결: ASSIGNMENT 힌트의 bang-bang 방식 채택. brakeRatio 를 0/1 이진
%     스위치로 사용 — |slip| > kappaTarget 이면 즉시 brakeRatio=0.5(brake
%     절반), 그 아래로 내려오면 즉시 brakeRatio=1(brake 전체 복원). 부드러운
%     비례 게인이 아니라 즉각적인 on/off 전환이라 빠른 슬립 변화에도
%     반응 지연이 없음. brakeRatio 는 coordinator 가 시나리오 강제
%     brakeTorque 에 직접 곱하는 형태로 사용 (Fx_total 경로 대신).
%   ----------------------------------------------------------------------

    %% Tuning parameters
    massApprox   = 1600;     % [kg]
    kappaTarget  = 0.12;     % ABS 목표 슬립비 (절댓값)
    decelGate    = -0.5;     % [m/s^2] ax 기준 강제제동 판정
    largeErrGate = 2.0;      % [m/s] err 기준 강제제동 판정 (ax 지연 보완)

    %% 상태 초기화
    if ~isfield(ctrlState, 'intError');   ctrlState.intError   = 0; end
    if ~isfield(ctrlState, 'prevForce');  ctrlState.prevForce  = 0; end
    if ~isfield(ctrlState, 'wheelSlip');  ctrlState.wheelSlip  = zeros(4,1); end

    %% (1) 속도추종 PI (cruise 구간 전용 — 강제제동 중엔 가속요구 억제)
    err = vxRef - vx;
    isBraking = (ax < decelGate) || (err > largeErrGate);

    intCandidate = ctrlState.intError + err * dt;
    intCandidate = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, intCandidate));

    Fx_unsat = CTRL.LON.Kp * err + CTRL.LON.Ki * intCandidate;

    Fx_cap = LIM.MAX_AX * massApprox;
    Fx_pi = max(-Fx_cap, min(Fx_cap, Fx_unsat));

    if isBraking && Fx_pi > 0
        Fx_pi = 0;
    end

    if abs(Fx_unsat - Fx_pi) < 1e-6
        ctrlState.intError = intCandidate;
    end

    %% (3) Jerk limit — Fx_total(PI 경로만) 변화율 cap
    maxDeltaF = LIM.MAX_JERK * massApprox * dt;
    dF = Fx_pi - ctrlState.prevForce;
    dF = max(-maxDeltaF, min(maxDeltaF, dF));
    Fx_total = ctrlState.prevForce + dF;

    ctrlState.prevForce = Fx_total;

    %% (2) ABS — Bang-bang brakeRatio
    %      이진 전환: 슬립 초과 시 0.5, 그 외 1.0 (시나리오 brake 전체 유지)
    brakeRatio = 1.0;
    if isBraking
        slip = ctrlState.wheelSlip;
        if any(abs(slip) > kappaTarget)
            brakeRatio = 0.65;
        end
    end

    %% Outputs
    forceCmd.Fx_total   = Fx_total;
    forceCmd.brakeRatio = brakeRatio;

end