function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad]
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm]
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가
%       3. vx 적응
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - β-limiter 는 slip angle 초과 시 반대 방향 yaw moment를 인가

    %% Tuning parameters
    v_ref   = 15.0;              % [m/s]
    maxAFS  = deg2rad(4.0);      % [rad]
    BETA_TH = deg2rad(1.0);      % [rad]
    K_BETA  = 12000;             % [Nm/rad]
    M_MAX   = 5000;              % [Nm]

    %% 상태 초기화
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
    end

    if ~isfield(ctrlState, 'prevError')
        ctrlState.prevError = 0;
    end

  %% (1) Yaw-rate PID based AFS
err = yawRateRef - yawRate;

dErrRaw = (err - ctrlState.prevError) / max(dt, 1e-4);
dErr = max(-2.0, min(2.0, dErrRaw));   % derivative limiting — D항 폭주 방지

intCandidate = ctrlState.intError + err * dt;
intCandidate = max(-CTRL.LAT.intMax, ...
                   min(CTRL.LAT.intMax, intCandidate));

afsSched = min(max(vx / v_ref, 0.4), 1.5);

steerUnsat = afsSched * ...
    (CTRL.LAT.Kp * err + ...
     CTRL.LAT.Ki * intCandidate + ...
     0.1 * CTRL.LAT.Kd * dErr);   % Kd 게인 1/10로 축소

    steerAFS = max(-maxAFS, min(maxAFS, steerUnsat));

    if abs(steerUnsat - steerAFS) < 1e-8
        ctrlState.intError = intCandidate;
    end

    ctrlState.prevError = err;

    %% (2) ESC beta-limiter
    escSched = min(max(vx / v_ref, 0.6), 2.0);

    if abs(slipAngle) > BETA_TH
        betaExcess = abs(slipAngle) - BETA_TH;
        yawMoment = -K_BETA * escSched * ...
                    sign(slipAngle) * betaExcess;
    else
        yawMoment = 0;
    end

    yawMoment = max(-M_MAX, min(M_MAX, yawMoment));

    %% Outputs
    deltaAdd.steerAngle = steerAFS;
    deltaAdd.yawMoment  = yawMoment;

end