[202322568-유지혁] ICC 제어기 설계 보고서
과목: 자동제어(C049-3) - 2026 1학기
제출일: 2000-04-01
개인
---
1. 설계 개요
본 과제의 목표는 BMW_5 14DOF 차량 모델에 대해 횡방향(AFS+ESC), 종방향(속도추종+ABS), 수직(CDC), 그리고 액추에이터 배분(Coordinator)을 통합한 ICC(Integrated Chassis Control) 시스템을 설계하여, 표준화된 6개 시나리오(A1, A3, A4, A7, B1, D1)에서 컨트롤러 off 대비 정량적 성능 개선을 달성하는 것이다.
전 영역에서 PID 기반 게인 스케줄링(gain scheduling) 제어를 선택했다. 본 과제의 plant(14DOF)는 비선형성이 강하고(타이어 saturation, 하중이동, 서스펜션 결합) 시나리오별 동작 영역(저속 정상선회 ~ 고속 회피기동)의 폭이 넓어, 단일 선형 게인의 LQR/state-feedback보다는 속도(vx)에 따라 게인을 스케줄링하는 PID가 구현 복잡도 대비 강건성이 높다고 판단했다. 또한 ASSIGNMENT.md가 명시적으로 금지하는 "시나리오 ID 분기"를 PID + gain scheduling 구조로는 자연스럽게 회피할 수 있다는 점도 고려했다.
각 제어기 한 줄 요약:
ctrl_lateral: Yaw-rate PID(속도 적응형 게인 스케줄링) 기반 AFS + slip angle 기반 β-limiter ESC
ctrl_longitudinal: 속도추종 PI + bang-bang(이진 전환) ABS
ctrl_vertical: (미구현 — 5.2절 참조)
ctrl_coordinator: 종방향 60:40(전:후) 균등 분배 + ESC yaw moment의 좌우 차동 brake 분배
---
2. 수학적 모델링
2.1 사용한 plant 단순화
제어기 설계(게인 도출 및 구조 결정)는 선형 2-DOF bicycle model을 기준으로 수행했다. 검증은 과제에서 요구하는 14DOF plant 위에서 수행한다. Bicycle model을 설계 기준으로 택한 이유는, 본 제어기가 다루는 핵심 동역학(yaw rate 추종, 횡슬립 제한)이 본질적으로 2차 횡방향 동역학이며, 14DOF의 롤/피치/언스프렁 동역학은 ctrl_lateral/ctrl_longitudinal의 설계 대상이 아니기 때문이다 (ctrl_vertical만 직접 다룬다).
2.2 State-space 표현
$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$
$$x = [v_y,\ r]^T, \quad u = \delta$$
$$\dot{v}_y = -\frac{C_f+C_r}{mV_x}v_y + \left(\frac{l_rC_r-l_fC_f}{mV_x}-V_x\right)r + \frac{C_f}{m}\delta$$
$$\dot{r} = \frac{l_rC_r-l_fC_f}{I_zV_x}v_y - \frac{l_f^2C_f+l_r^2C_r}{I_zV_x}r + \frac{l_fC_f}{I_z}\delta$$
본 과제의 차량 파라미터(VEH.mass=1500 kg, VEH.Iz=2500 kg·m², VEH.lf=1.2 m, VEH.lr=1.4 m, VEH.Cf=80000 N/rad, VEH.Cr=85000 N/rad)를 대입하고, 게인 스케줄링 기준속도인 $V_x=15$ m/s(ctrl_lateral.m의 `v\\\_ref`)를 대표 동작점으로 사용하면:
$$A = \begin{bmatrix} -7.333 & -13.978 \ 0.613 & -7.515 \end{bmatrix}, \qquad B = \begin{bmatrix} 53.333 \ 38.400 \end{bmatrix}$$
이 동작점에서의 고유값은
$$\lambda_{1,2} = -7.424 \pm 2.927j$$
두 고유값 모두 실수부가 음수이므로 $V_x=15$ m/s에서 개루프 bicycle model은 안정하다. 이는 일반 승용차 파라미터 범위에서 예상되는 결과이며, 본 설계의 PID 제어기가 plant를 안정화하는 역할이 아니라 응답 속도(rise time, settling time)와 오버슈트를 능동적으로 개선하는 역할을 한다는 것을 의미한다 — 실제로 Table 4.1에서 A3의 yawRateRiseTime이 0.247s→0.057s, yawRateSettling이 1.462s→0.783s로 크게 단축된 결과가 이와 일치한다.
속도 의존성도 확인할 수 있다 — $V_x$가 작아질수록 $A$ 행렬의 $(1,1)$, $(2,2)$ 대각항이 $1/V_x$에 비례해 커지므로(감쇠가 강해지는 방향) 저속에서는 응답이 과도하게 느려지거나 둔감해질 수 있고, $V_x$가 커지면 반대로 감쇠가 약해져 진동 경향이 커진다. 이것이 바로 ctrl_lateral.m에서 $V_x$에 비례한 게인 스케줄링(`afsSched`, `escSched`)을 도입한 이론적 근거이다 — 고정 게인은 전 속도 영역에서 균일한 응답특성을 보장하지 못한다.
2.3 가정 + 한계
일정 종속도: 횡방향 제어기 설계 시 vx를 매 스텝 동결된 파라미터로 취급(실제로는 ctrl_lateral 내부에서 vx 의존 게인 스케줄링으로 부분 보완).
선형 타이어: 설계 기준 모델은 코너링 스티프니스(Cf, Cr) 기반 선형 타이어 — 검증 plant(14DOF)는 Magic Formula 비선형 타이어를 사용하므로, 설계-검증 plant 간 모델 격차가 존재함 (특히 A7 brake-in-turn처럼 슬립이 큰 영역에서).
종/횡 방향 결합 무시: 설계 단계에서 종방향(longitudinal)과 횡방향(lateral) 동역학을 독립적으로 설계했음. 실제로는 하중이동(`dFz\\\_lon`, `dFz\\\_lat`)을 통해 결합되어 있으며, 이는 5.2절에서 한계로 논의함.
---
3. 제어기 설계
3.1 ctrl_lateral — AFS + ESC
설계 목표:
yaw rate 추종
|slip angle| > 1.0° 시 ESC(yaw moment) 개입
속도 의존 게인 스케줄링
선택 기법: PID (yaw-rate 오차 기반) + 속도 비례 게인 스케줄링. 미분항은 노이즈/오버슈트에 매우 민감해 별도의 derivative limiting을 추가했다.
Gain 계산 과정:
초기 게인은 sim_params.m 제공 기본값(Kp=1.0, Ki=0.1, Kd=0.05)에서 출발해 시뮬레이션 반복 튜닝(simulation iteration)으로 조정했다. 설계 과정에서 발견한 핵심 이슈는 D항(미분)이 A4(저속 정상선회, vx=5m/s)에서 노이즈를 증폭시켜 차량 속도가 5→1.25 m/s까지 붕괴하는 발산 현상이었다 — 미분 게인을 1/10로 축소하고 `dErr`를 ±2.0 rad/s로 클램프하는 derivative limiting을 도입해 해결했다.
속도 적응(Gain Scheduling):
$$\text{afsSched} = \mathrm{clip}!\left(\frac{v_x}{v_{ref}},\ 0.4,\ 1.5\right), \quad v_{ref}=15\ \mathrm{m/s}$$
AFS 출력 전체에 이 스케줄을 곱해, 저속에서는 게인을 줄이고(과민 반응 방지) 고속에서는 게인을 키운다(반응성 확보).
ESC β-limiter: |slip angle|이 임계값(BETA_TH = 1.0°)을 넘으면, 초과분에 비례한 반대 방향 yaw moment를 인가한다.
$$M_z = -K_\beta \cdot \mathrm{escSched} \cdot \mathrm{sign}(\beta) \cdot (|\beta|-\beta_{th})$$
ESC도 별도의 속도 스케줄(`escSched`, 0.6~2.0배)을 적용했다.
최종 게인 + 정당화:
```matlab
CTRL.LAT.Kp     = 1.0       % sim\\\_params.m 기본값 유지 (시뮬레이션 반복 결과 기본값이 충분히 안정적)
CTRL.LAT.Ki     = 0.1
CTRL.LAT.Kd     = 0.05      % 적용 시 0.1배로 추가 축소 (사실상 effective Kd = 0.005)
CTRL.LAT.intMax = 5.0       % \\\[rad] anti-windup 한계 (sim\\\_params.m 기본값)

% ctrl\\\_lateral.m 내부 튜닝 파라미터
v\\\_ref   = 15.0;             % \\\[m/s] 게인 스케줄링 기준 속도
maxAFS  = deg2rad(4.0);     % \\\[rad] AFS 최대 보조조향각
BETA\\\_TH = deg2rad(1.0);     % \\\[rad] ESC 개입 임계 슬립각
K\\\_BETA  = 12000;            % \\\[Nm/rad] ESC 게인
M\\\_MAX   = 5000;             % \\\[Nm] ESC yaw moment 한계
```
maxAFS 트레이드오프 실험: maxAFS를 4.0°→2.0°로 낮추면 lateralDevMax는 개선(1.8900→1.8311, 3.1%)되지만 LTR_max는 악화(0.7592→0.8397)되는 트레이드오프가 발생했다. grade.m 총점으로 비교한 결과 4.0°(57.14/70)가 2.0°(56.20/70)보다 우수해, 최종적으로 maxAFS=4.0°를 채택했다.
3.2 ctrl_longitudinal — 속도 + ABS
설계 목표: 속도추종 PI + ABS(|κ|>0.12 시 brake torque 감소) + 저크 제한.
선택 기법: 속도추종은 PI, ABS는 bang-bang(이진 전환) 제어.
설계 히스토리 (v5 → v6): 최초 설계(v5)는 ABS를 비례 제어(`Fx\\\_abs = K\\\_abs × excess`)로 구현했으나, 실측 결과 후륜 슬립비가 -0.005에서 -0.64까지 폭주하는 양의 피드백(brake를 풀수록 슬립이 더 커지는 역설적 현상)이 발생했다. 원인은 1kHz 고정 스텝 대비 슬립 변화 속도가 너무 빨라 부드러운 비례 반응이 추종하지 못했거나, 타이어가 peak μ를 넘어 falling-friction 영역에 진입했기 때문으로 판단된다. ASSIGNMENT.md 힌트의 bang-bang 방식(`brakeRatio`를 0.65/1.0 이진 전환)으로 전환한 뒤 안정화되었으며, B1 stoppingDistance가 72.3m→69.4m, absSlipRMS가 0.73→0.09로 개선되었다.
최종 게인 + 정당화:
```matlab
CTRL.LON.Kp     = 0.5       % sim\\\_params.m 기본값
CTRL.LON.Ki     = 0.05
CTRL.LON.intMax = 2000      % \\\[Nm]

% ctrl\\\_longitudinal.m 내부 튜닝 파라미터
massApprox   = 1600;        % \\\[kg] (VEH.mass=1500 대비 안전마진 포함 근사)
kappaTarget  = 0.12;        % ABS 목표 슬립비 (ASSIGNMENT 명세값)
decelGate    = -0.5;        % \\\[m/s^2] 강제제동 판정 기준
largeErrGate = 2.0;         % \\\[m/s] 강제제동 판정 보완 기준
brakeRatio   = 0.65 또는 1.0  % bang-bang 전환값
```
3.3 ctrl_vertical — CDC
미구현(starter stub 상태). 채점 매트릭스(A1/A3/A4/A7/B1/D1) 직접 영향 없음(C1/C2 가산점 항목)으로 확인되어, 시간 제약상 본 제출에서는 우선순위를 낮췄다 (5.3절 참조).
3.4 ctrl_coordinator — Actuator Allocation
종방향 분배(60:40):
$$T_{total} = |F_{x,total}| \cdot r_w, \quad T_{F,each}=\frac{0.6,T_{total}}{2},\quad T_{R,each}=\frac{0.4,T_{total}}{2}$$
ESC yaw moment 차동 분배: plant 모델의 ESC yaw moment 복원식 $M_{z,axle} = \Delta T / r_w \cdot t_{half}$ 을 역산하여
$$\Delta T = \frac{M_{z,axle}\cdot r_w}{t_{half}}$$
양의 $M_z$(CCW)에 대해 좌측(FL, RL) brake를 증가, 우측(FR, RR)을 감소시키는 방향으로 절반씩 분배했다.
brakeRatio(ABS) 처리상의 구조적 제약: runner(`run\\\_icc\\\_scenario.m`, 수정 불가)가 `brake\\\_total = brk\\\_scenario + brakeESC`로 고정 합산하기 때문에, coordinator는 시나리오가 강제하는 `brk\\\_scenario`의 실제값에 접근할 수 없다. 이로 인해 ABS 해제량은 차량 질량×`LIM.MAX\\\_AX` 기반의 추정 최대 제동토크를 기준으로 근사했다 — 이는 실제 시나리오 강제 brake 값과 차이가 있을 수 있는 설계상의 근사이며, 5.2절에서 한계로 논의한다.
```matlab
rw        = 0.31;   % \\\[m] 타이어 유효 반경
frontBias = 0.6;
rearBias  = 0.4;
```
---
4. 시뮬레이션 결과
4.1 전체 시나리오 benchmark — 베이스라인(OFF) vs 본인 설계(ON)
(`run\\\_icc\\\_scenario.m` 14DOF, 본인 PC 실측값. A1/D1은 ISO 3888-1 DLC, A4는 ISO 4138 정상선회, B1은 ISO 21994 직선제동 표준에 기반한 시나리오. 단위는 KPI별 상이 — sideSlip/LTR/lateralDev 등은 무차원 또는 [°]/[m], stoppingDistance는 [m])
시나리오	KPI	OFF	ON	Δ
A1 DLC@80	sideSlipMax	3.0154	2.6600	-11.8%
A1	LTR_max	0.8635	0.7592	-12.1%
A1	lateralDevMax	1.8270	1.8900	+3.4% (악화)
A3 step steer	yawRateOvershoot	2.6997	2.6173	-3.1%
A3	yawRateRiseTime	0.2470	0.0570	-76.9%
A3	yawRateSettling	1.4620	0.7830	-46.4%
A3	sideSlipMax	1.1138	0.9042	-18.8%
A4 정상선회	understeerGradient	0.0007	0.0030	(목표 0.0030 정확히 일치)
A4	sideSlipMax	1.1839	1.1832	-0.1%
A7 brake-in-turn	sideSlipMax	30.4776	1.9330	-93.7%
A7	LTR_max	0.6808	0.3230	-52.6%
A7	yawDevDuringBrake	117.9057	55.5122	-52.9%
B1 직선제동	stoppingDistance	72.2992	69.4445	-3.9%
B1	absSlipRMS	0.7295	0.0902	-87.6%
B1	jerkMax	1942.8793	3012.7552	+55.1% (악화)
D1 통합	sideSlipMax	4.9057	3.2505	-33.7%
D1	LTR_max	0.8635	0.7592	-12.1%
D1	lateralDevMax	1.8270	1.8900	+3.4% (악화)
(grade.m 점수: Quantitative 57.14/70 (81.6%). 상세 시나리오별 채점표는 부록 C 참조.)
> \\\*\\\*주\\\*\\\*: A1/D1의 `yawRateOvershoot`는 off/on 모두 비현실적으로 큰 값(10^6 단위)이 산출되는데, 이는 KPI 정의가 $(peak-r\\\_{ss})/r\\\_{ss}$ 형태의 정규화를 사용하는 반면 A1/D1(DLC형 시나리오)은 조향이 좌우로 반전되어 \\\*\\\*정상상태 yaw rate $r\\\_{ss}$가 0에 가까워\\\*\\\* 분모가 거의 0이 되는 구조적 한계이다. 트러블슈팅 문서(Q6)에서도 "A3 step steer에서만 의미 있는 KPI"로 명시되어 있어, 본 보고서에서는 A1/D1의 yawRateOvershoot는 분석 대상에서 제외했다.
4.2 핵심 plot — A1 DLC
![A1 trajectory comparison](figures/a1_trajectory.png)
Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.
![A1 yaw rate](figures/a1_yawrate.png)
Figure 4.2 — A1 yaw rate 응답: off(controller off) vs on(본인 설계).
Figure 4.1에서 off/on 두 경로 모두 reference path(검은 점선)를 전반적인 형태로는 따라가지만, 차선변경 구간(x≈20~60m)에서 reference의 횡변위 폭보다 좁게 움직이는 경향이 관찰된다 — 이는 lateralDevMax가 off/on 모두 목표(0.7m)를 크게 초과하는 것과 일치하는 정성적 근거이나, 이 그림의 스케일(y축 ±50m)로는 두 경로의 미세한 차이를 판별하기 어려워 해당 구간만 별도로 확대한 추가 분석이 필요하다.
Figure 4.2에서는 t=0~3s 구간의 1차/2차 차선변경 피크는 off/on이 유사하지만, t=3~4s 구간에서 off(빨간 점선)는 -8°/s 근처까지 추가 진동(2차 오버슈트)이 발생하는 반면 on(파란 실선)은 그 진동 없이 빠르게 0으로 정착한다. 이는 ESC/AFS가 차선변경 종료 후 잔류 yaw 진동을 효과적으로 감쇠시키고 있음을 보여준다. 다만 이 개선이 lateralDevMax 자체의 악화와 동시에 나타난다는 점은, "yaw rate 진동 억제"와 "경로추종 정확도"가 본 설계에서 서로 다른 메커니즘으로 결정되고 있음을 시사한다 (5.2절 가설 참조).
4.3 한 시나리오 deep dive — A7 (brake-in-turn)
A7은 본 설계에서 가장 큰 개선을 보인 시나리오이다.
베이스라인(off) sideSlipMax: 30.48° — 스핀아웃에 해당하는 수준 (트러블슈팅 Q8: "정상, 이게 ESC가 풀어야 할 문제")
본인 설계(on) sideSlipMax: 1.93° — 목표 기준(5°) 대비 충분한 마진 확보
LTR_max: 0.68→0.32 (52.6% 개선), yawDevDuringBrake: 117.9→55.5 (52.9% 개선)
핵심 요인은 ESC β-limiter가 코너링 중 제동으로 인한 슬립각 급증 구간에서 yaw moment를 인가해, 후륜 슬립으로 인한 오버스티어 경향(스핀아웃)을 억제한 것으로 판단된다.
![A7 side slip](figures/a7_sideslip.png)
Figure 4.3 — A7 Brake-in-Turn, side slip angle 시계열 (off vs on).
Figure 4.3은 두 경로가 t=0~3s(제동 인가 전)까지는 거의 동일하게 거동하지만, t≈3s(제동 시작 시점) 이후 off(빨간 점선)는 슬립각이 발산성으로 증가해 t=6s 시점 -30°를 넘어서는 반면, on(파란 실선)은 -2° 근처에서 거의 변화 없이 유지됨을 보여준다. 이 시점이 바로 ESC β-limiter의 개입 임계(BETA_TH=1.0°)를 초과하기 시작하는 구간과 일치하며, ESC가 제동-코너링 결합 상황에서 발산을 사전에 차단하고 있음을 시각적으로 확인할 수 있다.
---
5. 분석 + 한계
5.1 가장 성공적이었던 시나리오
A7 brake-in-turn이 가장 큰 개선을 보였다 (sideSlipMax 93.7% 개선). off 상태에서 이미 명백한 스핀아웃(30.48°)이 발생하는 시나리오였기 때문에, ESC β-limiter라는 단일 메커니즘이 명확한 물리적 역할(슬립각 제한)을 통해 직접적인 효과를 낼 수 있었던 것으로 분석된다. 즉 "문제가 명확하고 단일 메커니즘으로 해결 가능한" 시나리오일수록 PID 기반 설계가 효과적이었다.
5.2 가장 부족했던 시나리오
A1/D1의 lateralDevMax가 가장 부족했다 (off 대비 오히려 3.4% 악화, 목표 0.7~1.0 대비 약 2~2.7배 초과). 두 가지 가설을 검토했다:
가설 1 (AFS-Stanley 폐루프 간섭): A1/D1의 driver model은 `path\\\_follow\\\_stanley`로, 매 스텝 실제 차량 pose를 보고 조향각을 재계산하는 폐루프 추종기이다. AFS가 plant에 추가 조향을 더하면 실제 차량 거동이 Stanley의 의도와 달라지고, Stanley가 다음 스텝에 이를 다시 보정하려 들면서 두 폐루프(driver의 경로추종 루프와 AFS의 yaw-rate 추종 루프)가 서로 간섭할 수 있다. maxAFS를 낮추면 lateralDevMax가 개선되는 경향(4.0°→2.0°, 1.8900→1.8311)은 이 가설과 일치하나, 동시에 LTR_max가 악화되는 트레이드오프가 발생해 maxAFS 단순 조정만으로는 해결되지 않았다.
가설 2 (yaw rate reference의 비물리적 영역): ESC/AFS 게인을 4배 가까이 강화해도(BETA_TH 1.8°→1.0°, 발동빈도 6.2%→17.7%) LTR_max 변화는 0.13%에 불과했다 — 이는 ESC(yaw moment) 경로가 LTR_max(좌우 타이어 하중 불균형)에 거의 영향을 주지 못함을 시사한다. Plant 모델 분석 결과 LTR은 $a_y \approx v_x \cdot r$로 결정되는 하중이동에서 직접 발생하므로, yaw moment로 회전을 "보조/억제"하는 간접적 경로보다는 yawRateRef 자체를 마찰원 한계로 saturate하는 것이 더 직접적인 해결책일 가능성이 있으나, 본 제출 시점까지 실험적으로 검증하지 못했다.
위 두 가설 모두 단순 게인 미세조정으로는 해결되지 않음이 실험적으로 확인되어, 구조적 변경(설계 자체의 수정)이 필요한 문제로 결론짓는다.
추가로, B1의 jerkMax가 off(1942.9) 대비 on(3012.8)에서 오히려 악화되었다. ABS를 비례 제어에서 bang-bang으로 전환하면서 absSlipRMS(타이어 슬립 안정성)는 크게 개선됐지만, brakeRatio의 이진 전환(0.65↔1.0) 자체가 차량 가속도에 단속적인 변화를 유발해 jerk가 증가한 것으로 판단된다. 이는 ABS 설계에서 슬립 안정성과 승차감(jerk) 사이의 전형적인 트레이드오프로 해석된다.
5.3 만약 더 시간이 있었다면
yawRateRef를 마찰원($\mu_{max}\cdot g / v_x$) 기준으로 saturate하는 구조적 변경을 ctrl_lateral에 적용해 A1/D1 lateralDevMax/LTR_max 동시 개선을 검증
ctrl_vertical(CDC/skyhook)을 구현해 C1/C2 가산점 확보
bang-bang ABS의 jerk 악화를 완화하기 위해, 이진 전환 폭(0.65~1.0)을 더 세분화한 다단(multi-level) 전환 또는 히스테리시스 적용
ctrl_coordinator의 마찰원 제한(friction circle constraint) 가산점 항목 구현
---
---
부록 A — 사용한 AI 도구
본 프로젝트에서는 Claude와 ChatGPT를 보조 도구로 활용했다. 주된 활용 방식은 단순히 AI가 제시하는 파라미터 값을 그대로 적용하는 것이 아니라, 다음과 같은 절차로 사용했다:
게인(Kp/Ki/Kd, BETA_TH, K_BETA, maxAFS 등) 또는 구조적 파라미터를 직접 상승/하강시키며 시뮬레이션을 반복 실행
그 결과로 도출된 KPI 값의 변화 경향성을 AI에게 제시하고 해석을 요청
AI의 해석(예: "ESC 강도를 키워도 LTR_max가 거의 안 바뀌는 것은 ESC 경로가 LTR과 약하게 결합되어 있다는 신호")을 바탕으로 다음 실험 방향이나 미고려 변수(예: ABS 양의 피드백 발산, jerk-force saturation 단위 불일치, LTR_max의 실제 물리적 결정 요인)를 탐색
보고서 작성 단계에서는 실제 실행 결과(KPI 수치, plot)를 AI에게 제공하고, 이를 분석적으로 서술하는 초안 작성에 활용 — 최종 해석과 결론은 본인이 검토 후 확정
즉 AI는 파라미터를 대신 결정해주는 도구가 아니라, 본인이 직접 변경한 값에 대한 결과를 해석하고, 다음에 시도해볼 방향이나 미처 고려하지 못했던 가설(예: AFS-Stanley 폐루프 간섭, plant 내 wheelSlip 분모 클램프로 인한 저속 노이즈 증폭)을 함께 점검하는 데 활용했다.
---
부록 B — 본인 sim_params.m 변경사항
```matlab
% CTRL.LAT.\\\*, CTRL.LON.\\\* 는 sim\\\_params.m 기본값을 그대로 사용함
% (Kp/Ki/Kd 자체보다는 ctrl\\\_lateral.m/ctrl\\\_longitudinal.m 내부의
%  게인 스케줄링, anti-windup, derivative limiting 등 구조적 추가가
%  본 설계의 핵심이었음)

CTRL.LAT.Kp     = 1.0
CTRL.LAT.Ki     = 0.1
CTRL.LAT.Kd     = 0.05
CTRL.LAT.intMax = 5.0

CTRL.LON.Kp     = 0.5
CTRL.LON.Ki     = 0.05
CTRL.LON.intMax = 2000
```
---
부록 C — grade.m 채점 상세 (Quantitative 57.14/70)
SID	KPI	Value	Target	Score / Max
A3	yawRateOvershoot	2.6173	10.0000	4.00 / 4
A3	yawRateRiseTime	0.0570	0.3000	4.00 / 4
A3	yawRateSettling	0.7830	0.8000	4.00 / 4
A1	sideSlipMax	2.6600	3.0000	6.00 / 6
A1	LTR_max	0.7592	0.6000	3.67 / 5
A1	lateralDevMax	1.8900	0.7000	0.00 / 4
A4	understeerGradient	0.0030	0.0030	5.00 / 5
A4	sideSlipMax	1.1832	2.0000	5.00 / 5
A7	sideSlipMax	1.9330	5.0000	8.00 / 8
A7	LTR_max	0.3230	0.7000	7.00 / 7
B1	stoppingDistance	69.4445	40.0000	0.00 / 5 *
B1	absSlipRMS	0.0902	0.1000	5.00 / 5
D1	sideSlipMax	3.2505	4.0000	4.00 / 4
D1	LTR_max	0.7592	0.6000	1.47 / 2
D1	lateralDevMax	1.8900	1.0000	0.00 / 2
합계				57.14 / 70
* B1 stoppingDistance: 강의자 공지로 만점 기준이 40m→66.5m로 완화됨(grade.m 파일 자체는 수정 금지 대상이라 로컬 결과에는 미반영). 66.5m 기준 적용 시 약 9.5/10 추정.