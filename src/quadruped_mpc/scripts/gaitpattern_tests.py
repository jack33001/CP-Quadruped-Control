def run_fsm(T_curr, T_gait_start, T_offset, T_stance, T_swing):
    T_cycle = T_stance + T_swing
    # State -2
    if T_curr - T_gait_start < T_offset:
        state = -2
        phase = (T_curr - T_gait_start)/T_offset
    # State -1
    elif T_curr - T_gait_start < T_offset + T_cycle:
        state = -1
        phase = (T_curr - T_gait_start - T_offset)/T_cycle
    else:
        t = ((T_gait_start + T_offset + T_cycle) - T_curr) % T_cycle
        # State 0
        if t < T_stance:
            state = 0
            phase = t/T_stance
        # State 1
        else:
            state = 1
            phase = (t - T_stance)/T_swing
            
    return state, phase

t_curr = 5
t_gait_start = 5
stages = 50
horizon = 5
offset = .5
T_stance = .5
T_swing = .1

# Simulate a real world clock
for i in range(200):
    t = t_curr
    state = []
    phase = []
    # Run the process
    for i in range(stages):
        timestep = horizon/stages
        state_temp, phase_temp = run_fsm(t, t_gait_start, offset, T_stance, T_swing)
        state.append(state_temp)
        phase.append(phase_temp)
        t += timestep
    t_curr += .01
    print(f"Timestamp: {t_curr:.2f} \n\rState: {state} \n\rPhase: {phase}")
    print()