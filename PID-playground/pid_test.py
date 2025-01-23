from pid import PIDController

# init the PID controller
pid = PIDController(
    Kp=1.0,
    Ki=0.01,
    Kd=0.1,
    tau=0.02,
    limMin=0.0,
    limMax=100.0,
    limMinInt=-10.0,
    limMaxInt=10.0,
    T=1.0
)

# example simulation loop
setpoint = 100.0  # desired altitude (100 meters)
measurement = 90.0  # curr altitude (90 meters)

for _ in range(50):  # simulate 50 iterations
    control_output = pid.update(setpoint, measurement)
    print(f"Control Output: {control_output},     Measurement: {measurement}")

    # simulate system response (for example, decrease altitude slightly)
    measurement += (setpoint - measurement) * 0.1 - control_output * 0.01