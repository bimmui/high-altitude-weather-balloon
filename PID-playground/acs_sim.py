from pid import PIDController
from ambiance import Atmosphere
import pandas as pd

# constants
ACS_ACTIVATE_ALTITUDE = 30000  # meters
ZERO_VERTICAL_VELOCITY = 0
BALLOON_MASS = 5
EARTH_GRAV_ACCEL = 9.81

# state variables
position_pid_active = False
vert_velocity_pid_active = False
descending = False

# dynamical model state variables
curr_alt = None
curr_vert_velo = None
balloon_radius = None



balloon_volume = lambda r_0, rho_atmo, T, n: r_0 * (1/rho_atmo)**(1/3)
dvdt = lambda V_b, rho_atmo, rho_b, g, m, C_D, S, v: ((V_b * (rho_atmo - rho_b) * g) - (mg) + (0.5 * C_D * rho_atmo * S * v**2) / m)


hab_data = pd.read_csv("data/PID-playground/data/24057799_alt-ascent-descent.csv")
simulation_df = pd.DataFrame(
    columns=[
        "Time",
        "Altitude",
        "Vertical Velocity",
        "Control Output",
        "Valve State",
        "PID Active",
    ]
)

# getting relative time to start simulation at t = 0
start_time = datetime.strptime(hab_data["Time"].iloc[0], "%Y-%m-%d %H:%M:%S")
hab_data["Relative Time"] = hab_data["Time"].apply(
    lambda ts: (datetime.strptime(ts, "%Y-%m-%d %H:%M:%S") - start_time).total_seconds()
)

position_pid = PIDController(
    Kp=1.0,
    Ki=0.01,
    Kd=0.1,
    tau=0.02,
    limMin=0.0,
    limMax=100.0,
    limMinInt=-10.0,
    limMaxInt=10.0,
    T=1.0,
)

vert_velocity_pid = PIDController(
    Kp=1.0,
    Ki=0.01,
    Kd=0.1,
    tau=0.02,
    limMin=0.0,
    limMax=100.0,
    limMinInt=-10.0,
    limMaxInt=10.0,
    T=1.0,
)

for index, row in hab_data.iterrows():
    curr_altitude = row["Altitude"]
    curr_vert_velo = row["Ascent Rate (Reported, m/s)"]
    
    # case where the hab is still below the activation altitude for the control system
    if (curr_altitude < ACS_ACTIVATE_ALTITUDE) and descending is False:
        new_row = {
        "Time": row["Relative Time"],
        "Altitude": curr_altitude,
        "Vertical Velocity": curr_vert_velo,
        "Control Output": 0,
        "PID Active": "None"
        }
    else:
        break

vert_velocity_pid_active = True
while curr_alt > hab_data['Altitude'].iloc[0]:
    control_output = vert_velocity_pid.update(0, curr_altitude)
    
    
    
# case where the hab met activation altitude, is slowly decreasing vertical velocity
if position_pid_active and not vert_velocity_pid_active:
    control_output = position_pid_active.update(MAX_ALTITUDE, curr_altitude)
new_row = {
    "Time": row["Relative Time"],
    "Altitude": curr_altitude,
    "Vertical Velocity": curr_vert_velo,
    "Control Output": control_output,
}
new_row["PID Active"] = (
"position_pid" if position_pid_active else 
"vert_velocity_pid" if vert_velocity_pid_active)


for _ in range(50):  # Simulate 10 iterations
    control_output = pid.update(setpoint, measurement)
    print(f"Control Output: {control_output},     Measurement: {measurement}")

    # Simulate system response (for example, decrease altitude slightly)
    measurement += (setpoint - measurement) * 0.1 - control_output * 0.01



runge_katta_velocity_calc(t, v, altitude):
    atmopheric_model = Atmosphere(altitude)
    curr_air_pressure = atmopheric_model.pressure
    k1 = dvdt()
    