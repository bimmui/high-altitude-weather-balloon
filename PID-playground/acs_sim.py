from pid import PIDController
# from ambiance import Atmosphere
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# # constants
# ACS_ACTIVATE_ALTITUDE = 30000  # meters
# ZERO_VERTICAL_VELOCITY = 0
# BALLOON_MASS = 5
# EARTH_GRAV_ACCEL = 9.81

# # state variables
# position_pid_active = False
# vert_velocity_pid_active = False
# descending = False

# # dynamical model state variables
# curr_alt = None
# curr_vert_velo = None
# balloon_radius = None



# balloon_volume = lambda r_0, rho_atmo, T, n: r_0 * (1/rho_atmo)**(1/3)
# dvdt = lambda V_b, rho_atmo, rho_b, g, m, C_D, S, v: ((V_b * (rho_atmo - rho_b) * g) - (mg) + (0.5 * C_D * rho_atmo * S * v**2) / m)


# hab_data = pd.read_csv("data/PID-playground/data/24057799_alt-ascent-descent.csv")
# simulation_df = pd.DataFrame(
#     columns=[
#         "Time",
#         "Altitude",
#         "Vertical Velocity",
#         "Control Output",
#         "Valve State",
#         "PID Active",
#     ]
# )

# # getting relative time to start simulation at t = 0
# start_time = datetime.strptime(hab_data["Time"].iloc[0], "%Y-%m-%d %H:%M:%S")
# hab_data["Relative Time"] = hab_data["Time"].apply(
#     lambda ts: (datetime.strptime(ts, "%Y-%m-%d %H:%M:%S") - start_time).total_seconds()
# )

# position_pid = PIDController(
#     Kp=1.0,
#     Ki=0.01,
#     Kd=0.1,
#     tau=0.02,
#     limMin=0.0,
#     limMax=100.0,
#     limMinInt=-10.0,
#     limMaxInt=10.0,
#     T=1.0,
# )

# vert_velocity_pid = PIDController(
#     Kp=1.0,
#     Ki=0.01,
#     Kd=0.1,
#     tau=0.02,
#     limMin=0.0,
#     limMax=100.0,
#     limMinInt=-10.0,
#     limMaxInt=10.0,
#     T=1.0,
# )

# for index, row in hab_data.iterrows():
#     curr_altitude = row["Altitude"]
#     curr_vert_velo = row["Ascent Rate (Reported, m/s)"]
    
#     # case where the hab is still below the activation altitude for the control system
#     if (curr_altitude < ACS_ACTIVATE_ALTITUDE) and descending is False:
#         new_row = {
#         "Time": row["Relative Time"],
#         "Altitude": curr_altitude,
#         "Vertical Velocity": curr_vert_velo,
#         "Control Output": 0,
#         "PID Active": "None"
#         }
#     else:
#         break

# vert_velocity_pid_active = True
# while curr_alt > hab_data['Altitude'].iloc[0]:
#     control_output = vert_velocity_pid.update(0, curr_altitude)
    
    
    
# # case where the hab met activation altitude, is slowly decreasing vertical velocity
# if position_pid_active and not vert_velocity_pid_active:
#     control_output = position_pid_active.update(MAX_ALTITUDE, curr_altitude)
# new_row = {
#     "Time": row["Relative Time"],
#     "Altitude": curr_altitude,
#     "Vertical Velocity": curr_vert_velo,
#     "Control Output": control_output,
# }
# new_row["PID Active"] = (
# "position_pid" if position_pid_active else 
# "vert_velocity_pid" if vert_velocity_pid_active)


# for _ in range(50):  # Simulate 10 iterations
#     control_output = pid.update(setpoint, measurement)
#     print(f"Control Output: {control_output},     Measurement: {measurement}")

#     # Simulate system response (for example, decrease altitude slightly)
#     measurement += (setpoint - measurement) * 0.1 - control_output * 0.01



# runge_katta_velocity_calc(t, v, altitude):
#     atmopheric_model = Atmosphere(altitude)
#     curr_air_pressure = atmopheric_model.pressure
#     k1 = dvdt()


# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# # from pid import PIDController
# # from ambiance import Atmosphere

# Constants
g = 9.81  # Gravitational acceleration (m/s^2)
ZERO_VERTICAL_VELOCITY = 0.0  # Target vertical velocity (m/s)
ACS_ACTIVATE_ALTITUDE = 15000  # Altitude at which the ACS activates (m)
MAX_ALTITUDE = 30000  # Maximum altitude (m)

# Balloon Properties
V_b = 10.0  # Balloon volume (m^3)
rho_b = 0.2  # Balloon gas density (kg/m^3)
m = 5.0  # Mass of the payload + balloon (kg)
C_D = 0.5  # Drag coefficient
S = 1.0  # Cross-sectional area (m^2)

def dvdt(V_b, rho_atmo, rho_b, g, m, C_D, S, v):
    drag_force = 0.5 * C_D * rho_atmo * S * v**2
    buoyancy_force = V_b * (rho_atmo - rho_b) * g
    return (buoyancy_force - m * g - drag_force) / m

class Atmosphere:
    def __init__(self, altitude):
        self.altitude = altitude

    @property
    def density(self):
        if self.altitude <= 11000:  # Troposphere
            return 1.225 * (1 - 0.0000225577 * self.altitude)**4.2561
        elif self.altitude <= 20000:  # Lower Stratosphere
            return 0.36391 * np.exp(-0.000157688 * (self.altitude - 11000))
        else:  # Upper Stratosphere
            return 0.08803 * np.exp(-0.0001262 * (self.altitude - 20000))

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

def runge_kutta_velocity(t, v, altitude):
    atmosphere = Atmosphere(altitude)
    rho_atmo = atmosphere.density
    k1 = dvdt(V_b, rho_atmo, rho_b, g, m, C_D, S, v)
    k2 = dvdt(V_b, rho_atmo, rho_b, g, m, C_D, S, v + 0.5 * t * k1)
    k3 = dvdt(V_b, rho_atmo, rho_b, g, m, C_D, S, v + 0.5 * t * k2)
    k4 = dvdt(V_b, rho_atmo, rho_b, g, m, C_D, S, v + t * k3)
    return v + (t / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

# Load HAB data (replace 'your_file.csv' with actual file path)
hab_data = pd.DataFrame({
    "Relative Time": np.arange(0, 3600, 10),  # Dummy time data (0 to 3600 seconds)
    "Altitude": np.linspace(0, 10000, 360),  # Dummy altitude data
    "Ascent Rate (Reported, m/s)": np.linspace(5, 0, 360),  # Dummy ascent rates
})

# Initialize PID controllers
position_pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
vert_velocity_pid = PIDController(kp=0.8, ki=0.2, kd=0.1)

# Initialize simulation variables
curr_alt = hab_data["Altitude"].iloc[0]
curr_vert_velo = hab_data["Ascent Rate (Reported, m/s)"].iloc[0]
descending = False
vert_velocity_pid_active = False  # Initialize the PID active state

# Simulation DataFrame
simulation_df = pd.DataFrame(columns=["Time", "Altitude", "Vertical Velocity", "Control Output", "PID Active"])

time_step = 1  # seconds

for i in range(len(hab_data)):
    curr_time = hab_data["Relative Time"].iloc[i]
    control_output = 0

    if curr_alt < ACS_ACTIVATE_ALTITUDE and not descending:
        control_output = 0  # No PID activation
        vert_velocity_pid_active = False
    else:
        vert_velocity_pid_active = True
        control_output = vert_velocity_pid.update(ZERO_VERTICAL_VELOCITY, curr_vert_velo)

    # Calculate new velocity and altitude
    curr_vert_velo = runge_kutta_velocity(time_step, curr_vert_velo, curr_alt)
    curr_alt += curr_vert_velo * time_step

    # Check if descending state is reached
    if curr_alt >= MAX_ALTITUDE and not descending:
        descending = True

    # Store results
    simulation_df = pd.concat([simulation_df, pd.DataFrame([{
        "Time": curr_time,
        "Altitude": curr_alt,
        "Vertical Velocity": curr_vert_velo,
        "Control Output": control_output,
        "PID Active": "vert_velocity_pid" if vert_velocity_pid_active else "None",
    }])], ignore_index=True)

# Plot results
plt.figure(figsize=(12, 6))
plt.plot(simulation_df["Time"], simulation_df["Altitude"], label="Altitude (m)")
plt.plot(simulation_df["Time"], simulation_df["Vertical Velocity"], label="Vertical Velocity (m/s)")
plt.xlabel("Time (s)")
plt.ylabel("Value")
plt.legend()
plt.title("Weather Balloon Sim")
plt.show()
  