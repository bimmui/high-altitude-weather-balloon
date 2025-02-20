from pid import PIDController
from ambiance import Atmosphere
import pandas as pd
import numpy as np


# constants
ACS_ACTIVATE_ALTITUDE = 30000  # meters
ZERO_VERTICAL_VELOCITY = 0
g = 9.81 # m/s^2
c = 0 # this needs to be changed but idk what its supposed to be
Psl0 = 101325 # pascals
Psl2 = 2481 # pascals
a_valve = 0.5 # meters, filler so change this 
latex_elastic_stress = 0.1 # pascals, filler so change this 
rho_helium = 7 # kg / meters^3, filler so change this 


class Balloon:
    """
    Initializes the Balloon class with initial state parameters and
    contains the dynamic state variables updated through the simulation.

    :param init_alt: Initial altitude in meters
    :param init_velo: Initial velocity in meters per sec
    :param init_r: Initial radius of the balloon in meters
    :param init_r: Mass of the balloon payload
    """
    def __init__(self, init_alt, init_velo, init_r, final_r, m):
        self.curr_velo = init_velo
        self.init_r = init_r
        self.final_r = final_r
        self.mass = m
        self.curr_pressure = 0
        self.curr_volume = 0
        self.curr_radius = 0
        self.curr_density = 0
        self.valve_volumetric_flow_rate = 0
        self.cross_section_area = 0 


    
    def update(self, altitude, delta_t):
        # TODO: update the curr_velocity of the balloon as well, do it in here or in another function
        atmosphere = Atmosphere(altitude)

        # a little bit standalone compared to the other calcs that are dependent on each other
        self.cross_section_area = self.calc_balloon_cross_section_area(self.curr_radius)

        self.curr_pressure = self.calc_balloon_pressure(atmosphere.pressure, self.curr_radius)
        self.valve_volumetric_flow_rate = self.calc_balloon_dV(a_valve, atmosphere.pressure, rho_helium)
        self.curr_volume = self.calc_balloon_volume(rho_helium, delta_t)
        self.curr_density = self.calc_balloon_density(rho_helium, delta_t)
        self.cross_section_area = self.calc_balloon_cross_section_area(self.curr_radius)

        # updating balloon state values for next iteration
        self.curr_radius = np.cbrt((3 * self.curr_volume) / (4 * np.pi))




    
    def calc_balloon_cross_section_area(self):
        """
        Calculate the cross-sectional area of the balloon.
        NOTE: drag force requires cross sectional area of the balloon which changes with its radius r
        """
        return np.pi * self.curr_radius**2

    def calc_balloon_pressure(self, pressure_atmo):
        """Calculate the balloon's internal pressure"""
        return pressure_atmo + ((2 * latex_elastic_stress) / self.curr_radius)

    def calc_balloon_dV(self, A_valve, pressure_atmo, rho_helium):
        """Calculate the differential volume change through the valve"""
        return A_valve * np.sqrt(2 * ((self.curr_pressure - pressure_atmo) / rho_helium))

    def calc_balloon_volume(self, pressure_atmo, delta_t):
        """Calculate the balloon's volume"""
        # change that c constant idk what it is
        return ((4/3) * np.pi) * (c + ((self.final_r - self.init_r) / (Psl0 - Psl2)) * pressure_atmo) - (self.valve_volumetric_flow_rate * delta_t)

    def calc_balloon_density(self, rho_helium, delta_t):
        """Calculate the balloon's density"""
        return (((4/3) * np.pi * self.init_r**3 * rho_helium) - (self.curr_pressure * self.valve_volumetric_flow_rate * delta_t)) / self.curr_volume


# state variables
position_pid_active = False
vert_velocity_pid_active = False
descending = False


def force_drag(C_d, rho_atmo, balloon_obj):
    """Calculate the drag force on the balloon"""
    # Assuming multiplication between A_d and ((-1 * v) * abs(v))
    return ((C_d * rho_atmo * balloon_obj.cross_section_area) * ((-1 * balloon_obj.curr_velo) * abs(balloon_obj.curr_velo))) / 2

def force_lift(rho_atmo, balloon_obj):
    """Calculate the lift force on the balloon"""
    return (g * balloon_obj.curr_volume) * (rho_atmo - balloon_obj.curr_density)

def force_gravity(balloon_obj):
    """Calculate the force of gravity on the balloon"""
    return balloon_obj.mass * g

def dvdt(balloon, altitude, delta_t):
    atmosphere = Atmosphere(altitude)
    balloon.update(altitude, delta_t)
    return (force_lift(atmosphere.density, balloon) - force_gravity(balloon) + force_drag(0.5, atmosphere.density, balloon)) / balloon.mass




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
    