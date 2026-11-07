# just drag
import numpy as np
from rocketpy.mathutils.vector_matrix import Vector
from rocketpy.mathutils.vector_matrix import Matrix

def quaternion_to_zero_rotation(q):
    #takes in rocket quaternion and returns the rotation matrix to rotate the rocket to a zero rotation state
    e0, e1, e2, e3 = q

    # Normalize quaternion
    q_norm = np.linalg.norm(q)
    e0, e1, e2, e3 = np.array(q) / q_norm

    # Body-to-world rotation matrix
    R = Matrix([
        [1 - 2*(e2**2 + e3**2), 2*(e1*e2 - e0*e3),     2*(e1*e3 + e0*e2)],
        [2*(e1*e2 + e0*e3),     1 - 2*(e1**2 + e3**2), 2*(e2*e3 - e0*e1)],
        [2*(e1*e3 - e0*e2),     2*(e2*e3 + e0*e1),     1 - 2*(e1**2 + e2**2)]
    ])

    # Inverse rotation
    return R

vector1 = Vector([1, -2, 3])
vector2 = Vector([-4, 5, 6])
vector3 = vector1 + vector2
gravity = Vector([0, 0, -9.81])

print(f"Vector 1: {vector1}")
print(f"Vector 2: {vector2}")
print(f"Vector 3: {vector3}")

print(f"Vector 1 magnitude: {abs(vector1)}")
print(f"Vector 2 direction: {vector2/abs(vector2)}")
print(f"Vector 1 cross 2: {vector1.cross(vector2)}")
print(f"Vector 1 dot 2: {vector1.dot(vector2)}")

matrix1 = Matrix([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
matrix2 = Matrix([[7, 8, 9], [10, 11, 12], [13, 14, 15]])
matrix3 = matrix1 + matrix2

print(f"Matrix 1 transpose: {matrix1.transpose}")

e = [0,0,-1,0] #0,0,0 quaternion
R_ub = quaternion_to_zero_rotation(e)

print(R_ub.transpose @ gravity)
#that was the test for the vector and matrix classes, now onto the actual simulation code



parachute_initial_height = 500 #parachute deploys fully 50 meters later, and initially goes 20 m/s downward
parachute_radius = (244/2)/100
parachute_drag_coefficient = 2.2

from rocketpy import Environment, Flight, Rocket, SolidMotor
env = Environment(latitude=32.990254, longitude=-103.66157, elevation=899) #accurate btw, going off Saragossa, Texas

import datetime
today = datetime.date.today() + datetime.timedelta(days=2)
env.set_date(
    (today.year, today.month, today.day, 12)
)
env.set_atmospheric_model(type="Forecast", file="GFS")
#env.info()

dead_motor = SolidMotor(
    thrust_source=5000,
    dry_mass=1,
    dry_inertia=(0.125, 0.125, 0.002),
    nozzle_radius=33 / 1000,
    grain_number=5,
    grain_density=1815,
    grain_outer_radius=33 / 1000,
    grain_initial_inner_radius=15 / 1000,
    grain_initial_height=120 / 1000,
    grain_separation=5 / 1000,
    grains_center_of_mass_position=0.397,
    center_of_dry_mass_position=0.317,
    nozzle_position=0,
    burn_time=1,
    throat_radius=11 / 1000,
    coordinate_system_orientation="nozzle_to_combustion_chamber",
)

#dead_motor.info()


payload = Rocket(
    radius= 13.1/1000,
    mass=36,
    inertia=(9, 9, 0.06),
    power_off_drag=.5,
    power_on_drag=.5,
    center_of_mass_without_motor=0,
    coordinate_system_orientation="tail_to_nose",
)

#payload.add_motor(dead_motor,-1.2)

rail_buttons = payload.set_rail_buttons(
    upper_button_position=0.0818,
    lower_button_position=-0.618,
    angular_position=45,
)

nose_cone = payload.add_nose(length=0.55829, kind="vonKarman", position=1.278)

fin_set = payload.add_trapezoidal_fins(
    n=4,
    root_chord=0.120,
    tip_chord=0.060,
    span=0.110,
    position=-1.04956,
    cant_angle=0.5,
    airfoil=None,
)

tail = payload.add_tail(
    top_radius=0.0635, bottom_radius=0.0435, length=0.060, position=-1.194656
)

#payload.info()

def main_trigger(p, h, y):
    # h = height above ground level. Deploy main once descending through 500 m AGL.
    return True if y[5] < 0  else False

Main = payload.add_parachute(
    "Main",
    cd_s = parachute_drag_coefficient * parachute_radius**2 * 3.14159,
    trigger=parachute_initial_height-50,
    sampling_rate=105, #all following are really non-essintial
    lag=0,
    noise=(0, 8.3, 0.5),
)

initial_solution = [
    0,                              # t (s)
    0, 0, env.elevation + parachute_initial_height,  # x, y, z (m)
    0, 0, -20,                        # vx, vy, vz (m/s)
    0, 0, -1, 0,                     # e0, e1, e2, e3
    0, 0, 0,                        # w1, w2, w3 (rad/s)
]

test_flight = Flight(
    rocket=payload, environment=env, rail_length=5.2, inclination=89.9, heading=0,
)

flight = Flight(
    rocket=payload,
    environment=env,
    rail_length=5.2,          # required arg, unused once initial_solution is set
    initial_solution=initial_solution,
    terminate_on_apogee=False,
)
print(f"Drop altitude: {parachute_initial_height} m AGL")
print(f"Impact time: {flight.t_final:.2f} s")
print(f"Impact velocity: {flight.impact_velocity:.2f} m/s")
print(f"Max speed during descent: {flight.max_speed:.2f} m/s")
flight.all_info()
