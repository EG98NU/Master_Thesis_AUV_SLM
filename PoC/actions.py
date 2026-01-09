import random
import numpy as np
from typing import List
from state import World, Buoy, Pipeline, Pipe, Marker, MainPipe

# ----------------------
# Utility functions
# ----------------------

def compute_distance(p1: list[float], p2: list[float]) -> float:
    """Compute the Euclidean distance between two points with 1 decimal digit."""
    distance = np.linalg.norm(np.array(p1) - np.array(p2))
    return float(round(distance, 1))


def rand_surroundings(object_position: list[float], distance: float) -> list[float]:
    """Generate random positions around a given object position within a specified distance.
       Each coordinate is rounded to 1 decimal place."""
    angle = random.uniform(0, 2 * np.pi)
    radius = random.uniform(0, distance)
    displacement = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])
    new_position = np.array(object_position) + displacement
    # Convert to plain Python floats with 1 decimal
    return [float(round(coord, 1)) for coord in new_position]


# ----------------------
# Sensor activation
# ----------------------
def activate_receiver(world: World):
    world.sensors.receiver = True
    print("    📡 Receiver activated")
    return True

def activate_camera(world: World):
    world.sensors.camera = True
    print("    📷 Camera activated")
    return True

def activate_sonar(world: World):
    world.sensors.sonar = True
    print("    🔊 Sonar activated")
    return True

def deactivate_receiver(world: World):
    world.sensors.receiver = False
    print("    📡 Receiver deactivated")
    return True

def deactivate_camera(world: World):
    world.sensors.camera = False 
    print("    📷 Camera deactivated")
    return True

def deactivate_sonar(world: World):
    world.sensors.sonar = False
    print("    🔊 Sonar deactivated")
    return True

# ----------------------
# Waypoint & navigation
# ----------------------
def process_waypoint(world: World, external_waypoints: List[List[float]]):
    """
    Process a waypoint for the world.
    If external_waypoints has entries, pop the first one and append it to the world's waypoints.
    """
    if not external_waypoints:
        print("❌ No external waypoints provided")
        return False

    # Take the first waypoint from the external list
    waypoint = external_waypoints.pop(0)
    world.coordinates.waypoints.append(waypoint)
    print(f"    🎯 Waypoint processed: {waypoint}")
    return True


def set_target_waypoint(world: World):
    if world.interesting_points:
        waypoint = world.interesting_points.pop()
        world.coordinates.waypoints.append(waypoint)
        print(f"    🎯 Target waypoint set: {waypoint}")
        return True
    else:
        print("    ⚠️ No interesting points available")
        return False

def reach_waypoint(world: World):
    if world.coordinates.waypoints:
        world.coordinates.current = world.coordinates.waypoints[-1]
        world.coordinates.waypoints.clear()
        print(f"    🏃 Waypoint reached: {world.coordinates.current}")
        return True
    else:
        print("    ⚠️ No waypoints to reach")
        return False

def compute_path(world: World):
    print(f"    🗺️ Path computed")
    return True

def is_there_gate(world: World):
    """If there are at least two yellow buoys, set them as the gate positions."""
    yellow_buoys = [buoy for buoy in world.buoys if buoy.color.lower() == "yellow"]

    if len(yellow_buoys) == 2:
        world.gate.position1 = yellow_buoys[0].position.copy()
        world.gate.position2 = yellow_buoys[1].position.copy()
        print(f"    🚪 Gate set from yellow buoys at {world.gate.position1} and {world.gate.position2}")
    elif len(yellow_buoys) == 1:
        print("    ⚠️ Only one yellow buoy found so far")
    else:
        print("    ⚠️ No yellow buoys found yet")
    return True


def compute_crossing_path(world: World):
    """Compute a path crossing the gate: one waypoint before and one after."""
    pos1 = np.array(world.gate.position1, dtype=float)
    pos2 = np.array(world.gate.position2, dtype=float)

    if not pos1.any() or not pos2.any():
        print("⚠️ Gate positions not set")
        return False

    # Midpoint between buoys
    midpoint = (pos1 + pos2) / 2.0

    # Direction vector of the gate line (pos1 -> pos2)
    gate_vec = pos2 - pos1

    # Perpendicular direction in XY plane (swap x,y and negate one)
    perp_vec = np.array([-gate_vec[1], gate_vec[0], 0.0])

    # Normalize
    norm = np.linalg.norm(perp_vec)
    if norm == 0:
        print("⚠️ Invalid gate positions (same point)")
        return False
    perp_unit = perp_vec / norm

    # Waypoints: 1m before and after
    wp_before = (midpoint - perp_unit).tolist()
    wp_after  = (midpoint + perp_unit).tolist()

    # Append to world waypoints
    world.coordinates.waypoints.clear()  # Clear previous waypoints
    world.coordinates.waypoints.append(wp_before)
    world.coordinates.waypoints.append(wp_after)

    print(f"    🗺️ Crossing path computed:")
    print(f"        Before gate: {wp_before}")
    print(f"        After gate:  {wp_after}")

    return True

def follow_path(world: World):
    if not world.coordinates.waypoints:
        print("    ⚠️ No path to follow")
        return False
    waypoints_reached = world.coordinates.waypoints[-1]
    for waypoint in world.coordinates.waypoints:
        world.coordinates.current = waypoint
    world.coordinates.waypoints.clear()
    print(f"    🏃 Path followed, reached {waypoints_reached}")
    return True


# ----------------------
# Buoy actions
# ----------------------
def generic_survey(world: World, real_world: World):
    """Survey real_world for buoys and pipelines not yet stored in world.
    Adds their positions to world's interesting_points if new."""

    # Helper: check if a position is already stored (within tolerance)
    def is_known(position: List[float], known_points: List[List[float]], tol: float = 1e-6) -> bool:
        for kp in known_points:
            if all(abs(a - b) < tol for a, b in zip(position, kp)):
                return True
        return False

    # --- Check buoys ---
    for buoy in real_world.buoys:
        already_known_buoy = is_known(buoy.position, [b.position for b in world.buoys])
        already_in_interesting = is_known(buoy.position, world.interesting_points)
        if not already_known_buoy and not already_in_interesting:
            world.interesting_points.append(buoy.position)
            print(f"❓ Something found at {buoy.position}")

    # --- Check pipelines (optional, kept commented out) ---
    # for pipeline in real_world.pipelines:
    #     already_known_pipeline = is_known(pipeline.position, [p.position for p in world.pipelines])
    #     already_in_interesting = is_known(pipeline.position, world.interesting_points)
    #     if not already_known_pipeline and not already_in_interesting:
    #         world.interesting_points.append(pipeline.position)
    #         print(f"❓ Something found at {pipeline.position}")

    return True




def gate_buoy_survey(world: World, real_world: World):
    """Search for the closest unknown yellow buoy and store full info."""
    # --- Filter only yellow buoys ---
    yellow_buoys = [b for b in real_world.buoys if b.color.lower() == "yellow"]

    # --- Already known positions ---
    known_positions = [b.position for b in world.buoys]

    # --- If all yellow buoys already discovered ---
    if len([b for b in yellow_buoys if b.position in known_positions]) >= len(yellow_buoys):
        print("⚠️ No more yellow buoys to be found.")
        return False

    current_pos = world.coordinates.current

    # --- Get unknown yellow buoys ---
    unknown_buoys = [b for b in yellow_buoys if b.position not in known_positions]

    # --- Find closest yellow buoy ---
    closest_buoy = min(unknown_buoys, key=lambda b: compute_distance(current_pos, b.position))

    # --- Store full buoy info in world (color + position + image=True) ---
    world.buoys.append(Buoy(color=closest_buoy.color, position=closest_buoy.position, image=True))
    print(f"🟡 New yellow buoy discovered at {closest_buoy.position}")

    # # --- Update robot current position (within radius 5 of buoy) ---
    # angle = random.uniform(0, 2*np.pi)
    # radius = random.uniform(0, 5.0)
    # displacement = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])

    # new_position = np.array(closest_buoy.position) + displacement
    # world.coordinates.current = new_position.tolist()

    world.coordinates.current = rand_surroundings(closest_buoy.position, 5.0)

    print(f"🤖 Robot is now at {world.coordinates.current}")

    return True

def map_buoy_area(world: World, real_world: World):
    """Detect the color of every buoy and perform the corresponding action."""
    last_position = None  # track the last buoy position

    if not world.interesting_points:
        print(f"⚠️ No buoy found")
        return False

    while world.interesting_points:
        # Take the first interesting point
        point = world.interesting_points.pop(0)

        # Move robot to this point
        world.coordinates.current = point
        print(f"🤖 Robot moved to interesting point {point}")

        # Find the corresponding buoy in real_world
        matching_buoy = next((b for b in real_world.buoys if b.position == point), None)

        if matching_buoy:
            # Store buoy in world with image=True
            world.buoys.append(Buoy(color=matching_buoy.color,
                                    position=matching_buoy.position,
                                    image=True))
            if matching_buoy.color == "yellow":
                print(f"🟡 Buoy discovered: color={matching_buoy.color}, position={matching_buoy.position}")
            elif matching_buoy.color == "red":
                print(f"🔴 Buoy discovered: color={matching_buoy.color}, position={matching_buoy.position}")
            elif matching_buoy.color == "black":
                print(f"⚫ Buoy discovered: color={matching_buoy.color}, position={matching_buoy.position}")
            elif matching_buoy.color == "white":
                print(f"⚪ Buoy discovered: color={matching_buoy.color}, position={matching_buoy.position}")

            # Execute the right move
            print(f"➡️ Move for color {matching_buoy.color} executed")

            # Update last position
            last_position = matching_buoy.position
            
        else:
            print(f"⚠️ No buoy found at {point}")
            return False

    # After finishing all interesting points → update current position
    if last_position:
        world.coordinates.current = last_position
        print(f"📍 Robot final position updated to last buoy at {last_position}")

    return True


# def buoy_survey(world: World, real_world: World):
#     """Search for the closest unknown buoy and update world state."""
#     # --- If all buoys already discovered ---
#     if len(world.buoys) >= len(real_world.buoys):
#         print("⚠️ No more buoys to be found.")
#         return False

#     # --- Find closest unknown buoy ---
#     known_positions = [b.position for b in world.buoys]
#     current_pos = world.coordinates.current

#     unknown_buoys = [b for b in real_world.buoys if b.position not in known_positions]

#     # Get closest buoy by distance
#     closest_buoy = min(unknown_buoys, key=lambda b: compute_distance(current_pos, b.position))

#     # Store buoy into world
#     world.buoys.append(closest_buoy)
#     print(f"{closest_buoy.color} buoy discovered at {closest_buoy.position}")

#     # # --- Update robot current position ---
#     # # Random displacement within radius 5
#     # angle = random.uniform(0, 2*np.pi)
#     # radius = random.uniform(0, 5.0)
#     # displacement = np.array([radius * np.cos(angle), radius * np.sin(angle), 0.0])

#     # new_position = np.array(closest_buoy.position) + displacement
#     # world.coordinates.current = new_position.tolist()

#     world.coordinates.current = rand_surroundings(closest_buoy.position, 5.0)

#     print(f"🤖 Robot is at {world.coordinates.current}")

#     return True

# ----------------------
# Pipeline actions
# ----------------------
def given_pipeline_survey(world: World, real_world: World, pipeline_id: int) -> bool:
    """Search for a pipeline by ID in real_world and store its metadata in world if not already known."""

    # --- Look for pipeline in real_world ---
    real_pipeline = next((p for p in real_world.pipelines if p.console_marker.number == pipeline_id), None)

    if not real_pipeline:
        print(f"⚠️ No pipeline with ID {pipeline_id}.")
        return False

    # --- Check if already known ---
    if any(p.console_marker.number == pipeline_id for p in world.pipelines):
        print(f"ℹ️ Pipeline with ID {pipeline_id} is already known.")
        world.coordinates.current = rand_surroundings(real_pipeline.console_marker.position, 1.0)
        print(f"🤖 Robot is now at {world.coordinates.current}")
        return False

    # --- Store only pipeline metadata (console_marker + empty pipes list) ---
    new_pipeline = Pipeline(
        console_marker=Marker(
            color=real_pipeline.console_marker.color,
            position=list(real_pipeline.console_marker.position),
            number=real_pipeline.console_marker.number,
            shape=real_pipeline.console_marker.shape,
            image=True
        ),
        pipes=[],   # Do not copy over real pipes yet
        image=True
    )

    world.pipelines.append(new_pipeline)

    print(f"🛠️ Pipeline {pipeline_id} discovered at {new_pipeline.console_marker.position}")
    world.coordinates.current = rand_surroundings(new_pipeline.console_marker.position, 1.0)
    print(f"🤖 Robot is now at {world.coordinates.current}")

    return True



# def pipeline_survey(world: World, real_world: World):
#     """Discover the closest unknown pipeline and move robot nearby."""
#     # --- Already known IDs ---
#     known_ids = {p.console_marker.number for p in world.pipelines}

#     # --- Unknown pipelines ---
#     unknown_pipelines = [p for p in real_world.pipelines if p.id not in known_ids]

#     if not unknown_pipelines:
#         print("✅ All pipelines have already been discovered.")
#         return False

#     # --- Robot's current position ---
#     current_pos = world.coordinates.current

#     # --- Pick closest unknown pipeline ---
#     closest_pipeline = min(unknown_pipelines,
#                            key=lambda p: np.linalg.norm(np.array(p.position) - np.array(current_pos)))

#     # --- Compute radius from area (if defined) ---
#     rad = np.sqrt(closest_pipeline.area / np.pi) if hasattr(closest_pipeline, "area") else 5.0

#     # --- Store pipeline in world ---
#     world.pipelines.append(Pipeline(position=closest_pipeline.position,
#                                     id=closest_pipeline.id,
#                                     image=True))
#     print(f"🛠️ Pipeline {closest_pipeline.id} discovered at {closest_pipeline.position}")

#     # --- Move robot to random point around pipeline ---
#     world.coordinates.current = rand_surroundings(closest_pipeline.position, rad)
#     print(f"🤖 Robot is now at {world.coordinates.current}")

#     return True



def check_pipes(world: World, real_world: World) -> bool:
    """Check pipelines for red markers and update the world state."""

    if not world.pipelines:
        print("⚠️ No known pipelines in world to search.")
        return False

    # --- Choose pipeline to search ---
    if len(world.pipelines) == 1:
        chosen_pipeline = world.pipelines[0]
    else:
        current_pos = np.array(world.coordinates.current)
        chosen_pipeline = min(
            world.pipelines,
            key=lambda p: np.linalg.norm(np.array(p.console_marker.position) - current_pos)
        )

    pipeline_num = chosen_pipeline.console_marker.number
    print(f"🔎 Searching pipeline {pipeline_num} for red marker...")

    # --- Find the corresponding real pipeline ---
    real_pipeline = next(
        (p for p in real_world.pipelines if p.console_marker.number == pipeline_num),
        None
    )
    if not real_pipeline:
        print(f"⚠️ Pipeline {pipeline_num} not found.")
        return False

    found_marker = False

    # --- Replace chosen pipes with real ones if a red marker exists ---
    for real_pipe in real_pipeline.pipes:
        # check if any marker is red
        if any(marker.color.lower() == "red" for marker in getattr(real_pipe, "markers", [])):
            # Append the pipe to the world pipeline
            chosen_pipeline.pipes.append(real_pipe)  # <-- safe
            chosen_pipeline.image = True
            print(f"🟥 Red marker found on pipe {real_pipe.number} (pipeline {pipeline_num})")

            # Move robot near the red marker
            red_marker = next(m for m in real_pipe.markers if m.color.lower() == "red")
            world.coordinates.current = rand_surroundings(red_marker.position, 1.0)
            print(f"🤖 Robot is now at {world.coordinates.current}")

            found_marker = True
            return True

    if not found_marker:
        print(f"ℹ️ No red marker found on pipeline {pipeline_num}.")
        return False
        
    


# ----------------------
def reach_end(world: World, real_world: World):
    """Move robot to the end position of the main pipe."""
    world.main_pipe.end_position = real_world.main_pipe.end_position.copy()
    world.coordinates.current = rand_surroundings(real_world.main_pipe.end_position, 1.0)
    print(f"🏁 Robot reached the end of the main pipe at {world.coordinates.current}")
    return True

def follow_pipe(world: World, real_world: World):
    """Move robot along the main pipe markers and record them in the world."""
    if world.main_pipe.end_position:
        if not real_world.main_pipe.markers:
            print("⚠️ No markers found in the real main pipe.")
            return False

        # Clear previous markers in world (to avoid duplicates if called multiple times)
        world.main_pipe.markers = []

        for marker in real_world.main_pipe.markers:
            # Copy marker into world
            world.main_pipe.markers.append(marker)

            # Compute distance from end
            distance = compute_distance(marker.position, real_world.main_pipe.end_position)

            # Print marker info
            if marker.color.lower() == "red":
                print(f"🟥 Found {marker.color} marker at {distance:.2f}m from the end")
            elif marker.color.lower() == "green":
                print(f"🟩 Found {marker.color} marker at {distance:.2f}m from the end")

        return True
    else:
        print("⚠️ Main pipe not found.")
        return False

def close_valve(world: World):
    """Check if the red marker on the pipeline has been found and close the valve."""
    if not world.pipelines:
        print("⚠️ No pipelines found")
        return False
        
    for pipeline in world.pipelines:
        if not hasattr(pipeline, 'pipes') or not pipeline.pipes:
            print(f"⚠️ No pipes found in pipeline")
            continue
            
        for pipe in pipeline.pipes:
            if not hasattr(pipe, 'markers'):
                continue
                
            for marker in pipe.markers:
                if marker.color.lower() == "red":
                    print(f"𓐟 Valve on pipe {pipe.number} grabbed.")
                    pipe.valve_closure = 45  
                    print(f"𓐟 Valve closed of {pipe.valve_closure}°.")
                    pipe.valve_closure = 90
                    print(f"𓐟 Valve closed of {pipe.valve_closure}°.")
                    print(f"✅ Valve on pipe {pipe.number} successfully closed and released.")
                    world.coordinates.current = rand_surroundings(pipeline.console_marker.position, 1.0)
                    print(f"🤖 Robot is now at {world.coordinates.current}")
                    return True

    print("⚠️ No red marker found on any known pipeline. Cannot close valve.")
    return False

def grab_ring(world: World):
    """Grab the ring on the pipeline with the marker if it exists."""
    for pipeline in world.pipelines:
        for pipe in pipeline.pipes:
            for marker in pipe.markers:
                if marker.color.lower() == "red":
                    print(f"🪢 Ring on pipe {pipe.number} touched.")
                    pipe.ring = "touched"
                    print(f"🪢 Ring grabbed and detached from console.")
                    pipe.ring = "grabbed"
                    world.coordinates.current = rand_surroundings(pipeline.console_marker.position, 1.0)
                    print(f"🤖 Robot is now at {world.coordinates.current}")
                    return True

    print("⚠️ No red marker found on any known pipeline. Cannot grab ring.")
    return False

def surface(world: World):
    """If ring is grabbed, surface the robot."""
    for pipeline in world.pipelines:
        for pipe in pipeline.pipes:
            if pipe.ring == "grabbed":
                print("🚤 Ring secured. Surfacing...")
                world.coordinates.current[2] = 0.0  # Set depth to 0
                print(f"🚤 Robot surfaced at position {world.coordinates.current}")
                return True
            else:
                print(f"⚠️ No ring has been grabbed. Cannot surface.")
                return False

    print("⚠️ No ring has been grabbed. Cannot surface.")
    return False

def tabula_rasa(world: World, external_waypoints: List[List[float]]):
    """Reset the world state to initial conditions."""
    try:
        # Reset all world attributes to initial state
        world.sensors.receiver = False
        world.sensors.camera = False
        world.sensors.sonar = False
        
        world.coordinates.current = [0.0, 0.0, 0.0]
        world.coordinates.waypoints.clear()
        
        world.buoys.clear()

        for pipeline in world.pipelines:
            for pipe in pipeline.pipes:
                for marker in pipe.markers:
                    if marker.color.lower() == "red":
                        pipe.valve_closure = 0
                        pipe.ring = "untouched"

        world.pipelines.clear()

        world.interesting_points.clear()
        
        world.gate.position1 = None
        world.gate.position2 = None
        
        world.main_pipe.markers.clear()
        world.main_pipe.end_position = None
        
        # Reset external waypoints
        external_waypoints.clear()
        external_waypoints.append([55.0, 0.0, 0.0])
        
        print("🧹 World state reset to initial conditions")
        return True
        
    except Exception as e:
        print(f"❌ Error during reset: {e}")
        return False



