from dataclasses import dataclass, field
from typing import List
import random

starting_position = [25.0, 90.0, 0.0]

# ROBOT STATE
@dataclass
class Coordinates:
    """Robot coordinate system and waypoint management"""
    current: List[float] = field(default_factory=lambda: starting_position.copy())
    waypoints: List[List[float]] = field(default_factory=list)

@dataclass
class Sensors:
    """Robot sensor activation status"""
    receiver: bool = False
    camera: bool = False
    sonar: bool = False

# ENVIRONMENT STATE
@dataclass
class Buoy:
    """Buoy object in the environment"""
    color: str = "unknown"
    position: List[float] = field(default_factory=list)
    image: bool = False

@dataclass
class Gate:
    """Gate object in the environment"""
    position1: List[float] = field(default_factory=list)
    position2: List[float] = field(default_factory=list)
    image: bool = False

@dataclass
class Marker:
    """Marker object in the environment"""
    color: str = "unknown"
    position: List[float] = field(default_factory=list)
    number: int = 0
    shape: str = "unknown"
    image: bool = False

@dataclass
class Pipe:
    """Individual pipe component of pipeline"""
    markers: list[Marker] = field(default_factory=list)
    number: int = 0
    valve_closure: int = 0
    ring: str = "untouched"
    image: bool = False


@dataclass
class Pipeline:
    """Pipeline structure containing two pipes"""
    console_marker: Marker = field(default_factory=Marker)     
    pipes: list[Pipe] = field(default_factory=list)
    image: bool = False

@dataclass
class MainPipe:
    """Main pipe that contains coloured markers"""
    end_position: List[float] = field(default_factory=list)
    markers: List[Marker] = field(default_factory=list)

@dataclass
class World:
    """Complete world state representation"""
    coordinates: Coordinates = field(default_factory=Coordinates)
    sensors: Sensors = field(default_factory=Sensors)
    buoys: List[Buoy] = field(default_factory=list)
    gate: Gate = field(default_factory=Gate)
    pipelines: List[Pipeline] = field(default_factory=list)
    interesting_points: List[List[float]] = field(default_factory=list)
    main_pipe: MainPipe = field(default_factory=MainPipe)
