"""
traffic_sim.py — Backwards-compatibility shim.

The simulation has been refactored into separate modules for clarity:

    cells.py          — RoadCell, JunctionCell, ApproachLight
    controllers.py    — JunctionController (traffic-light logic)
    car.py            — CarAgent (vehicle movement & junction navigation)
    grid_builder.py   — Procedural road-network generator
    model.py          — TrafficModel (simulation loop, spawning, pathfinding)
    visualization.py  — MESA CanvasGrid portrayal & server launcher
    main.py           — Entry point (``python main.py``)

This file re-exports every public name so that existing code doing
``from traffic_sim import TrafficModel, CarAgent, ...`` continues to
work without changes.
"""

# Re-export all public classes
from cells import RoadCell, JunctionCell, ApproachLight          # noqa: F401
from controllers import JunctionController                        # noqa: F401
from car import CarAgent                                          # noqa: F401
from model import TrafficModel                                    # noqa: F401
from drone import DroneAgent                                      # noqa: F401
from ground_station import GroundStation, JunctionReport          # noqa: F401
from visualization import agent_portrayal, run_server             # noqa: F401

# Allow running this file directly as before
if __name__ == "__main__":
    run_server()
