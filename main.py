"""
main.py — Entry point for the traffic simulation.

Run this file to start the MESA visualization server::

    python main.py

Then open http://localhost:8521 in your browser.

Use the sliders on the left-hand sidebar to tune parameters like
traffic density, green-light duration, and lane counts.  Press
**Reset** to create a fresh simulation with the current settings.
"""

from visualization import run_server

if __name__ == "__main__":
    run_server()
