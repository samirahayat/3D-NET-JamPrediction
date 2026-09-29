"""
ground_station.py — Fixed ground station for aggregating drone reports.

The ground station is placed at the lower-left corner of the grid.
It receives junction status reports from drones that are within
communication range — either directly or relayed via other drones
forming a connected ad-hoc network.

Key responsibilities:
  • Aggregate per-junction monitoring reports from all drones.
  • Track jam history and learned monitoring weights per junction.
  • Provide metrics:  update frequency and jam monitoring efficiency.
  • Share learned weights back to drones so they can prioritise
    high-congestion junctions.
"""

from collections import deque

from mesa import Agent


# ─────────────────────────────────────────────────────────────────
#  JunctionReport — lightweight data object
# ─────────────────────────────────────────────────────────────────

class JunctionReport:
    """A snapshot of a junction's status filed by a drone."""

    __slots__ = ("junction_id", "tick", "is_jammed",
                 "queue_lengths", "drone_id",
                 "segment_metrics", "predictions")

    def __init__(self, junction_id, tick, is_jammed,
                 queue_lengths, drone_id,
                 segment_metrics=None, predictions=None):
        self.junction_id = junction_id
        self.tick = tick
        self.is_jammed = is_jammed
        self.queue_lengths = dict(queue_lengths)   # direction → count
        self.drone_id = drone_id
        self.segment_metrics = segment_metrics or {}  # dir → {density, avg_velocity, flow, …}
        self.predictions = predictions or {}          # dir → {estimated_inflow, …}


# ─────────────────────────────────────────────────────────────────
#  GroundStation — MESA agent placed on the grid
# ─────────────────────────────────────────────────────────────────

class GroundStation(Agent):
    """
    A fixed ground station that collects and aggregates drone
    monitoring reports.

    Placed at the lower-left corner of the grid.  Drones deliver
    reports when they (or a connected chain of drones) are within
    communication range.

    Tracked metrics
    ---------------
    *Per-junction*
        last_update_tick : most recent tick a report was received.
        visit_count      : total reports received.
        jam_count        : total distinct jam events observed.
        jam_weights      : learned monitoring priority weight.

    *Global*
        total_updates        : lifetime count of received reports.
        updates_per_tick     : rolling list of ``(tick, count)`` pairs.
        jam_detection_delays : list of ticks between a jam starting
                               (first jammed report) and the drone
                               arriving at the junction.
    """

    def __init__(self, unique_id, model, pos=(1, 1)):
        super().__init__(unique_id, model)
        self.station_pos = pos

        # ── Per-junction tracking ────────────────────────────────
        self.last_update_tick = {}    # jid → tick
        self.visit_count = {}         # jid → int
        self.jam_count = {}           # jid → int
        self.jam_weights = {}         # jid → float (learned priority)

        # ── Global metrics ───────────────────────────────────────
        self.total_updates = 0
        self._updates_this_tick = 0
        self._ticks_recorded = 0
        # Keep only a rolling window for plotting/debugging to avoid
        # unbounded growth in long-running live simulations.
        self.updates_per_tick = deque(maxlen=4096)    # [(tick, count), …]

        # ── Jam tracking ─────────────────────────────────────────
        self.active_jams = {}         # jid → tick first reported
        self.jam_detection_delays = []
        self.monitored_jam_ticks = 0  # kept for GS-local reference
        self.total_jam_ticks = 0      # kept for GS-local reference

        # ── Snapshot of latest status per junction ───────────────
        self.junction_status = {}     # jid → {is_jammed, queues, tick}

    # ── Report ingestion ─────────────────────────────────────────

    def receive_report(self, report):
        """Process one :class:`JunctionReport` from a drone."""
        jid = report.junction_id
        tick = report.tick

        self.last_update_tick[jid] = tick
        self.visit_count[jid] = self.visit_count.get(jid, 0) + 1
        self.total_updates += 1
        self._updates_this_tick += 1

        # Update latest snapshot
        self.junction_status[jid] = {
            "is_jammed": report.is_jammed,
            "queues": report.queue_lengths,
            "tick": tick,
            "segment_metrics": report.segment_metrics,
            "predictions": report.predictions,
        }

        if report.is_jammed:
            if jid not in self.active_jams:
                # New jam event
                self.active_jams[jid] = tick
                self.jam_count[jid] = self.jam_count.get(jid, 0) + 1
            # Increase monitoring weight
            self.jam_weights[jid] = self.jam_weights.get(jid, 1.0) + 0.5
        else:
            # Jam cleared (or was never jammed)
            self.active_jams.pop(jid, None)

    # ── Per-tick bookkeeping (scheduled by MESA) ─────────────────

    def step(self):
        """Record update-frequency metrics, decay weights, notify controllers."""
        tick = getattr(self.model, "_tick_count", 0)

        # Record how many reports arrived this tick
        self.updates_per_tick.append((tick, self._updates_this_tick))
        self._ticks_recorded += 1
        self._updates_this_tick = 0

        # Count active-jam ticks (GS-view, for its own bookkeeping)
        if self.active_jams:
            self.total_jam_ticks += len(self.active_jams)

        # Slowly decay jam weights so recent jams weigh more
        for jid in list(self.jam_weights):
            self.jam_weights[jid] = max(1.0,
                                        self.jam_weights[jid] * 0.995)

        # ── Push congestion info to junction controllers ─────────
        self._notify_controllers()

    # ── Query helpers (used by visualization & drones) ───────────

    def get_jam_weights(self):
        """Return a copy of the current jam-priority weights."""
        return dict(self.jam_weights)

    def get_avg_update_frequency(self):
        """Average reports received per tick over the entire run."""
        if self._ticks_recorded <= 0:
            return 0.0
        return self.total_updates / self._ticks_recorded

    def get_junction_coverage(self):
        """
        Per-junction staleness: ticks since the last report.

        Returns ``{jid: staleness}`` for every known junction.
        """
        tick = getattr(self.model, "_tick_count", 0)
        coverage = {}
        for jid in self.model.junction_controllers:
            last = self.last_update_tick.get(jid, -1)
            coverage[jid] = tick - last if last >= 0 else tick
        return coverage

    def get_monitoring_efficiency(self):
        """
        Delegates to the model's objective efficiency.
        """
        return self.model.get_monitoring_efficiency()

    def is_junction_jammed(self, jid):
        """True if the most recent report for *jid* says it is jammed."""
        status = self.junction_status.get(jid)
        if status is None:
            return False
        return status.get("is_jammed", False)

    # ── Adaptive traffic-signal notification ─────────────────────

    def _notify_controllers(self):
        """
        Push congestion and prediction data to junction controllers
        so they can adapt signal timing.

        For each junction with a recent drone report, determine which
        approach directions are congested and forward the information
        (together with inflow predictions) to the controller.
        """
        if not getattr(self.model, "adaptive_signal_control", True):
            return

        from drone import JAM_THRESHOLD

        road_priority = getattr(self.model, "road_priority", "V")

        for jid, status in self.junction_status.items():
            ctrl = self.model.junction_controllers.get(jid)
            if ctrl is None:
                continue

            congested_approaches = set()

            # Use queue-based detection
            queues = status.get("queues", {})
            for d, q in queues.items():
                if q >= JAM_THRESHOLD:
                    congested_approaches.add("H" if d in ("E", "W") else "V")

            # Also use macroscopic density + velocity criteria
            density_threshold = getattr(self.model, "jam_density_threshold", 0.5)
            velocity_threshold = getattr(self.model, "jam_velocity_threshold", 1.0)
            seg = status.get("segment_metrics", {})
            for d, metrics in seg.items():
                density = metrics.get("density", 0)
                avg_v = metrics.get("avg_velocity", 0)
                if density >= density_threshold and avg_v <= velocity_threshold:
                    congested_approaches.add("H" if d in ("E", "W") else "V")

            predictions = status.get("predictions", {})

            ctrl.apply_congestion_response(
                congested_approaches,
                predictions=predictions,
                road_priority=road_priority,
            )
