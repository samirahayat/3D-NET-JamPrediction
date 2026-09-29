"""
results_logger.py — Experiment-grade CSV logging for traffic simulation runs.

File naming:  Runnum_{run_num}_drones_{D}_2lanprob_{L}_numjunc_{J}_spawnrate_{S}

CSVs produced (all written to results/):
  run_summary.csv            — one aggregate row per run  (appended across runs)
  Runnum_…_jams.csv          — one row per closed jam event
  Runnum_…_obstacles.csv     — one row per obstacle spawn / clear
"""

import csv
import os
from datetime import datetime


# ─────────────────── helpers ────────────────────────────────────────────────

def _fmt(value, digits=4):
    if value is None:
        return ""
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return value


def _next_run_num(results_dir: str) -> int:
    """Monotonically-increasing run counter persisted in results/run_counter.txt."""
    path = os.path.join(results_dir, "run_counter.txt")
    n = 1
    if os.path.exists(path):
        try:
            n = int(open(path).read().strip()) + 1
        except Exception:
            n = 1
    with open(path, "w") as fh:
        fh.write(str(n))
    return n


def _ensure_csv(path, headers):
    """Create CSV with header row if absent.  If present with different headers,
    archive the old file by renaming it so the new schema starts clean."""
    if not os.path.exists(path):
        with open(path, "w", newline="", encoding="utf-8") as handle:
            csv.DictWriter(handle, fieldnames=headers).writeheader()
        return
    # Peek at existing header row
    try:
        with open(path, "r", encoding="utf-8") as handle:
            existing_header = next(csv.reader(handle), [])
        if list(existing_header) == list(headers):
            return   # headers match — nothing to do
    except Exception:
        pass  # unreadable — fall through and overwrite
    # Headers differ: archive old file and create fresh one
    from datetime import datetime as _dt
    stamp = _dt.now().strftime("%Y%m%d_%H%M%S")
    archived = path.replace(".csv", f"_archived_{stamp}.csv")
    os.rename(path, archived)
    print(f"[logger] Archived incompatible CSV -> {os.path.basename(archived)}")
    with open(path, "w", newline="", encoding="utf-8") as handle:
        csv.DictWriter(handle, fieldnames=headers).writeheader()


# ─────────────────── column definitions ─────────────────────────────────────

# One row per completed jam event (primary analysis table).
JAM_HEADERS = [
    # Run identity
    "run_num", "run_id", "seed",
    "num_drones", "lanes_choice_prob", "num_junctions", "spawn_prob",
    # Jam identity
    "jam_id", "junction_id", "direction",
    # Timing
    "start_tick", "end_tick", "duration_ticks",
    # Detection
    "detected",
    "detection_delay_ticks",   # ticks from jam start to first drone detection tick
    # Jam longevity across signal cycles
    "red_phases_spanned",      # how many red phases the jam survived
    # Drone coverage during the jam
    "drone_visit_ticks",       # total ticks any drone was hovering at this locale while jammed
    "num_drone_visits",        # number of distinct arrivals of a drone during the jam
    # Prediction memory (did a drone predict this jam *before* it started?)
    "predicted_before_start",  # 1 / 0
    "prediction_tick",         # tick when the pre-jam prediction was issued
    "lead_time_ticks",         # start_tick - prediction_tick
    "prediction_drone_id",     # which drone issued it
    # Traffic metrics (averages over the entire jam lifetime)
    "avg_flow",
    "avg_delta_k",
    "avg_velocity",
    "peak_density",
    "peak_queue",
    # Post-detection residual duration
    "post_detect_duration_ticks",  # end_tick - first_detect_tick (controller benefit proxy)
]

# One row per run (aggregate KPIs for scatter / box plots).
SUMMARY_HEADERS = [
    "run_num", "run_id", "timestamp", "seed", "ticks",
    "num_drones", "lanes_choice_prob", "num_junctions", "spawn_prob",
    "v_max", "random_slow_prob",
    "green_duration", "yellow_duration",
    "jam_capacity_threshold", "jam_sustain_ticks",
    "hover_ticks_min", "hover_ticks_max",
    "cell_size_m", "tick_duration_s",
    "prediction_horizon",
    "event_spawn_prob", "event_duration_min", "event_duration_max",
    # KPIs
    "total_jam_events",
    "detected_jam_events",
    "missed_jam_events",
    "detection_rate",
    "mean_detection_delay",
    "median_detection_delay",
    "predicted_jam_events",        # jams that had a pre-jam prediction
    "prediction_rate",             # predicted_jam_events / total_jam_events
    "mean_lead_time",
    "median_lead_time",
    "mean_drone_visit_ticks",
    "mean_jam_duration",
    "mean_red_phases_spanned",     # avg signal cycles a jam survived
    "obstacle_events_spawned",
    "obstacle_events_cleared",
    # ── Does detection shorten jams? (controller signal-adaptation outcome) ──
    "mean_post_detect_duration",   # avg residual jam ticks after first detection
    "mean_duration_detected",      # avg jam duration — detected jams only
    "mean_duration_missed",        # avg jam duration — undetected jams only
    # ── Does controller break jams faster once drones report? ──
    "mean_red_phases_detected",    # avg red phases survived — detected jams
    "mean_red_phases_missed",      # avg red phases survived — undetected jams
    # ── Does pre-jam prediction reduce jam severity? ──
    "mean_duration_predicted",     # avg duration — jams that had a pre-jam alert
    "mean_duration_not_predicted", # avg duration — jams with no pre-jam alert
    # ── Prediction quality ──
    "false_positive_predictions",  # prediction records that expired without a jam
    "prediction_precision",        # prediction_correct / prediction_total
]

OBSTACLE_HEADERS = [
    "run_num", "run_id", "obstacle_id", "action", "tick",
    "x", "y", "direction", "duration_ticks", "end_tick",
]


# ─────────────────── SimulationLogger ────────────────────────────────────────

class SimulationLogger:
    """
    Per-run CSV writer.

    Construct at model init:
        self.logger = SimulationLogger(run_metadata)

    Call:
        logger.log_jam_event(evt)       when a jam closes
        logger.log_obstacle_event(row)  on spawn / clear
        logger.finalize(summary_dict)   at end of run
    """

    def __init__(self, run_metadata: dict, results_dir: str = "results"):
        os.makedirs(results_dir, exist_ok=True)
        self.results_dir = results_dir
        self.run_metadata = dict(run_metadata)

        # Monotonic run counter (shared across all runs in results_dir)
        self.run_num = _next_run_num(results_dir)

        d   = int(self.run_metadata.get("num_drones", 0))
        lp  = float(self.run_metadata.get("lanes_choice_prob", 0))
        j   = int(self.run_metadata.get("num_junctions", 0))
        sp  = float(self.run_metadata.get("spawn_prob", 0))

        self.run_id = (
            f"Runnum_{self.run_num}"
            f"_drones_{d}"
            f"_2lanprob_{lp:.2f}"
            f"_numjunc_{j}"
            f"_spawnrate_{sp:.3f}"
        )

        self._summary_path   = os.path.join(results_dir, "run_summary.csv")
        self._jams_path      = os.path.join(results_dir, f"{self.run_id}_jams.csv")
        self._obstacles_path = os.path.join(results_dir, f"{self.run_id}_obstacles.csv")

        _ensure_csv(self._summary_path,   SUMMARY_HEADERS)
        _ensure_csv(self._jams_path,      JAM_HEADERS)
        _ensure_csv(self._obstacles_path, OBSTACLE_HEADERS)

        self._obstacles_spawned = 0
        self._obstacles_cleared = 0

    # ── public write methods ─────────────────────────────────────────────────

    def log_jam_event(self, evt: dict):
        """Write one completed jam event row to the per-run jams CSV."""
        row = {
            "run_num":              self.run_num,
            "run_id":               self.run_id,
            "seed":                 self.run_metadata.get("seed", ""),
            "num_drones":           self.run_metadata.get("num_drones", ""),
            "lanes_choice_prob":    _fmt(self.run_metadata.get("lanes_choice_prob"), 3),
            "num_junctions":        self.run_metadata.get("num_junctions", ""),
            "spawn_prob":           _fmt(self.run_metadata.get("spawn_prob"), 4),
            "jam_id":               evt.get("event_id", ""),
            "junction_id":          evt.get("junction_id", ""),
            "direction":            evt.get("direction", ""),
            "start_tick":           evt.get("start_tick", ""),
            "end_tick":             evt.get("end_tick", ""),
            "duration_ticks":       evt.get("duration", ""),
            "detected":             int(bool(evt.get("detected", False))),
            "detection_delay_ticks": evt.get("detection_delay", ""),
            "red_phases_spanned":   evt.get("red_phases_spanned", 0),
            "drone_visit_ticks":    evt.get("drone_visit_ticks", 0),
            "num_drone_visits":     evt.get("num_drone_visits", 0),
            "predicted_before_start": int(bool(evt.get("predicted_before_start", False))),
            "prediction_tick":      evt.get("prediction_tick", ""),
            "lead_time_ticks":      evt.get("lead_time_ticks", ""),
            "prediction_drone_id":  evt.get("prediction_drone_id", ""),
            "avg_flow":             _fmt(evt.get("avg_flow")),
            "avg_delta_k":          _fmt(evt.get("avg_delta_k")),
            "avg_velocity":         _fmt(evt.get("avg_velocity")),
            "peak_density":         _fmt(evt.get("peak_density")),
            "peak_queue":           evt.get("peak_queue", 0),
            "post_detect_duration_ticks": evt.get("post_detect_duration", ""),
        }
        with open(self._jams_path, "a", newline="", encoding="utf-8") as fh:
            csv.DictWriter(fh, fieldnames=JAM_HEADERS).writerow(row)

    def log_obstacle_event(self, row: dict):
        """Write one obstacle spawn / clear row."""
        payload = {
            "run_num":        self.run_num,
            "run_id":         self.run_id,
            "obstacle_id":    row.get("obstacle_id", ""),
            "action":         row.get("action", ""),
            "tick":           row.get("tick", ""),
            "x":              row.get("x", ""),
            "y":              row.get("y", ""),
            "direction":      row.get("direction", ""),
            "duration_ticks": row.get("duration_ticks", ""),
            "end_tick":       row.get("end_tick", ""),
        }
        with open(self._obstacles_path, "a", newline="", encoding="utf-8") as fh:
            csv.DictWriter(fh, fieldnames=OBSTACLE_HEADERS).writerow(payload)
        if payload["action"] == "spawn":
            self._obstacles_spawned += 1
        elif payload["action"] == "clear":
            self._obstacles_cleared += 1

    def finalize(self, summary: dict):
        """Append one aggregate row to run_summary.csv."""
        row = {
            "run_num":   self.run_num,
            "run_id":    self.run_id,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            **{k: self.run_metadata.get(k, "") for k in [
                "seed", "ticks",
                "num_drones", "lanes_choice_prob", "num_junctions", "spawn_prob",
                "v_max", "random_slow_prob",
                "green_duration", "yellow_duration",
                "jam_capacity_threshold", "jam_sustain_ticks",
                "hover_ticks_min", "hover_ticks_max",
                "cell_size_m", "tick_duration_s",
                "prediction_horizon",
                "event_spawn_prob", "event_duration_min", "event_duration_max",
            ]},
            **summary,
            "obstacle_events_spawned": self._obstacles_spawned,
            "obstacle_events_cleared": self._obstacles_cleared,
        }
        # Guard: skip if this (num_drones, spawn_prob, lanes_choice_prob, num_junctions, seed)
        # key already exists — prevents duplicate rows from concurrent/repeated runs.
        _dedup_key = (row.get("num_drones"), row.get("spawn_prob"),
                      row.get("lanes_choice_prob"), row.get("num_junctions"),
                      row.get("seed"))
        try:
            if os.path.exists(self._summary_path):
                import csv as _csv
                with open(self._summary_path, "r", encoding="utf-8") as _fh:
                    _reader = _csv.DictReader(_fh)
                    for _existing in _reader:
                        _existing_key = (
                            _existing.get("num_drones"), _existing.get("spawn_prob"),
                            _existing.get("lanes_choice_prob"), _existing.get("num_junctions"),
                            _existing.get("seed"),
                        )
                        if _existing_key == tuple(str(v) for v in _dedup_key):
                            return  # already logged — skip
        except Exception:
            pass  # if check fails, fall through and write anyway
        with open(self._summary_path, "a", newline="", encoding="utf-8") as fh:
            csv.DictWriter(fh, fieldnames=SUMMARY_HEADERS).writerow(row)

    # Backward-compat alias used by model.py
    def finalize_run(self, summary: dict):
        self.finalize(summary)
