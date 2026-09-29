"""
controllers.py — Junction traffic-light controller.

Each junction has exactly one JunctionController that cycles through
four phases:

    H_GREEN → H_YELLOW → V_GREEN → V_YELLOW → (repeat)

  • GREEN   — cars from the matching approach (H or V) may enter.
  • YELLOW  — no new entries; the controller waits for the junction
              to empty (or until a maximum wait is exceeded) before
              switching to the next GREEN phase.

The controller is a scheduled MESA agent so its ``step()`` runs every
tick alongside the car agents.
"""

from mesa import Agent

from cells import JunctionCell


class JunctionController(Agent):
    """
    Controls traffic flow through one junction block.

    Parameters
    ----------
    junction_id : str
        Unique string id for this junction (e.g. ``"J_12_6"``).
    block_cells : list[tuple[int, int]]
        Every (x, y) position that belongs to this junction.
    green_duration : int
        Number of ticks a GREEN phase lasts before switching to YELLOW.
    yellow_duration : int
        Minimum ticks the YELLOW phase must last.
    yellow_max_wait : int
        Maximum ticks the YELLOW phase can last (forces switch even if
        the junction is still occupied).
    """

    # ── Phase cycle ──────────────────────────────────────────────
    #  H_GREEN  →  H_YELLOW  →  V_GREEN  →  V_YELLOW  →  (repeat)
    # ─────────────────────────────────────────────────────────────

    def __init__(self, unique_id, model, junction_id, block_cells,
                 green_duration=12, yellow_duration=4, yellow_max_wait=25):
        super().__init__(unique_id, model)
        self.junction_id = junction_id
        self.block_cells = set(block_cells)
        self.green_duration = green_duration
        self.yellow_duration = yellow_duration
        self.yellow_max_wait = yellow_max_wait

        # Start with horizontal traffic having the green light
        self.phase = "H_GREEN"
        self.timer = 0
        self._yellow_elapsed = 0

        # ── Adaptive signal control state ────────────────────────
        self._green_extension = 0          # extra ticks to extend current GREEN
        self._congested_approaches = set() # {'H'} / {'V'} / {'H', 'V'}
        self._road_priority = "V"          # default: vertical has priority
        self._predictions = {}             # direction → {estimated_inflow, …}
        self._prediction_switch = False    # True → shorten current green

    # ── Helpers ──────────────────────────────────────────────────

    def junction_occupied(self):
        """Return True if any JunctionCell inside this block has a car."""
        for pos in self.block_cells:
            for agent in self.model.grid.get_cell_list_contents([pos]):
                if isinstance(agent, JunctionCell) and agent.car is not None:
                    return True
        return False

    def allow_entry(self, approach_dir):
        """
        Can a car from the given approach enter right now?

        Parameters
        ----------
        approach_dir : str
            ``'H'`` for East/West approaches, ``'V'`` for North/South.

        Returns
        -------
        bool
        """
        if approach_dir == "H":
            return self.phase == "H_GREEN"
        return self.phase == "V_GREEN"

    # ── Phase-transition logic (called every tick) ───────────────

    def step(self):
        """
        Advance the phase timer and transition when conditions are met.

        Green-phase duration is dynamically adjusted:
        • Extended when the current direction is congested.
        • Shortened when the *opposite* direction is congested
          (or predicted to be).
        """
        self.timer += 1

        # Effective green = base + extension (may be negative for shortening)
        effective_green = max(4, self.green_duration + self._green_extension)

        if self.phase == "H_GREEN":
            if self.timer >= effective_green:
                self._green_extension = 0
                self._prediction_switch = False
                self._enter_yellow("H_YELLOW")

        elif self.phase == "H_YELLOW":
            self._yellow_elapsed += 1
            if self._should_exit_yellow():
                self._enter_green("V_GREEN")

        elif self.phase == "V_GREEN":
            if self.timer >= effective_green:
                self._green_extension = 0
                self._prediction_switch = False
                self._enter_yellow("V_YELLOW")

        elif self.phase == "V_YELLOW":
            self._yellow_elapsed += 1
            if self._should_exit_yellow():
                self._enter_green("H_GREEN")

    # ── Adaptive congestion response ─────────────────────────────

    def apply_congestion_response(self, congested_approaches,
                                  predictions=None, road_priority=None):
        """
        Receive congestion information from the ground station and
        adapt signal timing.

        Parameters
        ----------
        congested_approaches : set
            ``{'H'}`` / ``{'V'}`` / ``{'H', 'V'}`` indicating which
            approach type is currently congested.
        predictions : dict, optional
            Per-direction predicted inflow ``{dir: {estimated_inflow, …}}``.
        road_priority : str, optional
            ``'H'`` or ``'V'`` — which approach has higher priority when
            both directions are congested.
        """
        self._congested_approaches = congested_approaches
        if road_priority is not None:
            self._road_priority = road_priority
        if predictions is not None:
            self._predictions = predictions

        if not congested_approaches:
            self._green_extension = 0
            return

        both = "H" in congested_approaches and "V" in congested_approaches
        current_approach = None
        if self.phase == "H_GREEN":
            current_approach = "H"
        elif self.phase == "V_GREEN":
            current_approach = "V"

        if current_approach is None:
            # In a yellow phase — no extension, will be applied next green
            return

        # How much to extend / shorten (capped at ±50 % of base green)
        max_ext = self.green_duration // 2

        if both:
            # Both congested ⟶ prioritise per road_priority
            if self._road_priority == "NONE":
                # No priority — keep extensions neutral
                self._green_extension = 0
            elif current_approach == self._road_priority:
                self._green_extension = max_ext      # extend priority dir
            else:
                self._green_extension = -max_ext     # shorten non-priority
        elif current_approach in congested_approaches:
            # Only our direction is congested ⟶ extend
            self._green_extension = max_ext
        else:
            # Opposite direction congested ⟶ shorten to switch sooner
            self._green_extension = -max_ext

        # ── Prediction-based pre-emptive adjustment ──────────────
        if predictions:
            self._apply_prediction_adjustment(predictions, current_approach)

    def _apply_prediction_adjustment(self, predictions, current_approach):
        """
        If the predicted inflow for the opposite direction is high,
        shorten the current green to switch sooner (proactive response).
        """
        INFLOW_THRESHOLD = 0.3  # flow units above which we react

        opp = "V" if current_approach == "H" else "H"
        opp_dirs = ("N", "S") if opp == "V" else ("E", "W")

        max_inflow = 0.0
        for d in opp_dirs:
            pred = predictions.get(d, {})
            inflow = pred.get("estimated_inflow", 0)
            if inflow > max_inflow:
                max_inflow = inflow

        if max_inflow > INFLOW_THRESHOLD and opp not in self._congested_approaches:
            # Predicted high inflow for the opposite direction — switch early
            shortening = self.green_duration // 4
            self._green_extension = min(self._green_extension, -shortening)
            self._prediction_switch = True

    # ── Private transition helpers ───────────────────────────────

    def _enter_yellow(self, yellow_phase):
        """Transition from GREEN to the given YELLOW phase."""
        self.phase = yellow_phase
        self.timer = 0
        self._yellow_elapsed = 0

    def _enter_green(self, green_phase):
        """Transition from YELLOW to the given GREEN phase."""
        self.phase = green_phase
        self.timer = 0

    def _should_exit_yellow(self):
        """
        Return True if the YELLOW phase should end.

        Conditions (either one triggers):
        1. Minimum yellow duration elapsed AND junction is empty.
        2. Maximum yellow wait exceeded (force-switch to prevent deadlock).
        """
        min_met_and_clear = (
            self.timer >= self.yellow_duration and not self.junction_occupied()
        )
        forced = self._yellow_elapsed >= self.yellow_max_wait
        return min_met_and_clear or forced
