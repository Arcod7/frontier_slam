"""Stateful goal selection from frontier clusters.

Wraps three concerns that all act on the same committed goal:
  - nearest-frontier selection with hysteresis (don't flip-flop between equidistant goals)
  - stuck detection (abandon goals the robot can't approach)
  - timed blacklist (don't immediately re-pick an abandoned goal)
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class GoalSelection:
    """Result of one GoalManager.select() call."""
    gx: float
    gy: float
    stuck_pct: int        # 0–100, percent of stuck timeout elapsed on current goal
    event: str = ''       # '', 'STUCK_BLACKLIST', 'ALL_BLACKLISTED'
    # Populated only when event == 'STUCK_BLACKLIST', for user-facing logging.
    stuck_goal: tuple = None     # (x, y, elapsed_s, progress_m)


class GoalManager:
    def __init__(self, *,
                 min_explore_dist: float = 3.0,
                 switch_hysteresis: float = 3.0,
                 goal_vanish_dist: float = 3.0,
                 stuck_timeout: float = 15.0,
                 stuck_min_progress: float = 0.5,
                 blacklist_duration: float = 30.0):
        self.min_explore_dist  = min_explore_dist
        self.switch_hysteresis = switch_hysteresis
        self.goal_vanish_dist  = goal_vanish_dist
        self.stuck_timeout     = stuck_timeout
        self.stuck_min_progress = stuck_min_progress
        self.blacklist_duration = blacklist_duration

        self._committed: np.ndarray | None = None
        self._committed_time: float = 0.0
        self._committed_dist: float = float('inf')
        self._blacklist: list = []   # [(wx, wy, expiry_time)]

    # ---- queries
    @property
    def blacklist_size(self) -> int:
        return len(self._blacklist)

    def is_blacklisted(self, wx: float, wy: float, now: float) -> bool:
        self._blacklist = [(x, y, t) for x, y, t in self._blacklist if t > now]
        return any(
            np.hypot(wx - x, wy - y) < self.goal_vanish_dist
            for x, y, _ in self._blacklist
        )

    # ---- main entry point
    def select(self, clusters, robot_xy: np.ndarray, now: float):
        """Pick the next goal from the cluster list.

        Returns None when there are candidates but they're all within MIN_EXPLORE_DIST
        (the robot is already "at" every frontier). Returns a GoalSelection with
        event='ALL_BLACKLISTED' when every remaining candidate is blacklisted.
        """
        candidates = [c for c in clusters if c.distance >= self.min_explore_dist]
        if not candidates:
            return None

        candidates = [c for c in candidates if not self.is_blacklisted(c.wx, c.wy, now)]
        if not candidates:
            return GoalSelection(float('nan'), float('nan'), 0, 'ALL_BLACKLISTED')

        # Score = distance / size: prefer large clusters and close ones equally.
        # This breaks ties between equidistant clusters in favour of the more
        # information-rich one, and naturally deprioritises tiny noise patches.
        candidates.sort(key=lambda c: c.distance / c.size)

        stuck_info = self._check_and_blacklist_if_stuck(candidates, robot_xy, now)
        event = 'STUCK_BLACKLIST' if stuck_info else ''
        if stuck_info:
            candidates = [c for c in candidates if not self.is_blacklisted(c.wx, c.wy, now)]
            if not candidates:
                return GoalSelection(float('nan'), float('nan'), 0, event, stuck_info)

        gx, gy = self._pick_with_hysteresis(candidates, robot_xy, now)
        stuck_pct = min(100, int((now - self._committed_time) / self.stuck_timeout * 100))
        return GoalSelection(gx, gy, stuck_pct, event, stuck_info)

    # ---- internals
    def _check_and_blacklist_if_stuck(self, candidates, robot_xy, now):
        if self._committed is None:
            return None
        cur_dist = np.hypot(self._committed[0] - robot_xy[0],
                            self._committed[1] - robot_xy[1])
        elapsed  = now - self._committed_time
        progress = self._committed_dist - cur_dist
        if elapsed <= self.stuck_timeout or progress >= self.stuck_min_progress:
            return None

        info = (float(self._committed[0]), float(self._committed[1]),
                float(elapsed), float(progress))
        self._blacklist.append((self._committed[0], self._committed[1],
                                now + self.blacklist_duration))
        self._committed = None
        return info

    def _pick_with_hysteresis(self, candidates, robot_xy, now):
        best = candidates[0]   # best-scored: smallest distance/size
        if self._committed is None:
            self._commit(best.wx, best.wy, robot_xy, now)
            return best.wx, best.wy

        # Has the committed goal moved with the map? Find candidates near it.
        near_old = [c for c in candidates
                    if np.hypot(c.wx - self._committed[0], c.wy - self._committed[1])
                    < self.goal_vanish_dist]
        if not near_old:
            # The committed goal vanished — pick the best-scored fresh candidate.
            self._commit(best.wx, best.wy, robot_xy, now)
            return best.wx, best.wy

        # The committed goal is still around. Only switch if the best-scored candidate
        # is also clearly closer (by distance) than where we already are — prevents
        # abandoning a large-cluster goal mid-journey for a small nearby distraction.
        committed_dist = np.hypot(self._committed[0] - robot_xy[0],
                                  self._committed[1] - robot_xy[1])
        if best.distance < committed_dist - self.switch_hysteresis:
            self._commit(best.wx, best.wy, robot_xy, now)
            return best.wx, best.wy

        # Otherwise track drift of the committed goal without resetting the stuck timer.
        drift = min(near_old, key=lambda c: np.hypot(c.wx - self._committed[0],
                                                     c.wy - self._committed[1]))
        self._committed = np.array([drift.wx, drift.wy])
        return drift.wx, drift.wy

    def _commit(self, gx: float, gy: float, robot_xy: np.ndarray, now: float) -> None:
        self._committed = np.array([gx, gy])
        self._committed_time = now
        self._committed_dist = float(np.hypot(gx - robot_xy[0], gy - robot_xy[1]))
