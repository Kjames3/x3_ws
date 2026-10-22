"""person_memory.py -- keeps people in the CBF while the tracker loses and re-labels them.

Why: the CBF only gets a person from _cbf_movers() when that person's track is matched
THIS frame, has been a person for >= 5 frames and has not jumped within a history
window.  In a busy lab (2026-10-22, 3 h of estimator_dropout_log) people were missing
from the estimator 21% of the time: 139 of 354 dropouts began with the person last seen
closer than 1 m (the camera cannot see that close), and 127 of the 142 dropouts of 1 s or
more came back under a DIFFERENT track id, which restarts every one of those guards.  A
professor walking at the robot was reacted to once, lost at 0.65 m, and ignored for ~10 s.

What this does, on top of the estimator output, in the base frame (x forward, y left):

  * Option 1, hold.  A person who was an established mover (or whose velocity was
    unknown) and disappears is kept for HOLD_S as a person-sized obstacle: its position
    runs on its last velocity for EXTRAPOLATE_S and is then frozen, and the velocity
    fades out with TAU_GHOST_S.  A person that vanishes close to the robot is far more
    likely occluded or too close for the camera than gone.
  * Option 2, re-link.  A new track appearing near a recent one (position gate that
    grows with the gap) is the same entry: tenure accumulates across id changes, and while
    the new track's own velocity is not yet trusted the last trusted velocity is carried
    forward, decaying with TAU_CARRY_S.  A track that jumped too far to be the same person
    starts a new entry with no carried velocity, which is the phantom-approach protection
    the original guards were written for.
  * A person whose velocity is not trusted and who has no history is reported with zero
    velocity once seen for MIN_UNKNOWN_TENURE_S: present, direction unknown.
  * A trusted person who is standing still is NOT reported (lidar covers them), as before.

Pure Python, no ROS: importable by tests.  Used from server_x3.py's move().
"""
import math

RANGE_M = 1.8            # CBF_SPEED_RANGE_M: beyond this the velocity model is not trusted
MIN_SPEED = 0.15         # CBF_MOVER_MIN_SPEED
MIN_PERSON_FRAMES = 5    # CBF_MOVER_MIN_PERSON_FRAMES
JUMP_HOLD_FRAMES = 10    # CBF_MOVER_JUMP_HOLD_FRAMES

HOLD_S = 1.5             # how long a vanished mover is kept
EXTRAPOLATE_S = 0.3      # run the ghost on its velocity for this long, then freeze it
TAU_GHOST_S = 0.5        # velocity fade of a ghost
TAU_CARRY_S = 1.0        # velocity fade when carrying across an untrusted re-ID
CARRY_MAX_S = 2.0        # never carry a velocity older than this
MATCH_BASE_M = 0.5       # association gate: this + MATCH_GROW_MPS * gap, capped
MATCH_GROW_MPS = 1.5
MATCH_MAX_M = 1.5
MIN_TENURE_S = 0.5       # a person must have been seen this long to be held as a ghost
MIN_UNKNOWN_TENURE_S = 0.3
UNKNOWN_RANGE_M = 1.5    # unknown-velocity people are only reported this close


class _Entry:
    __slots__ = ("pos", "first", "last", "v_trusted", "v_trusted_t", "mover_t",
                 "est_id", "ghosted")

    def __init__(self, pos, now, est_id):
        self.pos = pos
        self.first = now
        self.last = now
        self.v_trusted = None
        self.v_trusted_t = -1e9
        self.mover_t = -1e9      # last time it was a trusted mover (speed > MIN_SPEED)
        self.est_id = est_id
        self.ghosted = False     # currently being reported as an unobserved ghost


def _fade(v, dt, tau):
    k = math.exp(-max(dt, 0.0) / tau)
    return (v[0] * k, v[1] * k)


class PersonMemory:
    def __init__(self, range_m=RANGE_M, min_speed=MIN_SPEED, hold_s=HOLD_S):
        self.range_m = range_m
        self.min_speed = min_speed
        self.hold_s = hold_s
        self.entries = []
        self.stats = {"ghost_started": 0, "relinked": 0, "new_entries": 0}

    # -- helpers -----------------------------------------------------------------
    def _observations(self, estimates):
        obs = []
        for est in estimates:
            if est.get("category") != "person" or est.get("z", 0.0) > self.range_m:
                continue
            if est.get("age", 0) > 0:        # not matched this frame
                continue
            pos = (float(est.get("z", est.get("y", 0.0))), -float(est.get("x", 0.0)))
            vel = (float(est.get("vx", 0.0)), float(est.get("vy", 0.0)))
            trusted = (est.get("person_frames", MIN_PERSON_FRAMES) >= MIN_PERSON_FRAMES
                       and est.get("jump_age", JUMP_HOLD_FRAMES) >= JUMP_HOLD_FRAMES)
            obs.append({"pos": pos, "vel": vel, "trusted": trusted,
                        "speed": float(est.get("speed", math.hypot(*vel))),
                        "id": est.get("id")})
        return obs

    def _associate(self, obs, now):
        """Greedy nearest-first matching of observations to live entries."""
        pairs = []
        for i, o in enumerate(obs):
            for j, e in enumerate(self.entries):
                gap = now - e.last
                gate = min(MATCH_MAX_M, MATCH_BASE_M + MATCH_GROW_MPS * max(gap, 0.0))
                d = math.hypot(o["pos"][0] - e.pos[0], o["pos"][1] - e.pos[1])
                if d <= gate:
                    pairs.append((d, i, j))
        pairs.sort()
        used_o, used_e, match = set(), set(), {}
        for d, i, j in pairs:
            if i in used_o or j in used_e:
                continue
            used_o.add(i)
            used_e.add(j)
            match[i] = j
        return match

    # -- main entry point --------------------------------------------------------
    def update(self, estimates, now):
        """estimates: velocity_estimator.get_estimates().  Returns the CBF mover list
        [((x, y), (vx, vy)), ...] in the base frame."""
        obs = self._observations(estimates)
        self.entries = [e for e in self.entries if now - e.last <= self.hold_s + 0.5]
        match = self._associate(obs, now)
        out = []
        observed = set()
        for i, o in enumerate(obs):
            j = match.get(i)
            if j is None:
                e = _Entry(o["pos"], now, o["id"])
                self.entries.append(e)
                self.stats["new_entries"] += 1
            else:
                e = self.entries[j]
                gap = now - e.last
                if gap > 0.25 and o["id"] != e.est_id:
                    self.stats["relinked"] += 1
                e.pos = o["pos"]
                e.last = now
                e.est_id = o["id"]
                e.ghosted = False
            observed.add(id(e))
            tenure = now - e.first
            if o["trusted"]:
                e.v_trusted, e.v_trusted_t = o["vel"], now
                if o["speed"] > self.min_speed:
                    e.mover_t = now
                    out.append((o["pos"], o["vel"]))
                # trusted and slow: a person standing still is left to the lidar
                continue
            # Velocity not trusted (new id, still gated, or just jumped).
            if e.v_trusted is not None and now - e.mover_t <= CARRY_MAX_S:
                # it was walking a moment ago: keep that velocity, fading
                out.append((o["pos"], _fade(e.v_trusted, now - e.v_trusted_t, TAU_CARRY_S)))
                continue
            if tenure >= MIN_UNKNOWN_TENURE_S and o["pos"][0] <= UNKNOWN_RANGE_M:
                out.append((o["pos"], (0.0, 0.0)))
        # ghosts: established movers that were not observed this tick
        for e in self.entries:
            if id(e) in observed:
                continue
            gap = now - e.last
            if gap > self.hold_s or e.last - e.first < MIN_TENURE_S:
                continue
            if now - e.mover_t > self.hold_s + 1.0 and e.v_trusted is not None:
                continue            # was a standing person: the lidar still has them
            v0 = e.v_trusted if e.v_trusted is not None else (0.0, 0.0)
            run = min(gap, EXTRAPOLATE_S)
            pos = (e.pos[0] + v0[0] * run, e.pos[1] + v0[1] * run)
            if not e.ghosted:
                e.ghosted = True
                self.stats["ghost_started"] += 1
            out.append((pos, _fade(v0, gap, TAU_GHOST_S)))
        return out
