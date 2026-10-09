"""COLREG situation geometry (International Regulations for Preventing Collisions at Sea 1972, Rules 13-17) for two vessels in sight of one another.

Pure functions of positions and velocities (ENU, m and m/s) and headings (math yaw, rad); no state except the optional previous class for hysteresis.
Sectors, from the Rules:
  Rule 13 overtaking   "coming up with another vessel from a direction more than 22.5 degrees abaft her beam", i.e. the overtaker sees the other
                       vessel within the 135 degree arc on the overtaken vessel's stern: |bearing of the overtaker from the other's bow| > 112.5 deg.
                       The overtaking vessel keeps out of the way (give-way); the vessel being overtaken stands on.
  Rule 14 head-on      "reciprocal or nearly reciprocal courses": each sees the other (nearly) dead ahead; both alter course to starboard.
  Rule 15 crossing     "two power-driven vessels are crossing so as to involve risk of collision, the vessel which has the other on her own starboard side
                       shall keep out of the way and shall, if the circumstances of the case admit, avoid crossing ahead of the other vessel."
  Rule 16 give-way     "take early and substantial action to keep well clear".
  Rule 17 stand-on     keep course and speed (a); may act when it becomes apparent that the give-way vessel is not taking appropriate action (a)(ii);
                       must act when collision cannot be avoided by the give-way vessel alone (b); and should, if the circumstances admit, not alter
                       course to port for a vessel on her own port side (c).
The numerical thresholds (the 15 degree head-on cone, the hysteresis) are the usual implementation choices of the collision-avoidance literature, not text of the Rules.
"""

import math
from dataclasses import dataclass

import numpy as np

from .geo import rel_bearing

# situation classes
NONE = "none"  # no obligation: diverging, or already past
HEAD_ON = "head_on"  # Rule 14: both alter to starboard
OVERTAKING = "overtaking"  # Rule 13: we overtake, we give way
OVERTAKEN = "overtaken"  # Rule 13: we are overtaken, we stand on
GIVE_WAY = "crossing_give_way"  # Rule 15: target on our starboard side, we give way
STAND_ON = "crossing_stand_on"  # Rule 15/17: target on our port side, we stand on

HEAD_ON_CONE = math.radians(15.0)
ABEAM_ARC = math.radians(112.5)  # 90 deg + 22.5 deg: the edge of the stern sector
HYST = math.radians(
    4.0
)  # a class holds until the bearing is this far inside the neighbouring sector


@dataclass
class Geometry:
    """Relative geometry of a contact, from own vessel."""

    range: float  # m, centre to centre
    bearing: float  # rad, target from own bow, + starboard
    aspect: float  # rad, own from the target's bow (course), + starboard of the target
    cpa: float  # m, closest point of approach (>= 0)
    tcpa: float  # s, time to it (negative: already past)
    closing: float  # m/s, rate at which the range decreases (negative: opening)


def cpa_tcpa(p_own, v_own, p_tgt, v_tgt):
    """-> (cpa, tcpa) of two constant-velocity points; tcpa < 0 when the closest point is behind us, then cpa is the present range."""
    r = np.asarray(p_tgt, float) - np.asarray(p_own, float)
    v = np.asarray(v_tgt, float) - np.asarray(v_own, float)
    vv = float(v @ v)
    if vv < 1e-9:
        return float(np.hypot(*r)), math.inf if np.hypot(*r) > 0 else 0.0
    t = -float(r @ v) / vv
    if t <= 0:
        return float(np.hypot(*r)), t
    return float(np.hypot(*(r + v * t))), t


def geometry(own_pos, own_vel, own_psi, tgt_pos, tgt_vel):
    """Relative geometry. The target's course is that of its velocity vector (COG, as AIS gives it); a stopped target keeps aspect 0."""
    r = np.asarray(tgt_pos, float) - np.asarray(own_pos, float)
    rng = float(np.hypot(*r))
    tv = np.asarray(tgt_vel, float)
    tpsi = math.atan2(tv[1], tv[0]) if np.hypot(*tv) > 0.1 else math.atan2(r[1], r[0])
    cpa, tcpa = cpa_tcpa(own_pos, own_vel, tgt_pos, tgt_vel)
    dv = tv - np.asarray(own_vel, float)
    closing = -float(r @ dv) / rng if rng > 1e-9 else 0.0
    return Geometry(
        rng,
        rel_bearing(own_psi, own_pos, tgt_pos),
        rel_bearing(tpsi, tgt_pos, own_pos),
        cpa,
        tcpa,
        closing,
    )


def classify(g: Geometry, own_speed: float, tgt_speed: float, prev: str = NONE) -> str:
    """The Rule that applies between us and the target, from the geometry alone (call it for contacts with risk of collision).

    prev: the class of the previous call for this contact; a class is kept while the bearing is within HYST of the sector edge it was born in
    (a contact on the 112.5 degree line would otherwise flip between crossing and overtaking at every sensor tick)."""
    b, a = g.bearing, g.aspect
    ab, aa = abs(b), abs(a)
    h = HYST if prev in (OVERTAKING, OVERTAKEN) else 0.0
    # Rule 13: we are in the target's stern sector and it is not in ours: we come up on it. The speeds are not tested: the caller only classifies contacts with
    # risk of collision (a closing contact), so a vessel astern of the other that is closing on it is the overtaker by definition.
    if aa > ABEAM_ARC - h and ab < ABEAM_ARC:
        return OVERTAKING
    # Rule 13: the target is in our stern sector: it comes up on us, we are overtaken and stand on
    h = HYST if prev == OVERTAKEN else 0.0
    if ab > ABEAM_ARC - h and aa < ABEAM_ARC:
        return OVERTAKEN
    # Rule 14: reciprocal courses, each ahead of the other
    cone = HEAD_ON_CONE + (HYST if prev == HEAD_ON else 0.0)
    if ab <= cone and aa <= cone:
        return HEAD_ON
    # Rule 15: crossing; starboard side gives way
    return GIVE_WAY if b >= 0 else STAND_ON


def role(cls: str) -> str:
    """give_way | stand_on | none for a class."""
    return {
        HEAD_ON: "give_way",
        OVERTAKING: "give_way",
        GIVE_WAY: "give_way",
        OVERTAKEN: "stand_on",
        STAND_ON: "stand_on",
    }.get(cls, "none")


def passes_astern(own_pos, own_vel, tgt_pos, tgt_vel):
    """True when, at the closest point of approach, we are in the target's stern half (it crossed ahead of us if False for a crossing)."""
    cpa, t = cpa_tcpa(own_pos, own_vel, tgt_pos, tgt_vel)
    t = max(t, 0.0)
    po = np.asarray(own_pos) + np.asarray(own_vel) * t
    pt = np.asarray(tgt_pos) + np.asarray(tgt_vel) * t
    tv = np.asarray(tgt_vel, float)
    if np.hypot(*tv) < 1e-6:
        return True
    return float((po - pt) @ tv) < 0.0


def passes_port_to_port(own_pos, own_vel, tgt_pos, tgt_vel):
    """True when the target is on our port side at the closest point (the head-on result Rule 14 asks for)."""
    cpa, t = cpa_tcpa(own_pos, own_vel, tgt_pos, tgt_vel)
    t = max(t, 0.0)
    r = (np.asarray(tgt_pos) + np.asarray(tgt_vel) * t) - (
        np.asarray(own_pos) + np.asarray(own_vel) * t
    )
    ov = np.asarray(own_vel, float)
    return (
        float(ov[0] * r[1] - ov[1] * r[0]) > 0.0
    )  # target to the left of our velocity
