"""Candidate-level cross-class mutex: the entry with more observations wins.

Pure-function tests for semantic_map_memory_node's mutex rules:
  * landmark-level rule (unchanged): a different-class landmark within the
    mutex distance rejects the observation;
  * candidate-level rule (D2-a): candidates never reject each other's
    observations; a candidate is promoted only if it strictly out-counts every
    different-class candidate within the mutex distance.

Run with:  python3 -m pytest src/tb3_coordinator/test -q
"""
import math

import pytest

from tb3_coordinator.semantic_map_memory_node import (
    Candidate,
    Landmark,
    cross_class_rivals,
    landmark_mutex_blocks,
    may_promote,
)

DIST = 0.6      # cross_class_mutex_distance_m
MIN_OBS = 3     # mutex_min_observation_count
PERSON_MIN = 8
TRASH_MIN = 12


def cand(cls, n, x=0.6, y=-2.2):
    return Candidate(semantic_class=cls, x=x, y=y, obs_count=n, last_seen=0.0)


# ── candidate vs candidate: compare n at promotion ───────────────────────────

def test_rival_with_more_observations_holds_promotion():
    person = cand("person", 8)
    trash = cand("trash_can", 9)
    ok, rival = may_promote(person, [person, trash], DIST, MIN_OBS)
    assert ok is False and rival is trash


def test_rival_with_equal_observations_holds_promotion():
    person = cand("person", 8)
    trash = cand("trash_can", 8)
    ok, rival = may_promote(person, [person, trash], DIST, MIN_OBS)
    assert ok is False and rival is trash


def test_rival_with_fewer_observations_does_not_hold():
    person = cand("person", 8)
    trash = cand("trash_can", 7)
    ok, rival = may_promote(person, [person, trash], DIST, MIN_OBS)
    assert ok is True and rival is trash          # reported, not blocking


def test_rival_below_min_obs_is_ignored():
    person = cand("person", 8)
    trash = cand("trash_can", MIN_OBS - 1)
    ok, rival = may_promote(person, [person, trash], DIST, MIN_OBS)
    assert ok is True and rival is None


def test_rival_outside_mutex_distance_is_ignored():
    person = cand("person", 8)
    far = cand("trash_can", 50, x=0.6 + DIST + 0.05)
    ok, rival = may_promote(person, [person, far], DIST, MIN_OBS)
    assert ok is True and rival is None


def test_same_class_candidates_are_not_rivals():
    a = cand("person", 8)
    b = cand("person", 20, x=0.7)
    ok, rival = may_promote(a, [a, b], DIST, MIN_OBS)
    assert ok is True and rival is None


def test_strongest_rival_is_the_one_compared():
    person = cand("person", 8)
    weak = cand("trash_can", 4)
    strong = cand("chair", 9, x=0.5)
    ok, rival = may_promote(person, [person, weak, strong], DIST, MIN_OBS)
    assert ok is False and rival is strong


# ── candidate vs candidate: feed time never rejects ──────────────────────────

def test_candidates_never_reject_observations_at_feed_time():
    # A trash_can candidate at n=6 sits on the person; a person observation
    # arrives.  Landmark-level rule: no landmark, so not blocked.  The old
    # candidate-level drop is gone: the rival is only reported.
    trash = cand("trash_can", 6)
    assert landmark_mutex_blocks("person", 0.6, -2.2, [], DIST) is None
    lm, rival = cross_class_rivals("person", 0.6, -2.2, [], [trash], DIST, MIN_OBS)
    assert lm is None and rival is trash


# ── landmark-level rule: unchanged ───────────────────────────────────────────

def test_landmark_of_other_class_blocks_regardless_of_counts():
    lm = Landmark(landmark_id="trash_can_0", semantic_class="trash_can",
                  x=0.6, y=-2.2, observation_count=1)
    assert landmark_mutex_blocks("person", 0.62, -2.18, [lm], DIST) is lm


def test_landmark_of_same_class_does_not_block():
    lm = Landmark(landmark_id="person_0", semantic_class="person",
                  x=0.6, y=-2.2, observation_count=30)
    assert landmark_mutex_blocks("person", 0.62, -2.18, [lm], DIST) is None


def test_landmark_outside_distance_does_not_block():
    lm = Landmark(landmark_id="trash_can_0", semantic_class="trash_can",
                  x=0.6 + DIST + 0.05, y=-2.2, observation_count=100)
    assert landmark_mutex_blocks("person", 0.6, -2.2, [lm], DIST) is None


# ── the A·2 sequence ─────────────────────────────────────────────────────────
#
# 2026-09-03 04:42-04:44 (run A·2): a person candidate at n=7 dropped every
# trash_can observation on the person; it expired at 45 s; the trash_can
# candidate that then formed (n=4-6) dropped every person observation for the
# rest of the visit and the person was never promoted.  Replay that timing
# through the new rules with a minimal feed/promote loop that mirrors
# _obs_cb's order: landmark rule -> feed same-class candidate -> may_promote.

class _Memory:
    def __init__(self):
        self.landmarks = []
        self.candidates = []

    def observe(self, cls, min_obs, x=0.6, y=-2.2):
        if landmark_mutex_blocks(cls, x, y, self.landmarks, DIST) is not None:
            return "blocked_by_landmark"
        same = [c for c in self.candidates
                if c.semantic_class == cls and math.hypot(c.x - x, c.y - y) < 1.5]
        if same:
            c = same[0]
            c.obs_count += 1
        else:
            c = cand(cls, 1, x, y)
            self.candidates.append(c)
        if c.obs_count >= min_obs:
            ok, _ = may_promote(c, self.candidates, DIST, MIN_OBS)
            if ok:
                self.candidates.remove(c)
                self.landmarks.append(Landmark(
                    landmark_id="%s_%d" % (cls, len(self.landmarks)),
                    semantic_class=cls, x=c.x, y=c.y,
                    observation_count=c.obs_count))
                return "promoted"
            return "held"
        return "fed"

    def expire(self, cls):
        self.candidates = [c for c in self.candidates if c.semantic_class != cls]


def test_a2_sequence_person_is_promoted_and_trash_can_is_suppressed():
    m = _Memory()
    for _ in range(7):                       # person candidate reaches n=7
        assert m.observe("person", PERSON_MIN) == "fed"
    for _ in range(3):                       # trash_can observations on the person
        assert m.observe("trash_can", TRASH_MIN) == "fed"   # no longer dropped
    m.expire("person")                       # the 45 s timeout, as in A·2
    for _ in range(3):                       # trash_can candidate grows to n=6
        assert m.observe("trash_can", TRASH_MIN) == "fed"
    # The person comes back into view: under the old rule every one of these
    # was dropped by the trash_can candidate (n=6 >= 3).  Now they count.
    results = [m.observe("person", PERSON_MIN) for _ in range(PERSON_MIN)]
    assert results[-1] == "promoted", results
    assert [lm.semantic_class for lm in m.landmarks] == ["person"]
    # and from here on the landmark-level rule suppresses the trash_can side
    assert m.observe("trash_can", TRASH_MIN) == "blocked_by_landmark"


def test_a2_sequence_trash_can_cannot_be_promoted_over_a_bigger_person_candidate():
    m = _Memory()
    for _ in range(11):
        m.observe("trash_can", TRASH_MIN)    # n=11
    for _ in range(12):
        m.observe("person", PERSON_MIN)      # person promoted at 8 (11 < 8? no)
    # person reached 8 while trash_can had 11 >= 8 -> held; person keeps counting
    # and overtakes at 12 > 11.
    assert [lm.semantic_class for lm in m.landmarks] == ["person"]
    assert m.observe("trash_can", TRASH_MIN) == "blocked_by_landmark"


def test_bigger_rival_of_wrong_class_still_wins_when_it_is_genuinely_bigger():
    # If the wrong label is genuinely observed more, it wins, as before: the
    # rule is "more observations", not "preferred class".  This documents the
    # residual risk rather than hiding it.
    m = _Memory()
    for _ in range(12):
        m.observe("trash_can", TRASH_MIN, x=-1.3, y=2.4)
    for _ in range(5):
        m.observe("chair", 12, x=-1.3, y=2.4)
    assert [lm.semantic_class for lm in m.landmarks] == ["trash_can"]
