"""Manifest loading and the train/test split.

The split is the whole point of this module. Frames inside one throw are
near-duplicates -- same object, same light, 25ms apart -- so a random split
puts the same throw on both sides and reports an accuracy that means nothing.
Splitting by throw is better and still leaks: ten throws of the same Coke can
measure "do I recognise *this can*", not "is this trash".

So the split is by `object_id`, entire objects are held out, and the check is
asserted rather than remembered. `split_by_object` refuses to return a leaking
split -- there is no flag to turn that off, because the only reason to want
one is to make a number look better than it is.

Effective dataset size is the number of *tracks*, not crops. 13k crops from
450 throws is ~450 weakly-independent samples augmented 30x. Sizing a model
against the crop count will overfit.
"""

from __future__ import annotations

import json
import os
import random
from collections import Counter, defaultdict
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

TRASH = "trash"
NOT_TRASH = "not-trash"
DISTRACTOR = "distractor"

# Binary target. Distractors -- arms at release, shadows, detector false
# positives -- fold into not-trash: at run time they are exactly the things
# that must not trigger a catch.
LABEL_TO_TARGET = {TRASH: 1, NOT_TRASH: 0, DISTRACTOR: 0}


class Sample:
    __slots__ = ("crop", "frame", "label", "object_id", "session_id",
                 "block_id", "track_id", "run_id", "frame_idx", "verdict",
                 "area", "fit_residual", "root")

    def __init__(self, row: dict, root: str):
        self.root = root
        self.crop = row["crop"]
        self.frame = row.get("frame")
        self.label = row["label"]
        self.object_id = row.get("object_id") or "unknown"
        self.session_id = row.get("session_id", "")
        self.block_id = row.get("block_id", "")
        self.track_id = row.get("track_id", -1)
        self.run_id = row.get("run_id", "")
        self.frame_idx = row.get("frame_idx", -1)
        self.verdict = row.get("verdict", "")
        self.area = row.get("area", 0)
        self.fit_residual = row.get("fit_residual", float("nan"))

    @property
    def target(self) -> int:
        return LABEL_TO_TARGET[self.label]

    @property
    def crop_path(self) -> str:
        return os.path.join(self.root, self.crop)

    @property
    def track_key(self) -> Tuple[str, str, str, int]:
        """Identifies one throw globally.

        run_id is required: track ids restart at zero on every collect
        invocation, so without it separate throws of separate objects
        collide into a single apparent track.
        """
        return (self.session_id, self.block_id, self.run_id, self.track_id)

    def __repr__(self):
        return f"Sample({self.label}, {self.object_id}, track={self.track_id})"


class LeakageError(AssertionError):
    """An object appears in both train and test."""


def load_manifests(paths: Iterable[str]) -> List[Sample]:
    """Load one or more manifest.jsonl files (one per session)."""
    out: List[Sample] = []
    for path in paths:
        root = os.path.dirname(os.path.abspath(path))
        with open(path) as fh:
            for line_no, line in enumerate(fh, 1):
                line = line.strip()
                if not line:
                    continue
                try:
                    row = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_no}: {exc}") from None
                if row.get("label") not in LABEL_TO_TARGET:
                    raise ValueError(
                        f"{path}:{line_no}: unknown label {row.get('label')!r}")
                out.append(Sample(row, root))
    return out


def find_manifests(data_dir: str) -> List[str]:
    found = []
    for dirpath, _dirs, files in os.walk(data_dir):
        if "manifest.jsonl" in files:
            found.append(os.path.join(dirpath, "manifest.jsonl"))
    return sorted(found)


def objects_by_label(samples: Sequence[Sample]) -> Dict[str, List[str]]:
    out: Dict[str, set] = defaultdict(set)
    for s in samples:
        out[s.label].add(s.object_id)
    return {k: sorted(v) for k, v in out.items()}


def summarize(samples: Sequence[Sample]) -> dict:
    tracks = {s.track_key for s in samples}
    return {
        "crops": len(samples),
        "tracks": len(tracks),
        "objects": len({s.object_id for s in samples}),
        "sessions": len({s.session_id for s in samples}),
        "by_label": dict(Counter(s.label for s in samples)),
        "by_target": dict(Counter(s.target for s in samples)),
    }


def cap_distractors(samples: Sequence[Sample], max_fraction: float = 0.40,
                    seed: int = 0) -> List[Sample]:
    """Limit distractors to a fraction of the set.

    They come free and will otherwise swamp both real classes, teaching the
    model that the answer is almost always "no".
    """
    keep = [s for s in samples if s.label != DISTRACTOR]
    distractors = [s for s in samples if s.label == DISTRACTOR]
    if not keep:
        return list(distractors)
    budget = int(len(keep) * max_fraction / max(1e-9, 1.0 - max_fraction))
    if len(distractors) > budget:
        rng = random.Random(seed)
        # Drop whole tracks, not random crops: keeping 3 frames of one arm
        # gesture adds nothing over keeping all 30.
        by_track = defaultdict(list)
        for s in distractors:
            by_track[s.track_key].append(s)
        tracks = list(by_track)
        rng.shuffle(tracks)
        distractors = []
        for t in tracks:
            if len(distractors) >= budget:
                break
            distractors.extend(by_track[t])
    return keep + distractors


def assert_no_leakage(train: Sequence[Sample], test: Sequence[Sample]) -> None:
    tr = {s.object_id for s in train}
    te = {s.object_id for s in test}
    both = tr & te
    if both:
        raise LeakageError(
            f"{len(both)} object(s) in both train and test: {sorted(both)[:5]}"
            " -- accuracy from this split is meaningless"
        )
    tr_tracks = {s.track_key for s in train}
    te_tracks = {s.track_key for s in test}
    if tr_tracks & te_tracks:
        raise LeakageError("the same track appears on both sides")


def split_by_object(samples: Sequence[Sample],
                    holdout: Optional[Sequence[str]] = None,
                    n_holdout_per_class: int = 5,
                    seed: int = 0) -> Tuple[List[Sample], List[Sample]]:
    """Hold out whole objects. Always verifies the result before returning.

    `holdout` pins an explicit object list (use it to keep the test set fixed
    across experiments). Otherwise objects are drawn per class so both classes
    are represented on both sides.

    Distractors follow their object where one exists; those without a real
    object id stay in train, since they are negatives rather than a class
    whose generalisation is being measured.
    """
    rng = random.Random(seed)

    if holdout is None:
        chosen: List[str] = []
        for label in (TRASH, NOT_TRASH):
            objs = sorted({s.object_id for s in samples if s.label == label})
            if len(objs) <= n_holdout_per_class:
                raise ValueError(
                    f"only {len(objs)} '{label}' object(s); need more than "
                    f"{n_holdout_per_class} to hold any out. Collect more "
                    "objects rather than lowering the holdout."
                )
            picked = rng.sample(objs, n_holdout_per_class)
            chosen.extend(picked)
        holdout = chosen

    held = set(holdout)
    train = [s for s in samples if s.object_id not in held]
    test = [s for s in samples if s.object_id in held]

    if not test:
        raise ValueError(f"holdout {sorted(held)} matched no samples")
    test_targets = {s.target for s in test}
    if len(test_targets) < 2:
        raise ValueError(
            "test set has only one class -- it cannot measure discrimination"
        )

    assert_no_leakage(train, test)
    return train, test


def describe_split(train: Sequence[Sample], test: Sequence[Sample]) -> str:
    a, b = summarize(train), summarize(test)
    lines = [
        f"  train: {a['crops']:6d} crops  {a['tracks']:4d} tracks  "
        f"{a['objects']:3d} objects  {a['by_label']}",
        f"  test:  {b['crops']:6d} crops  {b['tracks']:4d} tracks  "
        f"{b['objects']:3d} objects  {b['by_label']}",
        f"  held-out objects: {sorted({s.object_id for s in test})}",
        f"  effective train size is ~{a['tracks']} throws, not {a['crops']} crops",
    ]
    return "\n".join(lines)
