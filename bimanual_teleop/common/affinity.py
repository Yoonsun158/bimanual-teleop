"""Best-effort physical-core partitioning for recording and control processes."""

import os
from pathlib import Path


def _physical_groups(allowed):
    groups = {}
    for cpu in sorted(allowed):
        root = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
        try:
            package = int((root / "physical_package_id").read_text().strip())
            core = int((root / "core_id").read_text().strip())
        except (OSError, ValueError):
            return []
        groups.setdefault((package, core), []).append(cpu)
    return [tuple(cpus) for _key, cpus in sorted(groups.items())]


def recording_cpu_sets():
    """Return disjoint control/background logical CPUs or ``None``.

    The split is enabled only when at least four physical cores are available.
    This prevents a container CPU mask or an unusual topology from being
    interpreted as the development workstation's 4-core/8-thread layout.
    """
    if not hasattr(os, "sched_getaffinity"):
        return None
    try:
        allowed = set(os.sched_getaffinity(0))
    except OSError:
        return None
    groups = _physical_groups(allowed)
    if len(groups) < 4:
        return None
    control_groups = groups[:2]
    background_groups = groups[2:]
    return {
        "control": frozenset(cpu for group in control_groups for cpu in group),
        "background": frozenset(cpu for group in background_groups for cpu in group),
    }


def apply_recording_affinity(role):
    """Apply the validated partition and return the chosen CPUs."""
    sets = recording_cpu_sets()
    if sets is None or role not in sets or not hasattr(os, "sched_setaffinity"):
        return None
    cpus = sets[role]
    try:
        os.sched_setaffinity(0, cpus)
    except OSError:
        return None
    return tuple(sorted(cpus))
