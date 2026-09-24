"""Best-effort CPU containment for background recording processes."""

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
    """Return background logical CPUs without restricting motion control.

    Motion control keeps the complete scheduler affinity.  On a four-core
    workstation, cutting it down to two physical cores made the host watchdog
    miss otherwise valid targets.  Only the lower-priority recording workers
    are contained on the latter half of the available physical cores.
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
    background_groups = groups[len(groups) // 2:]
    return {"background": frozenset(
        cpu for group in background_groups for cpu in group)}


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
