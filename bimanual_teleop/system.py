"""Lifecycle contract for the combined control runtime."""

from enum import Enum


class SystemState(str, Enum):
    DISCONNECTED = "disconnected"
    READY = "ready"
    ENGAGED = "engaged"
    PAUSED = "paused"
    HOMING = "homing"
    FAULT = "fault"
    CLOSED = "closed"
