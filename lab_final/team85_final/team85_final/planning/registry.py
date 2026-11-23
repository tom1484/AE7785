"""Name -> planner class, so the mission node picks one with the `planner` parameter."""

from __future__ import annotations

from .base import Planner

_REGISTRY: dict[str, type[Planner]] = {}


def register(name: str):
    def deco(cls: type[Planner]) -> type[Planner]:
        cls.name = name
        _REGISTRY[name] = cls
        return cls
    return deco


def make_planner(name: str, grid, **params) -> Planner:
    # Import for registration side effects.
    from . import explore_planner, graph_planner, grid_planner  # noqa: F401

    try:
        cls = _REGISTRY[name]
    except KeyError:
        raise ValueError(f'unknown planner {name!r}; have {sorted(_REGISTRY)}') from None
    return cls(grid, **params)


def available() -> list[str]:
    from . import explore_planner, graph_planner, grid_planner  # noqa: F401
    return sorted(_REGISTRY)
