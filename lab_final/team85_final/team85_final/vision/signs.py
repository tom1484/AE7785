"""Sign classes and what each one tells the robot to do."""

from __future__ import annotations

from enum import IntEnum


class SignClass(IntEnum):
    """Ids match the Lab 6 labels.txt convention and SignDetection.msg constants."""

    EMPTY = 0
    LEFT = 1
    RIGHT = 2
    DO_NOT_ENTER = 3
    STOP = 4
    GOAL = 5

    @property
    def quarter_turns(self) -> int | None:
        """Turn the sign commands (+1 left, -1 right, 2 around); None for GOAL/EMPTY."""
        return _TURNS.get(self)

    @property
    def is_turn(self) -> bool:
        return self.quarter_turns is not None

    @staticmethod
    def from_name(name: str) -> SignClass:
        """Detector class name -> SignClass ('left', 'do_not_enter', 'goal', ...)."""
        key = name.strip().upper().replace(' ', '_').replace('-', '_')
        return SignClass[_ALIASES.get(key, key)]


_TURNS = {
    SignClass.LEFT: 1,
    SignClass.RIGHT: -1,
    SignClass.DO_NOT_ENTER: 2,
    SignClass.STOP: 2,
}

_ALIASES = {'DNE': 'DO_NOT_ENTER', 'DONOTENTER': 'DO_NOT_ENTER', 'U_TURN': 'DO_NOT_ENTER',
            'STAR': 'GOAL', 'NONE': 'EMPTY'}
