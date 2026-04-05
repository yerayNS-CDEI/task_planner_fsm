from typing import Any, Dict, Optional


class BTBlackboardAdapter:
    """Adapter over FSM context to support BT-style key access incrementally."""

    def __init__(self, ctx: Dict[str, Any]):
        self._ctx = ctx

    def get(self, key: str, default: Any = None) -> Any:
        return self._ctx.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._ctx[key] = value

    def has(self, key: str) -> bool:
        return key in self._ctx

    def remove(self, key: str) -> None:
        self._ctx.pop(key, None)

    def get_bool(self, key: str, default: bool = False) -> bool:
        value = self._ctx.get(key, default)
        return bool(value)

    def get_int(self, key: str, default: int = 0) -> int:
        value = self._ctx.get(key, default)
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    def get_float(self, key: str, default: float = 0.0) -> float:
        value = self._ctx.get(key, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    def get_optional(self, key: str) -> Optional[Any]:
        return self._ctx.get(key)

    def raw(self) -> Dict[str, Any]:
        return self._ctx
