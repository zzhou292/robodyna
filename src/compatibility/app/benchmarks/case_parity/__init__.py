"""Resolved-case consistency and scoped performance admission; no solver authority."""
from .contracts import compare_contracts, read_contract
from .performance import assess

__all__ = ["read_contract", "compare_contracts", "assess"]
