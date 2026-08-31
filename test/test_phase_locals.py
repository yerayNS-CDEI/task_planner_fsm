"""No local may cross a phase boundary in ScanWall._run_scan.

Every `_seg_phase` branch runs on a SEPARATE tick of the FSM: the state returns
after each one and `run()` is called again later. So a local bound in one branch
does not exist in the next, and reading it raises `UnboundLocalError` at runtime
— in the middle of a wall, after the base has already moved.

That is not hypothetical. Splitting the transit into
`transit -> transit_column -> transit_send` so the column could be retracted
before the base moves left `arm_sweep` bound in `transit` and read in
`transit_send`:

    UnboundLocalError: local variable 'arm_sweep' referenced before assignment

It survived every unit test and every import check, because the phases only run
in that order against a live robot. This check reads the AST instead.

Run with:

    python3 -m pytest test/test_phase_locals.py -v
"""

import ast
import os
import re

import pytest

SOURCE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "task_planner_fsm", "states", "scan_wall.py",
)
PRE_DISPATCH = "<pre-dispatch>"


def phase_of_each_line(text):
    """Map line number -> the `_seg_phase` branch it sits in."""
    mapping, current = {}, PRE_DISPATCH
    for number, line in enumerate(text.split("\n"), 1):
        match = re.match(r'\s+if self\._seg_phase == "(\w+)":', line)
        if match:
            current = match.group(1)
        mapping[number] = current
    return mapping


@pytest.fixture(scope="module")
def analysis():
    text = open(SOURCE).read()
    run_scan = next(
        node for node in ast.walk(ast.parse(text))
        if isinstance(node, ast.FunctionDef) and node.name == "_run_scan"
    )
    return text, run_scan, phase_of_each_line(text)


def cross_phase_reads(analysis):
    """(name, reading phase, assigning phases) for every local read outside the
    branch that binds it."""
    _, run_scan, phase_at = analysis
    assigned = {}
    for node in ast.walk(run_scan):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Store):
            assigned.setdefault(node.id, []).append(node.lineno)

    offenders = {}
    for node in ast.walk(run_scan):
        if not (isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load)):
            continue
        if node.id not in assigned:
            continue
        reader = phase_at.get(node.lineno)
        writers = {phase_at.get(line) for line in assigned[node.id] if line < node.lineno}
        # Bound before the dispatch = available to every branch.
        if reader in writers or PRE_DISPATCH in writers:
            continue
        offenders.setdefault((node.id, reader), sorted(w for w in writers if w))
    return offenders


def test_no_local_is_read_outside_the_phase_that_binds_it(analysis):
    offenders = cross_phase_reads(analysis)
    assert not offenders, "\n".join(
        f"  {name!r} is read in phase '{reader}' but only assigned in {writers} "
        f"-- phases run on separate ticks, so this is an UnboundLocalError"
        for (name, reader), writers in sorted(offenders.items())
    )


def test_the_checker_would_have_caught_the_transit_send_regression(analysis):
    """Guards the guard: strip the rebind and the check must fail."""
    text, _, _ = analysis
    marker = ("            # Rebind: every phase is a separate tick, so nothing "
              "bound in\n            # `transit` survives to here.\n"
              "            arm_sweep = self._use_arm_sweep(ctx)\n")
    assert marker in text, "the transit_send rebind is gone; update this test"

    broken = text.replace(marker, "", 1)
    run_scan = next(
        node for node in ast.walk(ast.parse(broken))
        if isinstance(node, ast.FunctionDef) and node.name == "_run_scan"
    )
    assert cross_phase_reads((broken, run_scan, phase_of_each_line(broken)))


def test_every_phase_branch_is_found(analysis):
    """If the branch pattern ever stops matching, the checker would pass
    vacuously -- so assert it still sees the machine."""
    _, _, phase_at = analysis
    phases = set(phase_at.values()) - {PRE_DISPATCH}
    for expected in ("transit", "transit_column", "transit_send", "sweep_wait",
                     "line_column", "line_change"):
        assert expected in phases
