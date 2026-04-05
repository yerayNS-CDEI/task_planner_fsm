import py_trees

from task_planner_fsm.bt.adapter import BtStateAdapter
from task_planner_fsm.states import (
    GeometryReconstruction,
    Initialization,
    ObjectID,
)


def build_initial_bt_tree(ctx):
    root = py_trees.composites.Sequence(name="task_planner_root", memory=True)

    initialization = BtStateAdapter(
        name="Initialization",
        state=Initialization("Initialization"),
        ctx=ctx,
        success_transition="CreateMap",
    )
    object_id = BtStateAdapter(
        name="ObjectID",
        state=ObjectID("ObjectID"),
        ctx=ctx,
        success_transition="GeometryReconstruction",
    )
    geometry_reconstruction = BtStateAdapter(
        name="GeometryReconstruction",
        state=GeometryReconstruction("GeometryReconstruction"),
        ctx=ctx,
        success_transition="ComputeWallPoints",
    )

    root.add_children(
        [
            py_trees.decorators.Retry(name="RetryInit", child=initialization, num_failures=1),
            py_trees.decorators.Retry(name="RetryObjectId", child=object_id, num_failures=1),
            py_trees.decorators.Retry(
                name="RetryGeometryReconstruction", child=geometry_reconstruction, num_failures=1
            ),
        ]
    )
    return py_trees.trees.BehaviourTree(root=root)
