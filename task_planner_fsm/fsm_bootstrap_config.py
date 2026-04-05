FSM_STATE_ORDER = [
    "Initialization",
    "CreateMap",
    "ObjectID",
    "GeometryReconstruction",
    "ComputeWallPoints",
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
    "Finished",
    "Error",
]

PHASE2_DEFAULT_INITIAL_STATES = {
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
    "Finished",
}

WALL_DATA_REQUIRED_INITIAL_STATES = {
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
}

PHASE1_TARGET_REQUIRED_INITIAL_STATES = {
    "NavigateToTarget",
    "ArmUnfolding",
    "ScanWall",
}

PHASE2_BASE_REQUIRED_INITIAL_STATES = {
    "NavigateToTarget",
    "ArmUnfolding",
    "ScanWall",
    "ArmFolding",
    "ExhaustiveScan",
}

NEEDS_SYNTHETIC_DISCRETIZATION_INITIAL_STATES = {
    "WallTargetSelection",
    "NavigateToTarget",
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
}

NAV_CLIENT_BOOTSTRAP_STATES = {
    "ArmUnfolding",
    "ArmFolding",
    "ScanWall",
    "AreasOfInterest",
    "WallDiscretization",
    "BasePlacement",
    "ExhaustiveScan",
    "HomePosition",
}

NAV_SIM_REQUIRED_START_STATES = {
    s
    for s in FSM_STATE_ORDER[FSM_STATE_ORDER.index("ComputeWallPoints") :]
    if s not in {"Finished", "Error"}
}

PREDEFINED_WALLS = [
    ((3.0, 0.0, 2.0), (3.0, -3.0, 3.0)),
    ((9.0, 0.0, 0.19), (9.0, -4.5, 2.0)),
    ((10.0, 0.0, 0.2), (10.0, -4.5, 3.0)),
]
