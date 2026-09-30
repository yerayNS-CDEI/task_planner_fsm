# AI model weights (not in git)

Everything in this folder except this file is ignored by git. Copy the weights
here on each machine that runs the sensor processing (robot PC / Jetson):

| Path | Used by | Delivered in |
|---|---|---|
| `gpr/best.pt` | GPR hyperbola segmentation (Mask R-CNN ResNet101, 241 MB) | `GPR_DISCOVER_robot_pipeline_v2_1/Hyperbola_Segmentation/models/best.pt` |
| `hsi/classifier.joblib` | Hyperspectral material classifier (XGBoost bundle, 27 MB) | `HYPERSPECTRAL_DISCOVER_pipeline_v2/benjamin_original/classifier.joblib` |

Override the folder with the `sensor_models_dir` ROS parameter, or a single file
with `gpr_weights_path` / `hsi_model_path`. `ros2 run task_planner_fsm
check_sensor_setup` reports what is found.
