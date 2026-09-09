from ..state import State
from example_interfaces.srv import SetBool

import subprocess, os, time, socket
import rclpy
import rclpy.time
from rclpy.duration import Duration

from task_planner_fsm.states.proc_utils import start_proc
from ..utils import hyperspectral_processing as hp

from arm_control.srv import PredictMaterial


class SensorDataProcessing(State):
    def __init__(self, name):
        super().__init__(name)
        self.client = None
        self.future = None
        self.ml_client = None

    def on_enter(self, ctx):
        node = ctx["node"]
        # Hyperspectral first: it is local file work, it cannot fail the state,
        # and doing it here means the sweep's raw record is turned into
        # reflectance before anything downstream looks for results.
        self._process_hyperspectral(ctx)
        node.get_logger().info(f"[{self.name}] Calling the service /sensor_data_processing")
        ctx["data_processed"] = False
        ctx["drilling_required"] = False
        ctx["error_triggered"] = False

        self.client = node.create_client(SetBool, "/sensor_data_processing")
        request = SetBool.Request()
        request.data = True
        
        if not self.client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error(f"[{self.name}] Service /sensor_data_processing not available.")
            ctx["error_triggered"] = True
            return
        
        self.future = self.client.call_async(request)

    def run(self, ctx):
        node = ctx["node"]
        self.set_activity(ctx, "Processing the scan sensor data")

        if self.future is None:
            node.get_logger().info(f"[{self.name}] Future is None.")
            return
        
        if self.future.done():
            result = self.future.result()
            if result and result.success:
                node.get_logger().info(f"[{self.name}] Sensor data processed correctly.")
                ctx["data_processed"] = True
                ctx["drilling_required"] = True
            else:
                node.get_logger().error(f"[{self.name}] Error while processing sensor data.")
                ctx["error_triggered"] = True
            self.future = None

    def check_transition(self, ctx):
        if ctx.get("error_triggered"):
            return "Error"
        if not ctx.get("data_processed"):
            return None
        if ctx.get("drilling_required"):
            return "SendDataToPokeye"
        return "ArmFolding"

    # ------------------------------------------------------------------
    # Hyperspectral post-processing
    # ------------------------------------------------------------------
    def _process_hyperspectral(self, ctx):
        """Turn the sweep's raw spectra into reflectance, labels and metrics.

        This is the second half of the collect/process split: ScanWall recorded
        raw GSM counts and the session's GDS/GRF while the robot moved, and
        every number produced here is a pure function of that record. Nothing
        touches the camera, so a camera that has since been switched off, or a
        mission being re-processed days later with a retrained model, both work.

        Deliberately non-fatal. The GPR is the primary sensor and the drilling
        decision downstream does not depend on material labels, so a failure
        here is logged and the state carries on. The raw record is already on
        disk and can be re-processed by hand at any time -- losing the report is
        recoverable, losing the wall is not.
        """
        node = ctx["node"]
        session_dir = ctx.get("hyperspectral_session_dir")
        if not session_dir:
            return          # sampling was disabled, or no sweep recorded one
        session_dir = os.path.expanduser(str(session_dir))

        self.set_activity(ctx, "Processing the hyperspectral sweep samples",
                          publish=True)
        try:
            result = hp.process_session(
                session_dir,
                predict_fn=self._make_predict_fn(ctx),
                logger=node.get_logger(),
            )
        except Exception as exc:            # noqa: BLE001
            node.get_logger().error(
                f"[{self.name}] hyperspectral processing failed: {exc}. The raw "
                f"record in {session_dir} is intact and can be re-processed."
            )
            ctx["hyperspectral_processed"] = False
            return

        metrics = result["metrics"]
        totals = metrics.totals()
        ctx["hyperspectral_processed"] = True
        ctx["hyperspectral_reflectance_csv"] = result["reflectance_csv"]
        ctx["hyperspectral_metrics_json"] = result["metrics_json"]
        ctx["hyperspectral_totals"] = totals

        node.get_logger().info(f"[{self.name}] {metrics.summary_line('mission')}")
        for wall, wall_totals in sorted(metrics.walls().items()):
            node.get_logger().info(
                f"[{self.name}]   wall {wall}: "
                f"{wall_totals['triggered']} points triggered, "
                f"{wall_totals['captured']} captured, "
                f"{wall_totals['capture_failed']} capture failures, "
                f"{wall_totals['skipped']} skipped, "
                f"{wall_totals[hp.ACCEPTED]} accepted, "
                f"{wall_totals['rejected']} rejected "
                f"(yield {wall_totals['yield'] * 100.0:.0f}%)"
            )
        reasons = metrics.reject_reasons()
        if reasons:
            node.get_logger().info(
                f"[{self.name}]   rejections: "
                + ", ".join(f"{k} x{v}" for k, v in reasons.items())
            )
        node.get_logger().info(
            f"[{self.name}] hyperspectral report: {result['reflectance_csv']}"
        )

    def _make_predict_fn(self, ctx):
        """Return a ``predict_fn(vis, nir)`` for the ML service, or None.

        Blocking is fine HERE and nowhere else in this integration: the robot is
        stationary, no sweep is in flight and nothing is being timed. That is
        exactly what makes deferring the prediction worth it.

        Returns None when the service is absent, so a mission without the ML
        node still produces reflectance and the full coverage metrics -- the
        labels can be added later from the same raw file.
        """
        node = ctx["node"]
        if not bool(ctx.get("hyperspectral_predict_material", True)):
            return None
        if self.ml_client is None:
            self.ml_client = node.create_client(
                PredictMaterial,
                str(ctx.get("hyperspectral_ml_service",
                            "hyperspectral/predict_material")),
            )
        timeout = float(ctx.get("hyperspectral_ml_timeout_s", 5.0))
        if not self.ml_client.wait_for_service(timeout_sec=timeout):
            node.get_logger().warn(
                f"[{self.name}] ML service unavailable; writing reflectance and "
                f"metrics without material labels."
            )
            return None

        def predict(vis, nir):
            request = PredictMaterial.Request()
            # float32[] on the wire; the model casts to float64 internally.
            request.vis_spectrum = [float(v) for v in vis]
            request.nir_spectrum = [float(v) for v in nir]
            future = self.ml_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            response = future.result()
            if response is None:
                return "ERROR: no response", 0.0
            return response.material, float(response.confidence)

        return predict
