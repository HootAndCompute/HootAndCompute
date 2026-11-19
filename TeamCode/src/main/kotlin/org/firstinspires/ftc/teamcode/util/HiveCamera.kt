package org.firstinspires.ftc.teamcode.util

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.concurrent.TimeUnit
import kotlin.math.hypot
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection

/**
 * [HiveCamera] runs an [AprilTagProcessor] on a webcam and reports the distance to the nearest
 * BioBuzz scoring hive.
 *
 * The SDK's BioBuzz tag library groups the tags under each hive into a *cluster* of four 3.25" tags:
 * - RED SCORING   → tags 30–33
 * - RED AUDIENCE  → tags 34–37
 * - BLUE AUDIENCE → tags 38–41
 * - BLUE SCORING  → tags 42–45
 *
 * A cluster detection (pose of the whole group) is preferred because it is steadier; if only a
 * single tag of an accepted hive is visible, that tag is used instead.
 *
 * Auto-exposure is turned off: once the [VisionPortal] is streaming, the [ExposureControl] is
 * switched to Manual and [EXPOSURE_MS] / [GAIN] are applied. This is attempted (bounded wait) at
 * construction and retried from [update] until it succeeds.
 */
@Config
class HiveCamera(hardwareMap: HardwareMap) {
    companion object {
        // TODO(fill in): name of the webcam in the Robot Controller configuration. A webcam is a
        //  USB device, so there is no port number — whatever USB port it's plugged into, it must be
        //  added under the Control Hub's USB devices in the RC config with this exact name.
        const val WEBCAM_NAME = "Webcam 1"

        // TODO(test): manual exposure (ms) and gain. Short exposure (≈ 3–8 ms) reduces motion blur
        //  while driving; raise gain until tags are bright enough. Live-tunable from Dashboard;
        //  press START on gamepad 2 in Wind to re-apply after changing them.
        @JvmField var EXPOSURE_MS = 6
        @JvmField var GAIN = 230

        // TODO(test): which hives count as targets. Names come from the SDK BioBuzz library.
        @JvmField var ACCEPT_RED_SCORING = true
        @JvmField var ACCEPT_BLUE_SCORING = true
        @JvmField var ACCEPT_RED_AUDIENCE = true
        @JvmField var ACCEPT_BLUE_AUDIENCE = true

        /** Detections older than this are treated as "not seen" (camera stalled / dropped frame). */
        @JvmField var MAX_DETECTION_AGE_MS = 250.0

        // TODO(fill in once the webcam arrives): capture resolution. 640x480 is the safest default;
        //  it must be a resolution the webcam supports AND has a calibration for, or distances will
        //  be off (consistently off is OK because the velocity table is measured empirically).
        private const val RESOLUTION_WIDTH = 640
        private const val RESOLUTION_HEIGHT = 480

        private const val STARTUP_WAIT_MS = 2000L
    }

    /** A visible hive and where it is relative to the camera. */
    data class HiveTarget(
        val name: String,
        /** Horizontal (floor) distance from the camera to the hive tags, in inches. */
        val distance: Double,
        /** Degrees left(+)/right(-) of the camera's center line. */
        val bearing: Double,
        /** Straight-line 3D distance (includes tag height), in inches — for reference only. */
        val range: Double,
    )

    private val aprilTag: AprilTagProcessor =
        AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setDrawTagOutline(true)
            .setDrawTagID(true)
            .setDrawAxes(true)
            // TODO(fill in once calibrated): if the webcam has no built-in SDK calibration, add
            //  .setLensIntrinsics(fx, fy, cx, cy) here (from a calibration tool, e.g. 3DF Zephyr).
            .build()

    val visionPortal: VisionPortal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, WEBCAM_NAME))
            .setCameraResolution(Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .enableLiveView(true)
            .addProcessor(aprilTag)
            .build()

    var isExposureApplied = false
        private set

    init {
        // Live feed on FTC Dashboard as well as the Driver Station camera stream.
        FtcDashboard.getInstance()?.startCameraStream(visionPortal, 0.0)

        // Give the camera a moment to start so manual exposure is set before START.
        val deadline = System.currentTimeMillis() + STARTUP_WAIT_MS
        while (
            visionPortal.cameraState != VisionPortal.CameraState.STREAMING &&
                System.currentTimeMillis() < deadline
        ) {
            sleepMs(50)
        }
        tryApplyManualExposure()
    }

    /**
     * Turns off auto-exposure and applies [EXPOSURE_MS] / [GAIN]. Only works once the portal is
     * STREAMING.
     *
     * @return true if the settings were applied
     */
    fun tryApplyManualExposure(): Boolean {
        if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) return false

        val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
        val gainControl = visionPortal.getCameraControl(GainControl::class.java)
        if (exposureControl == null || gainControl == null) return false

        if (exposureControl.mode != ExposureControl.Mode.Manual) {
            exposureControl.mode = ExposureControl.Mode.Manual
            sleepMs(50)
        }

        val minExposure = exposureControl.getMinExposure(TimeUnit.MILLISECONDS)
        val maxExposure = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS)
        exposureControl.setExposure(
            EXPOSURE_MS.toLong().coerceIn(minExposure, maxExposure),
            TimeUnit.MILLISECONDS,
        )
        sleepMs(20)
        gainControl.setGain(GAIN.coerceIn(gainControl.minGain, gainControl.maxGain))
        sleepMs(20)

        isExposureApplied = exposureControl.mode == ExposureControl.Mode.Manual
        return isExposureApplied
    }

    /** Call every loop: retries manual exposure until it sticks and reports camera status. */
    context(telemetry: Telemetry)
    fun update() {
        if (!isExposureApplied) tryApplyManualExposure()
        telemetry.addData("Camera", visionPortal.cameraState)
        telemetry.addData("Camera FPS", "%.1f".format(visionPortal.fps))
        telemetry.addData("Manual Exposure", if (isExposureApplied) "✓ ${EXPOSURE_MS}ms / gain $GAIN" else "pending")
    }

    /**
     * @return the closest accepted hive in the most recent frame, or null if none is visible
     */
    fun getClosestHive(): HiveTarget? {
        val now = System.nanoTime()
        val fresh =
            aprilTag.detections.filter {
                it.ftcPose != null &&
                    (now - it.frameAcquisitionNanoTime) / 1e6 <= MAX_DETECTION_AGE_MS
            }

        val clusters =
            fresh.filterIsInstance<AprilTagClusterDetection>().filter {
                isAcceptedHive(it.metadata?.name)
            }
        val singles =
            fresh.filterIsInstance<AprilTagSingleDetection>().filter {
                isAcceptedHive(hiveNameForTag(it.id))
            }

        val best: Pair<String, AprilTagDetection> =
            clusters.minByOrNull { floorDistance(it) }?.let { it.metadata.name to it }
                ?: singles.minByOrNull { floorDistance(it) }?.let { hiveNameForTag(it.id)!! to it }
                ?: return null

        val (name, detection) = best
        return HiveTarget(
            name = name,
            distance = floorDistance(detection),
            bearing = detection.ftcPose.bearing,
            range = detection.ftcPose.range,
        )
    }

    fun close() {
        FtcDashboard.getInstance()?.stopCameraStream()
        visionPortal.close()
    }

    // --- Helpers ---

    /** Distance along the floor (ignores how high the tags are mounted). */
    private fun floorDistance(detection: AprilTagDetection): Double =
        hypot(detection.ftcPose.x, detection.ftcPose.y)

    private fun hiveNameForTag(id: Int): String? =
        when (id) {
            in 30..33 -> "RED SCORING"
            in 34..37 -> "RED AUDIENCE"
            in 38..41 -> "BLUE AUDIENCE"
            in 42..45 -> "BLUE SCORING"
            else -> null
        }

    private fun isAcceptedHive(name: String?): Boolean =
        when (name) {
            "RED SCORING" -> ACCEPT_RED_SCORING
            "BLUE SCORING" -> ACCEPT_BLUE_SCORING
            "RED AUDIENCE" -> ACCEPT_RED_AUDIENCE
            "BLUE AUDIENCE" -> ACCEPT_BLUE_AUDIENCE
            else -> false
        }

    private fun sleepMs(ms: Long) {
        try {
            Thread.sleep(ms)
        } catch (_: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }
}
