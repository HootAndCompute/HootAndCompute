package org.firstinspires.ftc.teamcode.opmode.manual

import com.acmerobotics.dashboard.config.Config
import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.manual.AdvancedDrivetrainManualMode
import dev.kingssack.volt.util.Event.ManualEvent.Tap
import dev.kingssack.volt.util.buttons.Button
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.robot.David
import org.firstinspires.ftc.teamcode.util.HiveCamera
import org.firstinspires.ftc.teamcode.util.OuttakeVelocityTable

/**
 * Wind is [Hummingbird] plus automatic outtake speed from AprilTags.
 *
 * Every loop the [HiveCamera] looks for the tags under a BioBuzz scoring hive. When one is seen,
 * its floor distance is looked up in [OuttakeVelocityTable] and the outtake's target velocity is
 * set to match. When no tag is seen, the target is left exactly where it was (if the outtake is
 * off it stays at 0; enabling it uses the last auto target).
 *
 * Controls are the same as Hummingbird, plus:
 * - BACK2: toggle auto velocity on/off
 * - DPAD_UP2 / DPAD_DOWN2: with auto ON, nudge a trim added on top of the table;
 *   with auto OFF, step the outtake speed manually (Hummingbird behavior)
 * - DPAD_RIGHT2: reset trim to 0
 * - START2: re-apply manual exposure/gain (after editing them in Dashboard)
 */
@Config
@VoltOpModeMeta("Wind", "Testing")
class Wind : AdvancedDrivetrainManualMode<PinpointMecanumPedroPathingDrivetrain, David>() {
    companion object {
        // TODO(test): 1.0 = use each new distance immediately; lower (e.g. 0.3) smooths jitter
        //  in the camera distance at the cost of reacting slower.
        @JvmField var DISTANCE_SMOOTHING = 0.5

        // TODO(test): don't send a new velocity unless it differs by at least this (ticks/s),
        //  so tiny distance jitter doesn't constantly disturb the flywheel PID.
        @JvmField var VELOCITY_DEADBAND = 15.0

        /** Ticks/s added to the trim per D-pad tap while auto is on. */
        @JvmField var TRIM_STEP = 25.0
    }

    override val robot = David(hardwareMap)
    private val camera = HiveCamera(hardwareMap)

    private var autoVelocity = true
    private var trim = 0.0
    private var smoothedDistance: Double? = null
    private var lastSeenMs = 0L

    private fun defineControls() {
        // Intake
        Tap(Button.RIGHT_BUMPER2) then { +robot.intake.enable() }
        Tap(Button.LEFT_BUMPER2) then { +robot.intake.disable() }
        Tap(Button.X2) then { +robot.intake.reverse() }

        // Outtake
        Tap(Button.A2) then { +robot.outtake.enable() }
        Tap(Button.B2) then { +robot.outtake.disable() }
        Tap(Button.DPAD_UP2) then {
            if (autoVelocity) instant { trim += TRIM_STEP } else +robot.outtake.increaseSpeed()
        }
        Tap(Button.DPAD_DOWN2) then {
            if (autoVelocity) instant { trim -= TRIM_STEP } else +robot.outtake.decreaseSpeed()
        }
        Tap(Button.DPAD_RIGHT2) then { instant { trim = 0.0 } }

        // Auto velocity / camera
        Tap(Button.BACK2) then {
            instant {
                autoVelocity = !autoVelocity
                smoothedDistance = null
            }
        }
        Tap(Button.START2) then { instant { camera.tryApplyManualExposure() } }

        // Kicker
        Tap(Button.Y2) then { +robot.kicker.forward() }
        Tap(Button.DPAD_LEFT2) then { +robot.kicker.retract() }
    }

    init {
        defineControls()
    }

    override fun tick() {
        with(telemetry) {
            camera.update()
            updateAutoVelocity()
        }
        super.tick()
    }

    override fun end() {
        camera.close()
        super.end()
    }

    private fun updateAutoVelocity() {
        telemetry.addData("Auto Velocity", if (autoVelocity) "ON" else "OFF (BACK2)")
        telemetry.addData("Trim", "%+.0f".format(trim))

        val hive = camera.getClosestHive()
        if (hive == null) {
            // Not seen this frame: keep the previous velocity untouched.
            val ago = if (lastSeenMs == 0L) "never" else "${System.currentTimeMillis() - lastSeenMs} ms ago"
            telemetry.addData("Hive", "not visible (last seen $ago) — holding velocity")
            return
        }
        lastSeenMs = System.currentTimeMillis()

        val alpha = DISTANCE_SMOOTHING.coerceIn(0.01, 1.0)
        val distance = smoothedDistance?.let { it + alpha * (hive.distance - it) } ?: hive.distance
        smoothedDistance = distance

        telemetry.addData("Hive", hive.name)
        telemetry.addData("Hive Distance", "%.1f in (raw %.1f)".format(distance, hive.distance))
        telemetry.addData("Hive Bearing", "%.1f°".format(hive.bearing))

        if (!autoVelocity) return
        val tableVelocity = OuttakeVelocityTable.velocityFor(distance) ?: return
        val desired = tableVelocity + trim
        telemetry.addData("Auto Target", "%.0f ticks/s".format(desired))

        if (abs(desired - robot.outtake.targetVelocity) >= VELOCITY_DEADBAND) {
            robot.outtake.updateTargetVelocity(desired)
        }
    }
}
