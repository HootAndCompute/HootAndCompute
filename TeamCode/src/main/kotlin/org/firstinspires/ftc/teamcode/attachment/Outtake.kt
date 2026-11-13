package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.Attachment
import kotlin.math.abs
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * [Outtake] is an [Attachment] that controls a single velocity-controlled outtake motor.
 *
 * It starts at a deliberately low [startVelocity] so the speed can be raised in small [velocityStep]
 * increments while testing, up to [maxVelocity].
 *
 * @param motor the [DcMotorEx] that drives the outtake (encoder must be connected)
 * @param direction the [DcMotorSimple.Direction] that spins the outtake outward
 * @param startVelocity the initial target velocity in ticks/s
 * @param maxVelocity the maximum allowed velocity in ticks/s
 * @param velocityStep how much each speed adjustment changes the target, in ticks/s
 * @param velocityTolerance how close to the target counts as "at speed", in ticks/s
 * @param pidfCoefficients optional velocity [PIDFCoefficients]; SDK defaults are used when null
 * @property targetVelocity the selected velocity the outtake runs at when enabled
 * @property isRunning whether the outtake is currently enabled
 * @property isAtSpeed whether the motor is within [velocityTolerance] of [targetVelocity]
 */
class Outtake(
    private val motor: DcMotorEx,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    startVelocity: Double,
    private val maxVelocity: Double,
    private val velocityStep: Double,
    private val velocityTolerance: Double = 40.0,
    pidfCoefficients: PIDFCoefficients? = null,
) : Attachment("Outtake") {

    // --- Properties ---

    var targetVelocity = startVelocity.coerceIn(0.0, maxVelocity)
        private set

    var isRunning = false
        private set

    val isAtSpeed: Boolean
        get() = isRunning && abs(motor.velocity - targetVelocity) < velocityTolerance

    init {
        motor.direction = direction
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        pidfCoefficients?.let { motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, it) }
        motor.velocity = 0.0
    }

    // --- Actions ---

    /**
     * Enables the outtake at [velocity] and waits until it is at speed.
     *
     * @return an [Action] that spins up the outtake
     */
    @VoltAction(name = "Enable Outtake", description = "Spins up the outtake to target velocity")
    fun enable(velocity: Double = targetVelocity): Action = action {
        init {
            targetVelocity = velocity.coerceIn(0.0, maxVelocity)
            setRunning(true)
        }
        loop {
            put("Outtake velocity", motor.velocity)
            !isRunning || isAtSpeed
        }
    }

    /**
     * Disables the outtake.
     *
     * @return an [Action] that stops the outtake
     */
    @VoltAction(name = "Disable Outtake", description = "Stops the outtake")
    fun disable(): Action = instant { setRunning(false) }

    /**
     * Raises [targetVelocity] by one step (applied immediately if running).
     *
     * @return an [Action] that increases outtake speed
     */
    @VoltAction(name = "Increase Outtake Speed", description = "Raises outtake velocity one step")
    fun increaseSpeed(): Action = instant { adjustVelocity(velocityStep) }

    /**
     * Lowers [targetVelocity] by one step (applied immediately if running).
     *
     * @return an [Action] that decreases outtake speed
     */
    @VoltAction(name = "Decrease Outtake Speed", description = "Lowers outtake velocity one step")
    fun decreaseSpeed(): Action = instant { adjustVelocity(-velocityStep) }

    // --- Helpers ---

    private fun instant(block: () -> Unit): Action = action {
        init { block() }
        loop { true }
    }

    private fun adjustVelocity(delta: Double) {
        targetVelocity = (targetVelocity + delta).coerceIn(0.0, maxVelocity)
        if (isRunning) motor.velocity = targetVelocity
    }

    private fun setRunning(running: Boolean) {
        isRunning = running
        motor.velocity = if (running) targetVelocity else 0.0
    }

    context(telemetry: Telemetry)
    override fun update() {
        super.update()
        with(telemetry) {
            addData("Outtake Status", if (isRunning) "RUNNING" else "STOPPED")
            addData("Outtake Target", "%.0f / %.0f".format(targetVelocity, maxVelocity))
            addData("Outtake Velocity", "%.0f".format(motor.velocity))
            addData("Outtake At Speed", if (isAtSpeed) "✓" else "—")
            addData("Outtake Overcurrent", if (motor.isOverCurrent) "⚠ YES" else "✓ NO")
        }
    }

    override fun stop() {
        super.stop()
        setRunning(false)
    }
}
