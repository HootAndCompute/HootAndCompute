package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.Attachment
import kotlin.math.abs
import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * [Intake] is an [Attachment] that controls a single motor spinning intake blades.
 *
 * Positive power pulls artifacts in, negative power spits them out.
 *
 * @param motor the [DcMotor] that spins the intake blades
 * @param direction the [DcMotorSimple.Direction] that makes positive power intake
 * @param intakePower the default power used when intaking
 * @param reversePower the default power used when reversing (as a positive magnitude)
 * @property power the power currently applied to the intake motor
 * @property isRunning whether the intake motor is currently powered
 */
class Intake(
    private val motor: DcMotor,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    private val intakePower: Double = 1.0,
    private val reversePower: Double = 1.0,
) : Attachment("Intake") {
    companion object {
        private const val POWER_DEADBAND = 0.05
    }

    // --- Properties ---

    var power = 0.0
        private set

    val isRunning: Boolean
        get() = power != 0.0

    init {
        motor.direction = direction
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.power = 0.0
    }

    // --- Actions ---

    /**
     * Spins the intake inward at [power].
     *
     * @return an [Action] that starts intaking
     */
    @VoltAction(name = "Start Intake", description = "Spins the intake inward")
    fun intake(power: Double = intakePower): Action = setPowerAction(abs(power))

    /**
     * Spins the intake outward at [power] to eject artifacts.
     *
     * @return an [Action] that reverses the intake
     */
    @VoltAction(name = "Reverse Intake", description = "Spins the intake outward")
    fun reverse(power: Double = reversePower): Action = setPowerAction(-abs(power))

    /**
     * Stops the intake.
     *
     * @return an [Action] that stops the intake
     */
    @VoltAction(name = "Stop Intake", description = "Stops the intake")
    fun halt(): Action = setPowerAction(0.0)

    /**
     * Toggles the intake between stopped and intaking at [power].
     *
     * @return an [Action] that toggles the intake
     */
    @VoltAction(name = "Toggle Intake", description = "Toggles the intake on and off")
    fun toggle(power: Double = intakePower): Action = action {
        init { setPower(if (isRunning) 0.0 else abs(power)) }
        loop { true }
    }

    /**
     * Sets the intake to a signed [power] in [-1, 1]. Useful for analog (trigger) control.
     *
     * @return an [Action] that sets the intake power
     */
    fun setPowerAction(power: Double): Action = action {
        init { setPower(power) }
        loop { true }
    }

    private fun setPower(target: Double) {
        val clamped = target.coerceIn(-1.0, 1.0)
        power = if (abs(clamped) < POWER_DEADBAND) 0.0 else clamped
        motor.power = power
    }

    context(telemetry: Telemetry)
    override fun update() {
        super.update()
        with(telemetry) {
            addData(
                "Intake Status",
                when {
                    power > 0.0 -> "INTAKING"
                    power < 0.0 -> "REVERSING"
                    else -> "STOPPED"
                },
            )
            addData("Intake Power", "%.2f".format(power))
        }
    }

    override fun stop() {
        super.stop()
        setPower(0.0)
    }
}
