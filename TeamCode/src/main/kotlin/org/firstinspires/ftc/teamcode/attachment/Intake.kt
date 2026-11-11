package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.Attachment
import dev.kingssack.volt.util.Power
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
    private val intakePower: Power = Power(1.0),
    private val reversePower: Power = Power(1.0),
) : Attachment("Intake") {

    // --- Properties ---

    var power = Power(0.0)
        private set

    val isRunning: Boolean
        get() = power.value != 0.0

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
     * @return an [Action] that enables the intake
     */
    @VoltAction(name = "Enable Intake", description = "Spins the intake inward")
    fun enable(power: Power = intakePower): Action = instant { setPower(Power(abs(power.value))) }

    /**
     * Spins the intake outward at [power] to eject artifacts.
     *
     * @return an [Action] that reverses the intake
     */
    @VoltAction(name = "Reverse Intake", description = "Spins the intake outward")
    fun reverse(power: Power = reversePower): Action = instant { setPower(Power(-abs(power.value))) }

    /**
     * Stops the intake.
     *
     * @return an [Action] that disables the intake
     */
    @VoltAction(name = "Disable Intake", description = "Stops the intake")
    fun disable(): Action = instant { setPower(Power(0.0)) }

    // --- Helpers ---

    private fun instant(block: () -> Unit): Action = action {
        init { block() }
        loop { true }
    }

    private fun setPower(target: Power) {
        power = target
        motor.power = target.value
    }

    context(telemetry: Telemetry)
    override fun update() {
        super.update()
        with(telemetry) {
            addData(
                "Intake Status",
                when {
                    power.value > 0.0 -> "INTAKING"
                    power.value < 0.0 -> "REVERSING"
                    else -> "STOPPED"
                },
            )
            addData("Intake Power", "%.2f".format(power.value))
        }
    }

    override fun stop() {
        super.stop()
        setPower(Power(0.0))
    }
}
