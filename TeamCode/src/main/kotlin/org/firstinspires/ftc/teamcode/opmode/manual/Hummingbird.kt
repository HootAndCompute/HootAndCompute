/**
 * This is for the intake of the robot
 */

package org.firstinspires.ftc.teamcode.opmode.manual

import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.manual.AdvancedDrivetrainManualMode
import dev.kingssack.volt.util.Event.ManualEvent.Change
import dev.kingssack.volt.util.Event.ManualEvent.Tap
import dev.kingssack.volt.util.buttons.AnalogInput
import dev.kingssack.volt.util.buttons.Button
import org.firstinspires.ftc.teamcode.robot.Rachel

/**
 * Hummingbird drives [Rachel] and controls her [org.firstinspires.ftc.teamcode.attachment.Intake].
 *
 * Gamepad 2:
 * - Right trigger: intake (proportional)
 * - Left trigger: reverse / eject (proportional)
 * - Right bumper: toggle intake at full power
 * - Left bumper: stop intake
 */
@VoltOpModeMeta("Hummingbird", "Testing")
class Hummingbird : AdvancedDrivetrainManualMode<PinpointMecanumPedroPathingDrivetrain, Rachel>() {
    override val robot = Rachel(hardwareMap)

    private val triggerPower: Double
        get() =
            (AnalogInput.RIGHT_TRIGGER2.handler.value - AnalogInput.LEFT_TRIGGER2.handler.value)
                .toDouble()

    private fun defineControls() {
        // Analog control: right trigger pulls in, left trigger pushes out
        Change(AnalogInput.RIGHT_TRIGGER2) then { +robot.intake.setPowerAction(triggerPower) }
        Change(AnalogInput.LEFT_TRIGGER2) then { +robot.intake.setPowerAction(triggerPower) }

        // Digital control
        Tap(Button.RIGHT_BUMPER2) then { +robot.intake.toggle() }
        Tap(Button.LEFT_BUMPER2) then { +robot.intake.halt() }
    }

    init {
        defineControls()
    }
}
