package org.firstinspires.ftc.teamcode.opmode.manual

import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.manual.AdvancedDrivetrainManualMode
import dev.kingssack.volt.util.Event.ManualEvent.Tap
import dev.kingssack.volt.util.buttons.Button
import org.firstinspires.ftc.teamcode.robot.Rachel

/**
 * Bat drives [Rachel] and controls her [org.firstinspires.ftc.teamcode.attachment.Outtake].
 *
 * Gamepad 2:
 * - A: toggle outtake on/off
 * - D-pad up: increase outtake speed one step
 * - D-pad down: decrease outtake speed one step
 */
@VoltOpModeMeta("Bat", "Testing")
class Bat : AdvancedDrivetrainManualMode<PinpointMecanumPedroPathingDrivetrain, Rachel>() {
    override val robot = Rachel(hardwareMap)

    private fun defineControls() {
        Tap(Button.A2) then { +robot.outtake.toggle() }
        Tap(Button.DPAD_UP2) then { +robot.outtake.increaseSpeed() }
        Tap(Button.DPAD_DOWN2) then { +robot.outtake.decreaseSpeed() }
    }

    init {
        defineControls()
    }
}
