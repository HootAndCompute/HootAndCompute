package org.firstinspires.ftc.teamcode.opmode.manual

import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.manual.AdvancedDrivetrainManualMode
import dev.kingssack.volt.util.Event.ManualEvent.Tap
import dev.kingssack.volt.util.buttons.Button
import org.firstinspires.ftc.teamcode.robot.David

/**
 * Hummingbird drives [David] and controls both his
 * [org.firstinspires.ftc.teamcode.attachment.Intake] and
 * [org.firstinspires.ftc.teamcode.attachment.Outtake].
 *
*/
@VoltOpModeMeta("Hummingbird", "Testing")
class Hummingbird : AdvancedDrivetrainManualMode<PinpointMecanumPedroPathingDrivetrain, David>() {
    override val robot = David(hardwareMap)

    private fun defineControls() {
        // Intake
        Tap(Button.RIGHT_BUMPER2) then { +robot.intake.enable() }
        Tap(Button.LEFT_BUMPER2) then { +robot.intake.disable() }
        Tap(Button.LEFT_STICK_BUTTON2) then { +robot.intake.reverse() }

        // Outtake
        Tap(Button.A2) then { +robot.outtake.enable() }
        Tap(Button.B2) then { +robot.outtake.disable() }
        Tap(Button.DPAD_UP2) then { +robot.outtake.increaseSpeed() }
        Tap(Button.DPAD_DOWN2) then { +robot.outtake.decreaseSpeed() }

        // Kicker
        Tap(Button.DPAD_RIGHT2) then { +robot.kicker.forward() }
        Tap(Button.DPAD_LEFT2) then { +robot.kicker.retract() }

        // Gate operation
        Tap(Button.X2) then { +robot.gate.openGate() }
        Tap(Button.Y2) then { +robot.gate.closeGate() }
    }

    init {
        defineControls()
    }
}
