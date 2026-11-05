package org.firstinspires.ftc.teamcode.opmode.manual

import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.manual.AdvancedDrivetrainManualMode
import org.firstinspires.ftc.teamcode.robot.Rachel

@VoltOpModeMeta("Bumblebee", "Competition")
class Bumblebee : AdvancedDrivetrainManualMode<PinpointMecanumPedroPathingDrivetrain, Rachel>() {
    override val robot = Rachel(hardwareMap)
}
