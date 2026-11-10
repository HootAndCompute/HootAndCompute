package org.firstinspires.ftc.teamcode.opmode.manual

import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.manual.AdvancedDrivetrainManualMode
import org.firstinspires.ftc.teamcode.robot.Rachel

@VoltOpModeMeta("Bumblebee", "Competition")
class Bumblebee : AdvancedDrivetrainManualMode<PinpointMecanumPedroPathingDrivetrain, Rachel>() {
    val poseFactory: PoseFactory = PoseFactory.degrees()
    val initialPose : Pose = poseFactory.of(0.0,0.0,90.0)
    override val robot = Rachel(hardwareMap,initialPose)
}
