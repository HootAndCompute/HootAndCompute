package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.api.Paths.*
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.autonomous.AutonomousMode
import dev.kingssack.volt.util.Event.AutonomousEvent.Start
import org.firstinspires.ftc.teamcode.robot.Rachel

@VoltOpModeMeta("Mosquito")
class Mosquito : AutonomousMode<Rachel>() {
    val poseFactory: PoseFactory = PoseFactory.degrees()
    val initialPose : Pose = poseFactory.of(0.0,0.0,90.0)

    override val robot = Rachel(hardwareMap, initialPose)

    private val end = robot.drivetrain.poseFactory.of(0.0, 25.0, 90.0)

    private val path1 = line(initialPose, end).linear(initialPose, end)

    init {
        Start then {
            +robot.drivetrain.followPath(path1)
        }
    }
}