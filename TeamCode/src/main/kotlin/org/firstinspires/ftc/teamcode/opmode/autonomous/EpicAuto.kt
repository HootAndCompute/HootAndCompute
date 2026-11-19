package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.api.Paths.*
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.autonomous.AutonomousMode
import dev.kingssack.volt.util.Event.AutonomousEvent.Start
import org.firstinspires.ftc.teamcode.robot.David

@VoltOpModeMeta("EpicAuto")
class EpicAuto : AutonomousMode<David>() {
    val poseFactory: PoseFactory = PoseFactory.degrees()
    val initialPose: Pose = poseFactory.of(58.6841, 141.5, 90.0)

    override val robot = David(hardwareMap, poseFactory, initialPose)

    private val end = robot.drivetrain.poseFactory.of(12.4376, 116.0672, 90.0)

    private val path1 = line(initialPose, end).constant(end)

    init {
        Start then {
            +robot.outtake.enable(1500.0)
            wait(5.0)
            +robot.gate.openGate()
            wait(10.0)
            +robot.gate.closeGate()
            +robot.outtake.disable()
            +robot.drivetrain.followPath(path1)
        }
    }
}