package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.pedropathing.api.Paths.*
import com.pedropathing.api.PoseFactory
import com.pedropathing.math.Pose
import dev.kingssack.volt.opmode.VoltOpModeMeta
import dev.kingssack.volt.opmode.autonomous.AutonomousMode
import dev.kingssack.volt.util.Event.AutonomousEvent.Start
import org.firstinspires.ftc.teamcode.robot.Rachel

@VoltOpModeMeta("kyintest")
class kyintest : AutonomousMode<Rachel>() {
    val poseFactory: PoseFactory = PoseFactory.degrees()
    val start : Pose = poseFactory.of(0.0,0.0,90.0)

    override val robot = Rachel(hardwareMap, start)

    private val end = robot.drivetrain.poseFactory.of(0.0, 25.0, 90.0)

    private val path1 = line(start, end).linear(start, end)

    init {
        Start then {
            +robot.drivetrain.followPath(path1)
        }
    }
}
