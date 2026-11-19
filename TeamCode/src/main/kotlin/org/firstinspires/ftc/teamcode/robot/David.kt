package org.firstinspires.ftc.teamcode.robot

import com.pedropathing.algorithm.Foresight
import com.pedropathing.algorithm.ForesightConfig
import com.pedropathing.api.PoseFactory
import com.pedropathing.controllers.Controller
import com.pedropathing.math.Matrix
import com.pedropathing.math.Pose
import com.pedropathing.math.Vector2D
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.pedropathing.revhub.localizers.PinpointConfig
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.robot.DrivetrainRobot
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.attachment.*

/**
 * David carries the [Intake] and [Outtake].
 */
class David(hardwareMap: HardwareMap, poseFactory: PoseFactory, initialPose: Pose) :
    DrivetrainRobot<PinpointMecanumPedroPathingDrivetrain>(
        hardwareMap,
        PinpointMecanumPedroPathingDrivetrain(
            hardwareMap,
            PinpointConfig {
                // Pinpoint tuning goes here
                it.name.set("pinpoint")
                it.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                it.xPodOffset.set(0.0)
                it.yPodOffset.set(0.0)
                it.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                it.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                it.globalDistanceUnit.set(DistanceUnit.INCH)
                it.offsetUnits.set(DistanceUnit.INCH)
            },
            MecanumConfig {
                // Mecanum tuning goes here
                it.frontLeftName.set("leftfront")
                it.frontRightName.set("rightfront")
                it.backLeftName.set("leftback")
                it.backRightName.set("rightback")
                it.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD)
                it.frontRightDirection.set(DcMotorSimple.Direction.FORWARD)
                it.backLeftDirection.set(DcMotorSimple.Direction.REVERSE)
                it.backRightDirection.set(DcMotorSimple.Direction.REVERSE)
            },
            Foresight(
                ForesightConfig {
                    // Foresight tuning goes here
                    val primaryTranslationalForward = Controller.proportional(0.1958287903)
                    val secondaryTranslationalForward = Controller.proportional(0.07235350536)
                    val primaryTranslationalLateral = Controller.proportional(0.3344695261)
                    val secondaryTranslationalLateral = Controller.proportional(0.1235775527)

                    it.forwardTranslational.set(
                        Controller.piecewise(secondaryTranslationalForward)
                            .put(2.5, primaryTranslationalForward),
                    )
                    it.strafeTranslational.set(
                        Controller.piecewise(secondaryTranslationalLateral)
                            .put(2.5, primaryTranslationalLateral),
                    )

                    it.coast.set(Controller.proportionalFeedforward(0.07235350536))
                    it.brake.set(Controller.proportionalFeedforward(0.3344695261))

                    it.headingFeedback.set(Controller.proportional(3.234391008))
                    it.headingBrakeCoefficients.set(
                        Vector2D.cartesian(0.04801236968, 0.007826707204),
                    )

                    it.linearBrakeCoefficients.set(Matrix.diag(0.08514591328, 0.04332634584))
                    it.quadraticBrakeCoefficients.set(Matrix.diag(0.001857174643, 0.002710833301))

                    it.maxAchievableForwardVelocity.set(59.4980951)
                    it.maxAchievableStrafeVelocity.set(47.11568766)
                    it.naturalForwardDeceleration.set(43.64667467)
                    it.naturalStrafeDeceleration.set(43.64667467)
                },
            ),
            poseFactory,
            initialPose,
        ),
    ) {
    // --- Constants ---

    // Outtake velocities in ticks/s. Max is kept below the REV UltraPlanetary's ~2800 ticks/s
    // no-load speed; start lower for safety and step up with the d-pad.
    private val outtakeStartVelocity = 1250.0
    private val outtakeMaxVelocity = 2100.0
    private val outtakeVelocityStep = 50.0

    // --- Hardware ---

    private val intakeMotor by motor("intake")
    private val outtakeMotor by motorEx("outtake")
    private val gateServo by servo("gate")
    private val kickerServo by servo("kicker")

    // --- Attachments ---

    val intake = attachment { Intake(intakeMotor) }

    val outtake = attachment {
        Outtake(
            outtakeMotor,
            startVelocity = outtakeStartVelocity,
            maxVelocity = outtakeMaxVelocity,
            velocityStep = outtakeVelocityStep,
        )
    }

    val kicker = attachment { Kicker(kickerServo) }
    val gate = attachment { Gate(gateServo) }
}
