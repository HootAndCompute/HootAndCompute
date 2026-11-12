package org.firstinspires.ftc.teamcode.robot

import com.pedropathing.algorithm.Foresight
import com.pedropathing.algorithm.ForesightConfig
import com.pedropathing.controllers.Controller
import com.pedropathing.math.Matrix
import com.pedropathing.math.Vector2D
import com.pedropathing.revhub.drivetrains.MecanumConfig
import com.pedropathing.revhub.localizers.PinpointConfig
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.kingssack.volt.attachment.drivetrain.pp.mecanum.PinpointMecanumPedroPathingDrivetrain
import dev.kingssack.volt.robot.DrivetrainRobot
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.attachment.Intake
import org.firstinspires.ftc.teamcode.attachment.Outtake

/**
 * David carries the [Intake] and [Outtake].
 *
 * TODO: drivetrain config below is copied from [Rachel] as a placeholder — retune for David.
 */
class David(hardwareMap: HardwareMap) : DrivetrainRobot<PinpointMecanumPedroPathingDrivetrain>(
    hardwareMap,
    PinpointMecanumPedroPathingDrivetrain(
        hardwareMap,
        PinpointConfig { c ->
            // Pinpoint tuning goes here
            c.name.set("pinpoint")
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            c.xPodOffset.set(-2.0175490792342057)
            c.yPodOffset.set(0.6881864052119218)
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            c.globalDistanceUnit.set(DistanceUnit.INCH)
            c.offsetUnits.set(DistanceUnit.INCH)
        },
        MecanumConfig { c ->
            // Mecanum tuning goes here
            c.frontLeftName.set("leftfront")
            c.frontRightName.set("rightfront")
            c.backLeftName.set("leftback")
            c.backRightName.set("rightback")
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD)
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE)
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD)
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE)
        },
        Foresight(
            ForesightConfig { c ->
                // Foresight tuning goes here
                val primaryTranslationalForward = Controller.proportional(0.2098691010306169)
                val secondaryTranslationalForward = Controller.proportional(0.07754102502515298)
                val primaryTranslationalLateral = Controller.proportional(0.29212452365761676)
                val secondaryTranslationalLateral = Controller.proportional(0.10793220578045742)

                c.forwardTranslational.set(
                    Controller.piecewise(secondaryTranslationalForward)
                        .put(2.5, primaryTranslationalForward)
                )
                c.strafeTranslational.set(
                    Controller.piecewise(secondaryTranslationalLateral)
                        .put(2.5, primaryTranslationalLateral)
                )

                c.coast.set(Controller.proportionalFeedforward(0.01971516482954282))
                c.brake.set(Controller.proportionalFeedforward(0.016757890105111396))

                c.headingFeedback.set(Controller.proportional(2.5176773194183926))
                c.headingBrakeCoefficients.set(
                    Vector2D.cartesian(
                        0.05966211996000128,
                        0.0064097441941583806
                    )
                )

                c.linearBrakeCoefficients.set(
                    Matrix.diag(
                        0.024221370117483272,
                        0.06596489984209315
                    )
                )
                c.quadraticBrakeCoefficients.set(
                    Matrix.diag(
                        0.004424107281391068,
                        0.0022634360593539573
                    )
                )

                c.maxAchievableForwardVelocity.set(51.90303219487147)
                c.maxAchievableStrafeVelocity.set(43.090628532541935)
                c.naturalForwardDeceleration.set(44.849449040709786)
                c.naturalStrafeDeceleration.set(63.55964608467275)
            }
        ),
    ),
) {
    // --- Constants ---

    // Outtake velocities in ticks/s. Max is kept below the REV UltraPlanetary's ~2800 ticks/s
    // no-load speed; start lower for safety and step up with the d-pad.
    private val outtakeStartVelocity = 1000.0
    private val outtakeMaxVelocity = 2100.0
    private val outtakeVelocityStep = 50.0

    // --- Hardware ---

    private val intakeMotor by motor("intake")
    private val outtakeMotor by motorEx("outtake")

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
}
