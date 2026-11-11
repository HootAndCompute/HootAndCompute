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

class Rachel(hardwareMap: HardwareMap) : DrivetrainRobot<PinpointMecanumPedroPathingDrivetrain>(
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
)
