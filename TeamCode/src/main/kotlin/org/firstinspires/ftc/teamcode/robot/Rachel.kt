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

class Rachel(hardwareMap: HardwareMap, poseFactory: PoseFactory, initialPose: Pose) :
    DrivetrainRobot<PinpointMecanumPedroPathingDrivetrain>(
        hardwareMap,
        PinpointMecanumPedroPathingDrivetrain(
            hardwareMap,
            PinpointConfig {
                // Pinpoint tuning goes here
                it.name.set("pinpoint")
                it.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                it.xPodOffset.set(-1.9)
                it.yPodOffset.set(0.6)
                it.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                it.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED)
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
                it.frontRightDirection.set(DcMotorSimple.Direction.REVERSE)
                it.backLeftDirection.set(DcMotorSimple.Direction.FORWARD)
                it.backRightDirection.set(DcMotorSimple.Direction.REVERSE)
            },
            Foresight(
                ForesightConfig {
                    // Foresight tuning goes here
                    val primaryTranslationalForward = Controller.proportional(0.1951697368)
                    val secondaryTranslationalForward = Controller.proportional(0.07211000272)
                    val primaryTranslationalLateral = Controller.proportional(0.2917547325)
                    val secondaryTranslationalLateral = Controller.proportional(0.1077955778)

                    it.forwardTranslational.set(
                        Controller.piecewise(secondaryTranslationalForward)
                            .put(2.5, primaryTranslationalForward)
                    )
                    it.strafeTranslational.set(
                        Controller.piecewise(secondaryTranslationalLateral)
                            .put(2.5, primaryTranslationalLateral)
                    )

                    it.coast.set(Controller.proportionalFeedforward(0.01813127229))
                    it.brake.set(Controller.proportionalFeedforward(0.01541158145))

                    it.headingFeedback.set(Controller.proportional(2.546942561))
                    it.headingBrakeCoefficients.set(
                        Vector2D.cartesian(0.05071789012, 0.00889022203)
                    )

                    it.linearBrakeCoefficients.set(Matrix.diag(0.3037390685, 0.06689066371))
                    it.quadraticBrakeCoefficients.set(Matrix.diag(-0.003422575056, 0.002562915756))

                    it.maxAchievableForwardVelocity.set(58.11681178)
                    it.maxAchievableStrafeVelocity.set(44.62430423)
                    it.naturalForwardDeceleration.set(56.4859368)
                    it.naturalStrafeDeceleration.set(75.90744029)
                }
            ),
            poseFactory,
            initialPose,
        ),
    )
