package org.firstinspires.ftc.teamcode.opmode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@Autonomous(name = "Bat")
class Bat : LinearOpMode() {

    private lateinit var leftFront: DcMotor
    private lateinit var leftBack: DcMotor
    private lateinit var rightFront: DcMotor
    private lateinit var rightBack: DcMotor

    override fun runOpMode() {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor::class.java, "leftfront")
        leftBack = hardwareMap.get(DcMotor::class.java, "leftback")
        rightFront = hardwareMap.get(DcMotor::class.java, "rightfront")
        rightBack = hardwareMap.get(DcMotor::class.java, "rightback")

        // Set motor directions
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        rightBack.direction = DcMotorSimple.Direction.FORWARD

        // Set zero power behavior
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Reset encoders
        leftFront.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightFront.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightBack.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Set run mode
        leftFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        leftBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightFront.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rightBack.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        telemetry.addData("Status", "Auto Initialized")
        telemetry.update()

        waitForStart()

        if (isStopRequested) return

        // Drive forward
        leftFront.power = 0.5
        leftBack.power = 0.5
        rightFront.power = 0.5
        rightBack.power = 0.5

        sleep(1000)

        // Stop motors
        leftFront.power = 0.0
        leftBack.power = 0.0
        rightFront.power = 0.0
        rightBack.power = 0.0
    }
}