package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.Servo
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.Attachment
import dev.kingssack.volt.attachment.ServoAttachment
import dev.kingssack.volt.util.ServoPosition

class Gate(
    private val bigAhhServo: Servo
) : ServoAttachment("Gate", bigAhhServo) {
    /* This opens the gate */
    @VoltAction("Open Gate", "This opens the gate")
    fun openGate(): Action = goTo(ServoPosition(1.0))

    /* This closes the gate */
    @VoltAction("Close Gate", "This closes the gate")
    fun closeGate(): Action = goTo(ServoPosition(0.0))
}