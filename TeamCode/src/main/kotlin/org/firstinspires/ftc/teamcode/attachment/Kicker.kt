package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.Servo
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.ServoAttachment
import dev.kingssack.volt.util.ServoPosition

class Kicker(
    kickerServo: Servo
) : ServoAttachment("Kicker", kickerServo) {
    /* This moves the kicker forward to push the pollen into the outtake */
    @VoltAction("Kick forward", "This kicks the pollen into the outtake")
    fun forward(): Action = goTo(ServoPosition(1.0))

    /* This retracts the kicker */
    @VoltAction("Retract Kicker", "This retracts the kicker")
    fun retract(): Action = goTo(ServoPosition(0.0))
}
