package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.Servo
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.ServoAttachment
import dev.kingssack.volt.util.ServoPosition
import org.firstinspires.ftc.robotcore.external.Telemetry

class Kicker(
    kickerServo: Servo
) : ServoAttachment("Kicker", kickerServo) {

    // Properties
    var isExtended = false
        private set

    /* This moves the kicker forward to push the pollen into the outtake */
    @VoltAction("Extend Kicker", "This kicks the pollen into the outtake")
    fun extend(): Action {
        isExtended = true
        return goTo(ServoPosition(1.0))
    }

    /* This retracts the kicker */
    @VoltAction("Retract Kicker", "This retracts the kicker")
    fun retract(): Action {
        isExtended = false
        return goTo(ServoPosition(0.0))
    }

    /* telemetry */
    context(telemetry: Telemetry)
    override fun update() {
        super.update()
        with(telemetry) {
            addData(
                "Kicker Status",
                when {
                    isExtended -> "Extended"
                    else -> "Retracted"
                }
            )
        }
    }
}
