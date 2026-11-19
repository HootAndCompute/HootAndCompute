package org.firstinspires.ftc.teamcode.attachment

import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.hardware.Servo
import dev.kingssack.volt.annotations.VoltAction
import dev.kingssack.volt.attachment.Attachment
import dev.kingssack.volt.attachment.ServoAttachment
import dev.kingssack.volt.util.ServoPosition
import org.firstinspires.ftc.robotcore.external.Telemetry

class Gate(
    private val bigAhhServo: Servo
) : ServoAttachment("Gate", bigAhhServo) {
    var gateIsOpen = false
        private set

    /* This opens the gate */
    @VoltAction("Open Gate", "This opens the gate")
    fun openGate(): Action {
        gateIsOpen = true
        return goTo(ServoPosition(1.0))
    }

    /* This closes the gate */
    @VoltAction("Close Gate", "This closes the gate")
    fun closeGate(): Action {
        gateIsOpen = false
        return goTo(ServoPosition(0.0))
    }

    /* telemetry */
    context(telemetry: Telemetry)
    override fun update() {
        super.update()
        with(telemetry) {
            addData(
                "Gate Status",
                when {
                    gateIsOpen -> "Open"
                    else -> "Closed"
                }
            )
        }
    }
}