package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.config.Config

/**
 * Maps the camera-measured distance to a scoring hive (inches) to an outtake velocity (ticks/s).
 *
 * The table is filled in by testing: park the robot at a distance, read "Hive Distance" from
 * telemetry, find the outtake velocity that scores reliably, and write the pair below. Between
 * points the velocity is linearly interpolated; outside the table it is held at the nearest end
 * point (no extrapolation).
 *
 * Every value is a `@JvmField var` so it can be live-tuned from FTC Dashboard (Config tab →
 * OuttakeVelocityTable) without redeploying. Copy the final values back into this file.
 *
 * Rules: distances must be in increasing order. A point with distance <= 0 is ignored, so unused
 * slots can be left at 0.
 */
@Config
object OuttakeVelocityTable {
    // TODO(test): fill in measured distance (in) → velocity (ticks/s) pairs.
    //  The values below are placeholders only; they have NOT been tested.
    @JvmField var DISTANCE_1 = 24.0
    @JvmField var VELOCITY_1 = 1000.0

    @JvmField var DISTANCE_2 = 36.0
    @JvmField var VELOCITY_2 = 1150.0

    @JvmField var DISTANCE_3 = 48.0
    @JvmField var VELOCITY_3 = 1300.0

    @JvmField var DISTANCE_4 = 60.0
    @JvmField var VELOCITY_4 = 1450.0

    @JvmField var DISTANCE_5 = 72.0
    @JvmField var VELOCITY_5 = 1600.0

    @JvmField var DISTANCE_6 = 0.0
    @JvmField var VELOCITY_6 = 0.0

    @JvmField var DISTANCE_7 = 0.0
    @JvmField var VELOCITY_7 = 0.0

    private fun points(): List<Pair<Double, Double>> =
        listOf(
                DISTANCE_1 to VELOCITY_1,
                DISTANCE_2 to VELOCITY_2,
                DISTANCE_3 to VELOCITY_3,
                DISTANCE_4 to VELOCITY_4,
                DISTANCE_5 to VELOCITY_5,
                DISTANCE_6 to VELOCITY_6,
                DISTANCE_7 to VELOCITY_7,
            )
            .filter { it.first > 0.0 }
            .sortedBy { it.first }

    /**
     * @param distance horizontal distance from the camera to the hive tags, in inches
     * @return the interpolated outtake velocity in ticks/s, or null if the table is empty
     */
    fun velocityFor(distance: Double): Double? {
        val pts = points()
        if (pts.isEmpty()) return null
        if (distance <= pts.first().first) return pts.first().second
        if (distance >= pts.last().first) return pts.last().second

        val upper = pts.indexOfFirst { it.first >= distance }
        val (d0, v0) = pts[upper - 1]
        val (d1, v1) = pts[upper]
        val t = (distance - d0) / (d1 - d0)
        return v0 + t * (v1 - v0)
    }
}
