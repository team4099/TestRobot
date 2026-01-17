package com.team4099.lib.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N4
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.units.base.Time

/** Represents a single vision pose with a timestamp and associated standard deviations. */
data class TimestampedVisionUpdate(
  val timestamp: Time,
  val fieldTRobot: Pose3d,
  val stdDevs: Matrix<N4, N1>,
  val fromVision: Boolean = false
)
