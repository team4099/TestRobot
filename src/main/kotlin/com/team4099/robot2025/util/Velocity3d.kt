package com.team4099.robot2025.util

import edu.wpi.first.math.geometry.Translation3d
import org.team4099.lib.geometry.Quaternion
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.angle
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.pow
import kotlin.math.sqrt

data class Velocity3d(val x: LinearVelocity, val y: LinearVelocity, val z: LinearVelocity) {

  constructor() : this(0.0.meters.perSecond, 0.0.meters.perSecond, 0.0.meters.perSecond)

  val velocity3dWPIlib =
    Translation3d(x.inMetersPerSecond, y.inMetersPerSecond, z.inMetersPerSecond)

  val magnitude =
    sqrt(x.inMetersPerSecond.pow(2) + y.inMetersPerSecond.pow(2) + z.inMetersPerSecond.pow(2))
      .meters
      .perSecond

  val heading: Angle = velocity3dWPIlib.toTranslation2d().angle.angle

  operator fun plus(other: Velocity3d): Velocity3d {
    return Velocity3d(x + other.x, y + other.y, z + other.z)
  }

  operator fun minus(other: Velocity3d): Velocity3d {
    return Velocity3d(x - other.x, y - other.y, z - other.z)
  }

  operator fun times(scalar: Double): Velocity3d {
    return Velocity3d(x * scalar, y * scalar, z * scalar)
  }

  operator fun div(scalar: Double): Velocity3d {
    return Velocity3d(x / scalar, y / scalar, z / scalar)
  }

  operator fun unaryMinus(): Velocity3d {
    return Velocity3d(x * -1, y * -1, z * -1)
  }

  fun rotateBy(other: Angle): Velocity3d {
    return rotateBy(Rotation3d(0.radians, 0.radians, other))
  }

  fun rotateBy(other: Rotation3d): Velocity3d {
    val p = Quaternion(0.0.radians, x.inMetersPerSecond, y.inMetersPerSecond, z.inMetersPerSecond)
    val qprime = other.quaternion.times(p).times(other.quaternion.inverse())
    return Velocity3d(qprime.x.perSecond, qprime.y.perSecond, qprime.z.perSecond)
  }

  fun normalize(): Velocity3d {
    return this / magnitude.inMetersPerSecond
  }

  override fun equals(other: Any?): Boolean {
    if (this === other) return true
    if (other !is Velocity3d) return false

    if ((x - other.x).absoluteValue.value > 1E-9) return false
    if ((y - other.y).absoluteValue.value > 1E-9) return false

    return true
  }
}
