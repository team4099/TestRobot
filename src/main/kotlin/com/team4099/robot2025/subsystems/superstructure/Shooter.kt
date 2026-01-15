package com.team4099.robot2025.subsystems.superstructure

import com.team4099.robot2025.commands.drivetrain.FaceHubCommand
import com.team4099.robot2025.config.constants.Constants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat.N1
import edu.wpi.first.math.Nat.N2
import edu.wpi.first.math.Vector
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.LinearVelocity
import org.team4099.lib.units.base.Length
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Angle
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

class Shooter {
  companion object {
    val SHOOTER_HEIGHT = 14.876.inches
    val SHOOTER_ANGLE = 70.degrees
    val SHOOTER_OFFSET = Translation2d(-9.330139.inches, 0.meters)

    /**
     * Result of a launch velocity calculation.
     *
     * @property distanceToTarget The distance from the robot to the target
     * @property launchVelocityField The velocity of the object after being launched in the field
     * plane
     * @property launchVelocityZ The vertical componenent of the velocity of the object after being
     * launched
     * @property timeOfFlight The time of flight of the object
     * @property wantedRotation The angle the drivetrain needs to be rotated to face the target.
     */
    data class CalculatedLaunchData(
      val distanceToTarget: Length,
      val launchVelocityField: LinearVelocity,
      val launchVelocityZ: LinearVelocity,
      val timeOfFlight: Time,
      val wantedRotation: Angle
    )

    /**
     * Calculates the required velocity to shoot towards a targeted pose, based off the drivetrain's
     * current position and chassis speeds.
     *
     * Bases the required velocity off of the momentum provided by the inertia of the drivetrain, as
     * well as drag. A linear feedforward term is calculated based off distance from the adjusted
     * target position.
     *
     * @param drivetrainPose Instantaneous field-relative pose of the drivetrain
     * @param chassisSpeeds Instantaneous robot-relative speeds of the chassis
     * @param targetTranslation The non-adjusted field-relative pose of the target.
     *
     * @return [CalculatedLaunchData] A data class containing the following information about the
     * trajecotry and other informmation: shooter position, distance to target, launch velocity on
     * the field plane, launch velocity in the vertical direction, time of flight, and the desired
     * rotation to aim in that direction.
     *
     * @see FaceHubCommand
     */
    fun calculateLaunchVelocity(
      drivetrainPose: Pose2d,
      chassisSpeeds: ChassisSpeeds,
      targetTranslation: Translation3d
    ): CalculatedLaunchData {
      val shooterPosition =
        drivetrainPose.translation + SHOOTER_OFFSET.rotateBy(drivetrainPose.rotation)

      val targetHeight = targetTranslation.z

      // Calculate the distance to the HUB
      val distanceToTargetX = targetTranslation.x - shooterPosition.x
      val distanceToTargetY = targetTranslation.y - shooterPosition.y
      val distanceToTargetMag =
        sqrt(distanceToTargetX.inMeters.pow(2) + distanceToTargetY.inMeters.pow(2)).meters

      // Get field-relative drivetrain velocity, and convert it into a vector.
      val fieldSpeeds =
        ChassisSpeeds(
          edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
            chassisSpeeds.chassisSpeedsWPILIB, drivetrainPose.rotation.inRotation2ds
          )
        )
      val driveVector =
        Vector(
          Matrix(
            N2(),
            N1(),
            doubleArrayOf(
              fieldSpeeds.vx.inMetersPerSecond, fieldSpeeds.vy.inMetersPerSecond
            )
          )
        )

      val robotTHubVector =
        Vector(
          Matrix(
            N2(),
            N1(),
            doubleArrayOf(distanceToTargetX.inMeters, distanceToTargetY.inMeters)
          )
        )

      // Get the distance (signed) between the robot and the HUB
      val hubUnitVector = robotTHubVector.times(1.0 / distanceToTargetMag.inMeters)
      val parallelScalar = driveVector.dot(hubUnitVector).meters

      /**
       * Time for the math...
       *
       * We must solve for v_launch, but that depends on the distance the ball will be displaced by
       * the drivetrain velocity times the time of flight. The time of flight is, unfortunately,
       * also based off of v_launch. We combine the following kinematics equations to derive
       * v_launch independently from time of flight.
       *
       * Variables:
       * ```
       * v_launch = Launch velocity, represented as a number (vector) in 2D.
       *            Multiply by SHOOTER_ANGLE.cos or SHOOTER_ANGLE.sin for
       *            v_launch_x or v_launch_y.
       * h = Height of the hub.
       * s = Height of the shooter.
       * d = Distance shooterTRobot + robotTHub.
       * v_parallel = Movement of drivetrain, orthogonal to the robotTHub vector.
       * ```
       *
       * From the following kinematics equations:
       * ```
       * d = (v_launch * SHOOTER_ANGLE.cos + v_parallel) * t (1)
       * h = s + (v_launch * SHOOTER_ANGLE.sin) * t + (-g / 2) * t^2 (2)
       * ```
       *
       * Multiply (1) by (v_launch * SHOOTER_ANGLE.sin)
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * d = (v_launch * SHOOTER_ANGLE.sin) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) * t (3)
       * ```
       *
       * From (2):
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * t = (h - s) + (g / 2) * t^2 (4)
       * ```
       *
       * Plug (4) into (3):
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * d = ((h - s) + (g / 2) * t^2) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) (5)
       * ```
       *
       * From (1):
       * ```
       * t = d / (v_launch * SHOOTER_ANGLE.cos + v_parallel) (6)
       * ```
       * Substitute (6) into (5):
       * ```
       * (v_launch * SHOOTER_ANGLE.sin) * d = = ((h - s) + (g / 2) * (d / (v_launch * SHOOTER_ANGLE.cos + v_parallel))^2) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)`
       *
       * (v_launch * SHOOTER_ANGLE.sin) * d = (h - s) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) + g * d^2 / (2 * (v_launch * SHOOTER_ANGLE.cos + v_p))` (7)
       * ```
       *
       * Multiply by (v_launch * SHOOTER_ANGLE.cos + v_parallel):
       * ```
       * v_launch * SHOOTER_ANGLE.sin * d * (v_launch * SHOOTER_ANGLE.cos + v_parallel) = (h - s) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)^2 + g * d^2 / 2
       *
       * 0 = (h - s) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)^2 + g * d^2 / 2 - v_launch * SHOOTER_ANGLE.sin * d * (v_launch * SHOOTER_ANGLE.cos + v_parallel)
       *
       * 0 = (h - s) * (v_launch * SHOOTER_ANGLE.cos)^2 + (h - s) * v_parallel^2 + (h - s) * (2 * v_launch * SHOOTER_ANGLE.cos * v_parallel) + g * d^2 / 2 - v_launch^2 * SHOOTER_ANGLE.sin * d * SHOOTER_ANGLE.cos - v_launch * SHOOTER_ANGLE.sin * d * v_parallel
       *
       * 0 =    (h - s) * (SHOOTER_ANGLE.cos)^2 * v_launch^2                (Quadratic term)
       *        - SHOOTER_ANGLE.sin * d * SHOOTER_ANGLE.cos * v_launch^2    (Quadratic term)
       *        + (h - s) * 2 * SHOOTER_ANGLE.cos * v_parallel * v_launch   (Linear term)
       *        - SHOOTER_ANGLE.sin * d * v_parallel * v_launch             (Linear term)
       *        + (h - s) * v_parallel^2 + g * d^2 / 2                      (Constant)
       * ```
       *
       * We know have a quadratic in terms of v_launch.
       * ```
       * A = (h - s) * (SHOOTER_ANGLE.cos)^2 - SHOOTER_ANGLE.sin * SHOOTER_ANGLE.cos * d
       * B = 2 * (h - s) * SHOOTER_ANGLE.cos * v_parallel - SHOOTER_ANGLE.sin * d * v_parallel
       * C = (h - s) * v_parallel^2 + g * d^2 / 2
       * ```
       */
      val a =
        (targetHeight.inMeters - SHOOTER_HEIGHT.inMeters) * SHOOTER_ANGLE.cos.pow(2) -
          SHOOTER_ANGLE.sin * SHOOTER_ANGLE.cos * distanceToTargetMag.inMeters
      val b =
        2 *
          (targetHeight.inMeters - SHOOTER_HEIGHT.inMeters) *
          SHOOTER_ANGLE.cos *
          parallelScalar.inMeters -
          SHOOTER_ANGLE.sin * distanceToTargetMag.inMeters * parallelScalar.inMeters
      val c =
        (targetHeight.inMeters - SHOOTER_HEIGHT.inMeters) * parallelScalar.inMeters.pow(2) +
          Constants.Universal.gravity.inMetersPerSecondPerSecond *
          distanceToTargetMag.inMeters.pow(2) / 2.0

      // To account for things like resistive forces, we add a small
      // feedforward boost proportional to the distance
      val launchSpeedFF = (distanceToTargetMag.inMeters * 0.1).meters.perSecond
      val launchSpeed =
        max(
          (-b + sqrt(b.pow(2) - 4.0 * a * c)) / (2 * a),
          (-b - sqrt(b.pow(2) - 4.0 * a * c)) / (2 * a)
        )
          .meters
          .perSecond + launchSpeedFF
      val launchSpeedField = launchSpeed * SHOOTER_ANGLE.cos
      val launchSpeedZ = launchSpeed * SHOOTER_ANGLE.sin

      val timeOfFlight =
        (
          distanceToTargetMag.inMeters /
            (launchSpeed.inMetersPerSecond * SHOOTER_ANGLE.cos + parallelScalar.inMeters)
          )
          .seconds

      // The distance the ball travels while in the air due to momentum
      val ballDistanceOffset = driveVector.times(timeOfFlight.inSeconds)

      // The wanted rotation is recieved by offsetting the usual angle
      // (the slope connecting the robot and the HUB) by the offset the
      // ball would travel in the air.
      val wantedRot =
        atan2(
          distanceToTargetY.inMeters - ballDistanceOffset.get(1),
          distanceToTargetX.inMeters - ballDistanceOffset.get(0)
        )
          .radians

      return CalculatedLaunchData(
        distanceToTargetMag, launchSpeedField, launchSpeedZ, timeOfFlight, wantedRot
      )
    }
  }
}
