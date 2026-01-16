package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.superstructure.Shooter
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.dyn4j.geometry.Circle
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation
import org.ironmaple.simulation.gamepieces.GamePieceProjectile
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRadians
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sqrt
import edu.wpi.first.math.geometry.Translation2d as WPITranslation2d

/** @author Nathan Arega, Ryan Chung */
class FaceHubCommand(
  private val drivetrain: Drive,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
) : Command() {
  // TODO replace with real
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var HUB_TRANSLATION: Translation3d =
    AllianceFlipUtil.apply(Translation3d(182.11.inches, 158.84.inches, 72.inches))
  private val MAX_VELOCITY_RADIUS = 1.5.meters.perSecond

  var hasAligned: Boolean = false

  init {
    addRequirements(drivetrain)

    if (RobotBase.isSimulation()) {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_HUB_PID_KP,
          DrivetrainConstants.PID.SIM_HUB_PID_KI,
          DrivetrainConstants.PID.SIM_HUB_PID_KD
        )
    } else {
      if (DriverStation.isAutonomous()) {
        thetaPID =
          PIDController(
            DrivetrainConstants.PID.AUTO_REEF_PID_KP,
            DrivetrainConstants.PID.AUTO_REEF_PID_KI,
            DrivetrainConstants.PID.AUTO_REEF_PID_KD
          )
      } else {
        thetaPID =
          PIDController(
            DrivetrainConstants.PID.TELEOP_THETA_PID_KP,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KI,
            DrivetrainConstants.PID.TELEOP_THETA_PID_KD
          )
      }
    }

    thetaPID.enableContinuousInput(-PI.radians, PI.radians)
  }

  override fun initialize() {
    thetaPID.reset()

    hasAligned = false
    HUB_TRANSLATION =
      if ((
        AllianceFlipUtil.shouldFlip() &&
          drivetrain.pose.x > AllianceFlipUtil.apply(158.6.inches)
        ) ||
        (!AllianceFlipUtil.shouldFlip() && drivetrain.pose.x < 158.6.inches)
      ) {
        AllianceFlipUtil.apply(Translation3d(182.11.inches, 158.84.inches, 72.inches))
      } else {
        AllianceFlipUtil.apply(Translation3d(120.inches, 158.84.inches, 72.inches))
      }
  }

  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", true)

    // TODO - Realistically, some of this should be moved to superstructure during SCORE state?
    val (distanceToHub, launchSpeedField, launchSpeedZ, timeOfFlight, wantedRotation) =
      Shooter.calculateLaunchVelocity(drivetrain.pose, drivetrain.chassisSpeeds, HUB_TRANSLATION)

    // PID and clamping of the calculated theta velocity
    val thetaVel = thetaPID.calculate(drivetrain.rotation, wantedRotation)

    CustomLogger.recordOutput("FaceHubCommand/thetaError", thetaPID.error.inDegrees)

    CustomLogger.recordOutput(
      "FaceHubCommand/wantedPose",
      Pose2d(drivetrain.pose.x, drivetrain.pose.y, wantedRotation).pose2d
    )

    // Take the drivers speed being inputted, and clamp the magnitude
    // of the drive vector to < MAX_VELOCITY_RADIUS meters per second
    var (speedX, speedY) = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
    val speedMagnitude =
      sqrt(speedX.inMetersPerSecond.pow(2) + speedY.inMetersPerSecond.pow(2)).meters.perSecond

    if (speedMagnitude > 0.1.meters.perSecond || !hasAligned) {
      if (speedMagnitude > MAX_VELOCITY_RADIUS) {
        // Convert to unit vector and then * MAX_VELOCITY_RADIUS
        speedX = speedX / speedMagnitude.inMetersPerSecond * MAX_VELOCITY_RADIUS.inMetersPerSecond
        speedY = speedY / speedMagnitude.inMetersPerSecond * MAX_VELOCITY_RADIUS.inMetersPerSecond
      }

      drivetrain.runSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, thetaVel, drivetrain.pose.rotation)
      )
    } else {
      drivetrain.stopWithX()
    }

    // Instead of using just angle to check if the robot is aligned, base
    // error on if the arc length surpasses the inradius of the HUB opening
    hasAligned = distanceToHub * thetaPID.error.absoluteValue.inRadians < 41.73.inches / 2

    CustomLogger.recordOutput("FaceHubCommand/hasAligned", hasAligned)

    if (RobotBase.isSimulation() && hasAligned && Clock.fpgaTime.inSeconds % .25 < 0.05) {
      val fieldSpeeds =
        ChassisSpeeds(
          edu.wpi.first.math.kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(
            drivetrain.chassisSpeeds.chassisSpeedsWPILIB, drivetrain.rotation.inRotation2ds
          )
        )

      val shooterPosition =
        drivetrain.pose.translation + Shooter.SHOOTER_OFFSET.rotateBy(drivetrain.rotation)

      SimulatedArena.getInstance()
        .addGamePieceProjectile(
          GamePieceProjectile(
            GamePieceOnFieldSimulation.GamePieceInfo(
              "Fuel",
              Circle(0.15),
              Meters.of(0.15),
              Kilograms.of(0.226796),
              0.05,
              0.085,
              0.4
            ),
            shooterPosition.translation2d,
            WPITranslation2d(
              fieldSpeeds.vx.inMetersPerSecond, fieldSpeeds.vy.inMetersPerSecond
            ) +
              WPITranslation2d(launchSpeedField.inMetersPerSecond, 0.0)
                .rotateBy(drivetrain.rotation.inRotation2ds),
            Shooter.SHOOTER_HEIGHT.inMeters,
            launchSpeedZ.inMetersPerSecond,
            Rotation3d.kZero
          )
        )
    }
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {
    CustomLogger.recordOutput("FaceHubCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", false)
  }
}
