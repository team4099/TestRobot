package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import kotlin.math.PI
import kotlin.math.atan2

class FaceHubCommand(
  private val drivetrain: Drive,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
) : Command() {

  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private val HUB_TRANSLATION = AllianceFlipUtil.apply(Translation2d(182.11.inches, 158.84.inches))
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
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    CustomLogger.recordOutput("FaceHubCommand/lastInitialized", Clock.fpgaTime.inSeconds)

    CustomLogger.recordOutput("FaceHubCommand/hubTranslation", HUB_TRANSLATION.translation2d)
    hasAligned = false
  }

  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", true)

    CustomLogger.recordOutput("FaceHubCommand/currentRotation", drivetrain.rotation.inDegrees)

    val wantedRot =
      atan2(
        (HUB_TRANSLATION.y - drivetrain.pose.translation.y).inMeters,
        (HUB_TRANSLATION.x - drivetrain.pose.translation.x).inMeters
      )
        .radians

    val thetavel = thetaPID.calculate(drivetrain.rotation, wantedRot)
    val ff = thetaPID.error * DrivetrainConstants.PID.SIM_HUB_PID_KV

    val appliedThetavel = thetavel + ff

    CustomLogger.recordOutput("FaceHubCommand/thetaveldps", appliedThetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("FaceHubCommand/thetaerror", thetaPID.error.inDegrees)
    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)

    drivetrain.runSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        speed.first, speed.second, appliedThetavel, drivetrain.pose.rotation
      ),
      flipIfRed = true
    )

    hasAligned = thetaPID.error.absoluteValue < 2.5.degrees

    CustomLogger.recordOutput("FaceHubCommand/hasAligned", hasAligned)
  }

  override fun isFinished(): Boolean {
    return false
  }

  override fun end(interrupted: Boolean) {

    CustomLogger.recordOutput("FaceHubCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }
}
