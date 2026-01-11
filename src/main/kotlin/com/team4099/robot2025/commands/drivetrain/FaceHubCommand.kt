package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.Time
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.inDegreesPerSecond
import kotlin.math.PI

class FaceHubCommand(
  private val drivetrain: Drive,
  val driveX: ()-> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
  
) : Command() {

  private val thetaPID: PIDController<Radian, Velocity<Radian>>

  private var startTime: Time = 0.0.seconds

  init {
    addRequirements(drivetrain)

    if (RobotBase.isSimulation()) {
      thetaPID =
        PIDController(
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KP,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KI,
          DrivetrainConstants.PID.SIM_AUTO_THETA_PID_KD
        )

    } else {
      if (DriverStation.isAutonomous()) { thetaPID = PIDController(
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
    startTime = Clock.fpgaTime
    thetaPID.reset()
    thetaPID.enableContinuousInput(-PI.radians, PI.radians)

    CustomLogger.recordOutput("FaceHubCommand/lastInitialized", Clock.fpgaTime.inSeconds)
  }

  override fun execute() {
    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", true)

    CustomLogger.recordOutput(
      "FaceHubCommand/currentRotation", drivetrain.rotation.inDegrees
    )

    val robotTHub = Transform2d(drivetrain.pose, Pose2d(182.11.inches, 158.84.inches, 0.radians))

    var thetavel =
      thetaPID.calculate(drivetrain.rotation, robotTHub.rotation)

    CustomLogger.recordOutput("FaceHubCommand/thetaveldps", thetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("FaceHubCommand/thetaerror", thetaPID.error.inDegrees)
      val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
      drivetrain.runSpeeds(
        ChassisSpeeds(speed.first, speed.second, thetavel), flipIfRed = false
      )
    }

  override fun isFinished(): Boolean {
    return thetaPID.error < 2.5.degrees
  }

  override fun end(interrupted: Boolean) {

    CustomLogger.recordOutput("FaceHubCommand/interrupted", interrupted)

    drivetrain.runSpeeds(ChassisSpeeds())
    CustomLogger.recordOutput("ActiveCommands/TargetTagCommand", false)
  }
}


