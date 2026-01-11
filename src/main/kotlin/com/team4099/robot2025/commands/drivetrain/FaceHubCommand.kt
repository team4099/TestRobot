package com.team4099.robot2025.commands.drivetrain

import com.team4099.lib.hal.Clock
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.util.AllianceFlipUtil
import com.team4099.robot2025.util.CustomLogger
import com.team4099.robot2025.util.driver.DriverProfile
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat.N1
import edu.wpi.first.math.Nat.N2
import edu.wpi.first.math.Vector
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.controller.PIDController
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class FaceHubCommand(
  private val drivetrain: Drive,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
) : Command() {

  private val templaunchsped= 10.meters.perSecond //mps
  private val SHOOTER_HEIGHT = 0.meters
  private val HUB_HEIGHT = 1.829.meters
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

    val TOF = ((-templaunchsped.inMetersPerSecond + kotlin.math.sqrt(templaunchsped.inMetersPerSecond.pow(2)+2*Constants.Universal.gravity.inMetersPerSecondPerSecond*(HUB_HEIGHT.inMeters - SHOOTER_HEIGHT.inMeters)))/Constants.Universal.gravity.inMetersPerSecondPerSecond).seconds
    val distanceToHubX = HUB_TRANSLATION.x - drivetrain.pose.translation.x
    val distanceToHubY = HUB_TRANSLATION.y - drivetrain.pose.translation.y
    val driveVector = Vector(Matrix(N2(), N1(), doubleArrayOf(drivetrain.chassisSpeeds.vx.inMetersPerSecond, drivetrain.chassisSpeeds.vy.inMetersPerSecond)))
    val robotTHubVector =Vector(Matrix(N2(), N1(), doubleArrayOf(distanceToHubX.inMeters, distanceToHubY.inMeters)))

    val perpVel = driveVector.minus(driveVector.projection(robotTHubVector))
    val ballDistanceOffset = (perpVel.times(TOF.inSeconds))
    val wantedRot =
      atan2(
        (distanceToHubY.inMeters - ballDistanceOffset.get(1)),
        (distanceToHubX.inMeters - ballDistanceOffset.get(0))
      )
        .radians

    val thetavel = thetaPID.calculate(drivetrain.rotation, wantedRot)
    val ff = thetaPID.error * DrivetrainConstants.PID.SIM_HUB_PID_KV

    val appliedThetavel = thetavel + ff

    CustomLogger.recordOutput("FaceHubCommand/thetaveldps", appliedThetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("FaceHubCommand/thetaerror", thetaPID.error.inDegrees)
    CustomLogger.recordOutput("FaceHubCommand/tof", TOF.inSeconds)
    val speed = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)

    CustomLogger.recordOutput("FaceHubCommand/wantedPose", Pose2d(
      drivetrain.pose.x, drivetrain.pose.y, wantedRot).pose2d
    )

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
