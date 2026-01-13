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
import org.team4099.lib.geometry.Transform2d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.kinematics.ChassisSpeeds
import org.team4099.lib.units.Velocity
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inSeconds
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.base.seconds
import org.team4099.lib.units.derived.Radian
import org.team4099.lib.units.derived.cos
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.inDegrees
import org.team4099.lib.units.derived.inRotation2ds
import org.team4099.lib.units.derived.radians
import org.team4099.lib.units.derived.sin
import org.team4099.lib.units.derived.tan
import org.team4099.lib.units.inDegreesPerSecond
import org.team4099.lib.units.inMetersPerSecond
import org.team4099.lib.units.inMetersPerSecondPerSecond
import org.team4099.lib.units.perSecond
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt
import edu.wpi.first.math.geometry.Translation2d as WPITranslation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds as WPIChassisSpeeds
import kotlin.math.absoluteValue
import kotlin.math.min

/** @author Nathan Arega, Ryan Chung */
class FaceHubCommand(
  private val drivetrain: Drive,
  val driveX: () -> Double,
  val driveY: () -> Double,
  val slowMode: () -> Boolean,
  val driver: DriverProfile
) : Command() {
  // TODO replace with real
  private var launchSpeedZ = 10.meters.perSecond // mps
  private val SHOOTER_HEIGHT = 14.876.inches
  private val SHOOTER_ANGLE = 70.degrees
  private val HUB_HEIGHT = 72.inches
  private val thetaPID: PIDController<Radian, Velocity<Radian>>
  private var HUB_TRANSLATION: Translation2d =
    AllianceFlipUtil.apply(Translation2d(182.11.inches, 158.84.inches))
  var hasAligned: Boolean = false

  val MAX_VELOCITY_RADIUS = 1.5

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

    HUB_TRANSLATION = AllianceFlipUtil.apply(Translation2d(182.11.inches, 158.84.inches))
  }

  override fun execute() {
    // this command is complicated so we have some comments to explain it
    CustomLogger.recordOutput("ActiveCommands/FaceHubCommand", true)

    // Calculate the distance to the HUB
    val distanceToHubX = HUB_TRANSLATION.x - drivetrain.pose.translation.x
    val distanceToHubY = HUB_TRANSLATION.y - drivetrain.pose.translation.y
    val distanceToHubMag =
      sqrt(distanceToHubX.inMeters.pow(2) + distanceToHubY.inMeters.pow(2)).meters

    /**
     * Time for the math...
     *
     * We must solve for v_launch, but that depends on the distance the ball
     * will be displaced by the drivetrain velocity times the time of
     * flight. The time of flight is, unfortunately, also based off of
     * v_launch. We combine the following kinematics equations to derive
     * v_launch independently from time of flight.
     *
     * Variables:
     * - v_launch   = Launch velocity, represented as a number (vector) in 2D.
     *                Multiply by SHOOTER_ANGLE.cos or SHOOTER_ANGLE.sin for
     *                v_launch_x or v_launch_y.
     * - h          = Height of the hub.
     * - s          = Height of the shooter.
     * - d          = Distance robotTHub.
     * - v_parallel = Movement of drivetrain, orthogonal to the robotTHub vector.
     *
     * From the following kinematics equations:
     *  - d = (v_launch * SHOOTER_ANGLE.cos + v_parallel) * t   (1)
     *  - h = s + (v_launch * SHOOTER_ANGLE.sin) * t + (-g / 2) * t^2   (2)
     *
     *  Multiply (1) by (v_launch * SHOOTER_ANGLE.sin)
     *  - (v_launch * SHOOTER_ANGLE.sin) * d = (v_launch * SHOOTER_ANGLE.sin) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) * t   (3)
     *
     *  From (2):
     *  - (v_launch * SHOOTER_ANGLE.sin) * t = (s - h) + (g / 2) * t^2   (4)
     *
     *  Plug (4) into (3):
     *  - (v_launch * SHOOTER_ANGLE.sin) * d = ((s - h) + (g / 2) * t^2) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)   (5)
     *
     *  From (1):
     *  - t = d / (v_launch * SHOOTER_ANGLE.cos + v_parallel)   (6)
     *
     *  Substitute (6) into (5):
     *  - (v_launch * SHOOTER_ANGLE.sin) * d = = ((s - h) + (g / 2) * (d / (v_launch * SHOOTER_ANGLE.cos + v_parallel))^2) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)
     *  - (v_launch * SHOOTER_ANGLE.sin) * d = (s - h) * (v_launch * SHOOTER_ANGLE.cos + v_parallel) + g * d^2 / (2 * (v_launch * SHOOTER_ANGLE.cos + v_p))   (7)
     *
     *  Multiply by (v_launch * SHOOTER_ANGLE.cos + v_parallel):
     *  - v_launch * SHOOTER_ANGLE.sin * d * (v_launch * SHOOTER_ANGLE.cos + v_parallel) = (s - h) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)^2 + g * d^2 / 2
     *  - 0 = (s - h) * (v_launch * SHOOTER_ANGLE.cos + v_parallel)^2 + g * d^2 / 2 - v_launch * SHOOTER_ANGLE.sin * d * (v_launch * SHOOTER_ANGLE.cos + v_parallel)
     *  - 0 = (s - h) * (v_launch * SHOOTER_ANGLE.cos)^2 + (s - h) * v_parallel^2 + (s - h) * (2 * v_launch * SHOOTER_ANGLE.cos * v_parallel) + g * d^2 / 2 - v_launch^2 * SHOOTER_ANGLE.sin * d * SHOOTER_ANGLE.cos - v_launch * SHOOTER_ANGLE.sin * d * v_parallel
     *  - 0 =
     *        (s - h) * (SHOOTER_ANGLE.cos)^2 * v_launch^2                (Quadratic)
     *        - SHOOTER_ANGLE.sin * d * SHOOTER_ANGLE.cos * v_launch^2    (Quadratic term)
     *        + (s - h) * 2 * SHOOTER_ANGLE.cos * v_parallel * v_launch   (Linear term)
     *        - SHOOTER_ANGLE.sin * d * v_parallel * v_launch             (Linear term)
     *        + (s - h) * v_parallel^2                                    (Constant)
     *        + g * d^2 / 2                                               (Constant)
     *
     *  We know have a quadratic in terms of v_launch.
     *  - A = (s - h) * (SHOOTER_ANGLE.cos)^2 - SHOOTER_ANGLE.sin * SHOOTER_ANGLE.cos * d
     *  - B = 2 * (s - h) * SHOOTER_ANGLE.cos * v_parallel - SHOOTER_ANGLE.sin * d * v_parallel
     *  - C = (s - h) * v_parallel^2 + g * d^2 / 2
     */

    // Get field-relative drivetrain velocity, and convert it into a vector.
    val fieldSpeeds =
      ChassisSpeeds(
        WPIChassisSpeeds.fromRobotRelativeSpeeds(
          drivetrain.chassisSpeeds.chassisSpeedsWPILIB, drivetrain.rotation.inRotation2ds
        )
      )
    val driveVector =
      Vector(
        Matrix(
          N2(),
          N1(),
          doubleArrayOf(fieldSpeeds.vx.inMetersPerSecond, fieldSpeeds.vy.inMetersPerSecond)
        )
      )

    val robotTHubVector =
      Vector(Matrix(N2(), N1(), doubleArrayOf(distanceToHubX.inMeters, distanceToHubY.inMeters)))

    // The drive vector is split into two (basis) vectors, one in the
    // direction of the hub and one orthogonal to it.
    val parallelVector = driveVector.projection(robotTHubVector)
    val perpendicularVel = driveVector.minus(parallelVector)
    // Gives the magnitude of the parallel vector, but signed
    val parallelScalar = driveVector.dot(robotTHubVector) / distanceToHubMag.inMeters

    val v_launch_a = (SHOOTER_HEIGHT.inMeters - HUB_HEIGHT.inMeters) * SHOOTER_ANGLE.cos.pow(2) - SHOOTER_ANGLE.sin * SHOOTER_ANGLE.cos * distanceToHubMag.inMeters
    val v_launch_b = 2 * (SHOOTER_HEIGHT.inMeters - HUB_HEIGHT.inMeters) * SHOOTER_ANGLE.cos * parallelScalar - SHOOTER_ANGLE.sin * distanceToHubMag.inMeters * parallelScalar
    val v_launch_c = (SHOOTER_HEIGHT.inMeters - HUB_HEIGHT.inMeters) * parallelScalar.pow(2) + Constants.Universal.gravity.inMetersPerSecondPerSecond * distanceToHubMag.inMeters.pow(2) / 2.0

    val v_launch_res_1 = (-v_launch_b + sqrt(v_launch_b.pow(2) - 4.0 * v_launch_a * v_launch_c)) / (2 * v_launch_a)
    val v_launch_res_2 = (-v_launch_b - sqrt(v_launch_b.pow(2) - 4.0 * v_launch_a * v_launch_c)) / (2 * v_launch_a)

//    val launchSpeed = when {
//      v_launch_res_1 > 0 && v_launch_res_2 > 0 -> min(v_launch_res_1, v_launch_res_2)
//      v_launch_res_1 > 0 -> v_launch_res_1
//      else -> v_launch_res_2
//    }.meters.perSecond

    val launchSpeed = max(v_launch_res_1, v_launch_res_2).meters.perSecond

    val launchSpeedZ = launchSpeed * SHOOTER_ANGLE.sin
    val launchSpeedField = launchSpeed * SHOOTER_ANGLE.cos


    CustomLogger.recordOutput("FaceHubCommand/distanceToHubMag", distanceToHubMag.inMeters)
    CustomLogger.recordOutput("FaceHubCommand/launchSpeedZ", launchSpeedZ.inMetersPerSecond)
    /*
    // Simple linear regression model to calculate the launch speed
    // based off the distance to the hub. Numbers are taken from CAD.
    launchSpeedZ = (1.00805 * distanceToHubMag.inMeters + 4.42681).meters.perSecond
     */
    // Solve the kinematics formula for t (time)
    val timeOfFlight = //TODO IMLPEMENT TMR

    // The distance the ball travels while in the air
    val ballDistanceOffset = (perpendicularVel.times(timeOfFlight.inSeconds))

    CustomLogger.recordOutput(
      "FaceHubCommand/shouldAimIfAccountingForBall",
      Pose2d(HUB_TRANSLATION, 0.radians)
        .transformBy(
          Transform2d(
            Translation2d(
              ballDistanceOffset.get(0).meters, ballDistanceOffset.get(1).meters
            ),
            0.radians
          )
            .inverse()
        )
        .pose2d
    )

    // The wanted rotation is recieved by offsetting the usual angle
    // (the slope connecting the robot and the HUB) by the offset the
    // ball would travel in the air.
    val wantedRot =
      atan2(
        distanceToHubY.inMeters - ballDistanceOffset.get(1),
        distanceToHubX.inMeters - ballDistanceOffset.get(0)
      )
        .radians

    // PID and clamping of the calculated theta velocity
    val thetavel = thetaPID.calculate(drivetrain.rotation, wantedRot)
    val appliedThetavel = thetavel // clamp(thetavel, -90.degrees.perSecond, 90.degrees.perSecond)

    CustomLogger.recordOutput("FaceHubCommand/thetaveldps", appliedThetavel.inDegreesPerSecond)
    CustomLogger.recordOutput("FaceHubCommand/thetaerror", thetaPID.error.inDegrees)
    CustomLogger.recordOutput("FaceHubCommand/tof", timeOfFlight.inSeconds)

    // Take the drivers speed being inputted, and clamp the magnitude
    // of the drive vector to < MAX_VELOCITY_RADIUS meters per second
    var (speedX, speedY) = driver.driveSpeedClampedSupplier(driveX, driveY, slowMode)
    val speedMagnitude = sqrt(speedX.inMetersPerSecond.pow(2) + speedY.inMetersPerSecond.pow(2))

    if (speedMagnitude > MAX_VELOCITY_RADIUS) {
      // Convert to unit vector and then * MAX_VELOCITY_RADIUS
      speedX = speedX / speedMagnitude * MAX_VELOCITY_RADIUS
      speedY = speedY / speedMagnitude * MAX_VELOCITY_RADIUS
    }

    CustomLogger.recordOutput(
      "FaceHubCommand/wantedPose", Pose2d(drivetrain.pose.x, drivetrain.pose.y, wantedRot).pose2d
    )

    drivetrain.runSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds(
        speedX, speedY, appliedThetavel, drivetrain.pose.rotation
      )
    )

    hasAligned = thetaPID.error.absoluteValue < 6.7.degrees

    CustomLogger.recordOutput("FaceHubCommand/hasAligned", hasAligned)

    if (RobotBase.isSimulation() &&
      (hasAligned || distanceToHubMag < 2.meters) &&
      Clock.fpgaTime.inSeconds % .25 < 0.05
    )
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
            drivetrain.pose.translation.translation2d,
            WPITranslation2d(
              fieldSpeeds.vx.inMetersPerSecond, fieldSpeeds.vy.inMetersPerSecond
            ) +
              WPITranslation2d(launchSpeedField.inMetersPerSecond, 0.0)
                .rotateBy(drivetrain.rotation.inRotation2ds),
            SHOOTER_HEIGHT.inMeters,
            launchSpeedZ.inMetersPerSecond,
            Rotation3d.kZero
          )
        )
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
