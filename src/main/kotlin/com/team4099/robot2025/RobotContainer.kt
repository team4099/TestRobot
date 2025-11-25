package com.team4099.robot2025

import com.ctre.phoenix6.signals.NeutralModeValue
import com.team4099.robot2025.commands.drivetrain.ResetGyroYawCommand
import com.team4099.robot2025.commands.drivetrain.TeleopDriveCommand
import com.team4099.robot2025.config.ControlBoard
import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.subsystems.drivetrain.Drive
import com.team4099.robot2025.subsystems.drivetrain.GyroIOPigeon2
import com.team4099.robot2025.subsystems.drivetrain.GyroIOSim
import com.team4099.robot2025.subsystems.drivetrain.ModuleIOTalonFXReal
import com.team4099.robot2025.subsystems.drivetrain.ModuleIOTalonFXSim
import com.team4099.robot2025.subsystems.vision.Vision
import com.team4099.robot2025.util.driver.Test
import edu.wpi.first.wpilibj.RobotBase
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import org.littletonrobotics.junction.Logger
import org.team4099.lib.geometry.Pose2d
import org.team4099.lib.smoothDeadband
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.radians

object RobotContainer {
  private val drivetrain: Drive
  private val vision: Vision

  var driveSimulation: SwerveDriveSimulation? = null

  init {
    if (RobotBase.isReal()) {
      drivetrain =
        Drive(
          GyroIOPigeon2,
          ModuleIOTalonFXReal.generateModules(),
          { edu.wpi.first.math.geometry.Pose2d.kZero },
          { pose -> {} }
        )

      vision = Vision(poseSupplier = { drivetrain.pose })
    } else {
      driveSimulation =
        SwerveDriveSimulation(Drive.mapleSimConfig, Pose2d(3.meters, 3.meters, 0.radians).pose2d)
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation)

      drivetrain =
        Drive(
          GyroIOSim(driveSimulation!!.gyroSimulation),
          ModuleIOTalonFXSim.generateModules(driveSimulation!!),
          driveSimulation!!::getSimulatedDriveTrainPose,
          driveSimulation!!::setSimulationWorldPose
        )

      if (Constants.Universal.SIMULATE_VISION) vision = Vision(poseSupplier = { drivetrain.pose })
      else vision = Vision(poseSupplier = { Pose2d() })
    }
  }

  fun mapDefaultCommands() {
    drivetrain.defaultCommand =
      TeleopDriveCommand(
        driver = Test(),
        { ControlBoard.forward.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.strafe.smoothDeadband(Constants.Joysticks.THROTTLE_DEADBAND) },
        { ControlBoard.turn.smoothDeadband(Constants.Joysticks.TURN_DEADBAND) },
        { ControlBoard.slowMode },
        drivetrain,
      )
  }

  fun zeroSensors(isInAutonomous: Boolean = false) {
    drivetrain.pose = Pose2d(drivetrain.pose.x, drivetrain.pose.y, 0.radians)
  }

  fun setDriveBrakeMode(neutralModeValue: NeutralModeValue = NeutralModeValue.Brake) {
    //    drivetrain.configNeutralMode(neutralModeValue)
  }

  fun mapTeleopControls() {
    ControlBoard.resetGyro.whileTrue(ResetGyroYawCommand(drivetrain))
  }

  fun mapTestControls() {}

  fun mapTunableCommands() {}

  fun resetSimulationField() {
    if (!RobotBase.isSimulation()) return

    driveSimulation!!.setSimulationWorldPose(Pose2d(3.meters, 3.meters, 0.radians).pose2d)
    SimulatedArena.getInstance().resetFieldForAuto()
  }

  fun updateSimulation() {
    if (!RobotBase.isSimulation()) return

    SimulatedArena.getInstance().simulationPeriodic()
    Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation!!.simulatedDriveTrainPose)
    Logger.recordOutput(
      "FieldSimulation/Coral", *SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
    )
    Logger.recordOutput(
      "FieldSimulation/Algae", *SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
    )
  }
}
