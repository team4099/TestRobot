package com.team4099.robot2025.auto.mode

import com.team4099.robot2025.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand

class ExamplePathAuto(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(WaitCommand(0.5))
  }

  companion object {

    // don't flip pose: poses are robot relative since field frame estimator was reset
  }
}
