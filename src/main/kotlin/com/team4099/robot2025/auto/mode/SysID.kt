package com.team4099.robot2025.auto.mode

import choreo.Choreo
import choreo.trajectory.SwerveSample
import com.team4099.robot2025.commands.SysIdCommand
import com.team4099.robot2025.commands.drivetrain.FollowChoreoPath
import com.team4099.robot2025.subsystems.drivetrain.Drive
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.team4099.lib.geometry.Pose2d

class SysID(val drivetrain: Drive) : SequentialCommandGroup() {
  init {
    addRequirements(drivetrain)

    addCommands(
      WaitCommand(0.5),
      drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
    )
  }

  companion object {
  }
}
