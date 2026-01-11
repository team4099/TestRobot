package com.team4099.robot2025.config

import com.team4099.robot2025.config.constants.Constants
import com.team4099.robot2025.config.constants.DrivetrainConstants
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team4099.lib.joystick.XboxOneGamepad
import kotlin.math.absoluteValue

/**
 * Maps buttons on the driver and operator controllers to specific actions with meaningful variable
 * names.
 */
object ControlBoard {

  private val driver = XboxOneGamepad(Constants.Joysticks.DRIVER_PORT)
  private val operator = XboxOneGamepad(Constants.Joysticks.SHOTGUN_PORT)
  private val technician = XboxOneGamepad(Constants.Joysticks.TECHNICIAN_PORT)

  val strafe: Double
    get() = -driver.leftXAxis

  val forward: Double
    get() = -driver.leftYAxis

  val turn: Double
    get() {
      return if (driver.rightXAxis.absoluteValue < 0.90) {
        driver.rightXAxis * DrivetrainConstants.TELEOP_TURNING_SPEED_PERCENT
      } else {
        driver.rightXAxis
      }
    }

  val slowMode: Boolean
    get() = driver.rightJoystickButton

  val resetGyro = Trigger { driver.startButton && driver.selectButton }

  val testCommand = Trigger { driver.rightTriggerAxis > .5 }
}
