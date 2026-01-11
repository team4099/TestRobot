package com.team4099.robot2025.config.constants

import edu.wpi.first.apriltag.AprilTagFieldLayout
import org.team4099.lib.apriltag.AprilTag
import org.team4099.lib.geometry.Pose3d
import org.team4099.lib.geometry.Rotation3d
import org.team4099.lib.geometry.Translation2d
import org.team4099.lib.geometry.Translation3d
import org.team4099.lib.units.base.inMeters
import org.team4099.lib.units.base.inches
import org.team4099.lib.units.base.meters
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.radians

// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. **All units in Meters** <br></br> <br></br>
 *
 * All translations and poses are stored with the origin at the rightmost point on the BLUE ALLIANCE
 * wall.<br></br> <br></br> Length refers to the *x* direction (as described by wpilib) <br></br>
 * Width refers to the *y* direction (as described by wpilib)
 */
object FieldConstants {
  val customAprilTags =
    listOf(
      AprilTag(
        1,
        Pose3d(
          Translation3d(467.08.inches, 291.79.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        2,
        Pose3d(
          Translation3d(468.56.inches, 182.08.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 90.degrees)
        )
      ),
      AprilTag(
        3,
        Pose3d(
          Translation3d(444.80.inches, 172.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        4,
        Pose3d(
          Translation3d(444.80.inches, 158.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        5,
        Pose3d(
          Translation3d(468.56.inches, 134.56.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 270.degrees)
        )
      ),
      AprilTag(
        6,
        Pose3d(
          Translation3d(467.08.inches, 24.85.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        7,
        Pose3d(
          Translation3d(470.03.inches, 24.85.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        8,
        Pose3d(
          Translation3d(482.56.inches, 134.56.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 270.degrees)
        )
      ),
      AprilTag(
        9,
        Pose3d(
          Translation3d(157.79.inches, 144.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        10,
        Pose3d(
          Translation3d(492.33.inches, 158.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        11,
        Pose3d(
          Translation3d(482.56.inches, 182.08.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 90.degrees)
        )
      ),
      AprilTag(
        12,
        Pose3d(
          Translation3d(470.03.inches, 291.79.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        13,
        Pose3d(
          Translation3d(649.58.inches, 291.02.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        14,
        Pose3d(
          Translation3d(649.58.inches, 274.02.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        15,
        Pose3d(
          Translation3d(649.57.inches, 169.78.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        16,
        Pose3d(
          Translation3d(649.57.inches, 152.78.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        17,
        Pose3d(
          Translation3d(183.03.inches, 24.85.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        18,
        Pose3d(
          Translation3d(181.56.inches, 134.56.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 270.degrees)
        )
      ),
      AprilTag(
        19,
        Pose3d(
          Translation3d(205.32.inches, 144.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        20,
        Pose3d(
          Translation3d(205.32.inches, 158.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        21,
        Pose3d(
          Translation3d(181.56.inches, 182.08.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 90.degrees)
        )
      ),
      AprilTag(
        22,
        Pose3d(
          Translation3d(183.03.inches, 291.79.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        23,
        Pose3d(
          Translation3d(180.08.inches, 291.79.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        24,
        Pose3d(
          Translation3d(167.56.inches, 182.08.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 90.degrees)
        )
      ),
      AprilTag(
        25,
        Pose3d(
          Translation3d(157.79.inches, 172.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        26,
        Pose3d(
          Translation3d(157.79.inches, 158.32.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        27,
        Pose3d(
          Translation3d(167.56.inches, 134.56.inches, 44.25.inches),
          Rotation3d(0.radians, 0.radians, 270.degrees)
        )
      ),
      AprilTag(
        28,
        Pose3d(
          Translation3d(180.08.inches, 24.85.inches, 35.00.inches),
          Rotation3d(0.radians, 0.radians, 180.degrees)
        )
      ),
      AprilTag(
        29,
        Pose3d(
          Translation3d(0.54.inches, 25.62.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        30,
        Pose3d(
          Translation3d(0.54.inches, 42.62.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        31,
        Pose3d(
          Translation3d(0.55.inches, 146.86.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      ),
      AprilTag(
        32,
        Pose3d(
          Translation3d(0.55.inches, 163.86.inches, 21.75.inches),
          Rotation3d(0.radians, 0.radians, 0.degrees)
        )
      )
    )

  val customFieldLayout =
    AprilTagFieldLayout(
      customAprilTags.map { it.apriltagWpilib }, 651.2.inches.inMeters, 317.7.inches.inMeters
    )

  val fieldLength = customFieldLayout.fieldLength.meters
  val fieldWidth = customFieldLayout.fieldWidth.meters

  object REEF {
    val blue_center: Translation2d = Translation2d(176.746.inches, fieldWidth / 2.0)
    val red_center: Translation2d = Translation2d(fieldLength - 176.746.inches, fieldWidth / 2.0)
  }
}
