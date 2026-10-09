# Decode-Premier-Ivy

Minimal FTC robot project with one autonomous OpMode, **Drive Forward 1m**
(`TeamCode/.../DriveForwardOneMeter.java`). It uses Pedro Pathing with the
goBILDA Pinpoint odometry computer to drive the mecanum robot straight ahead
1 meter and then stop.

Hardware config names: `frontLeft`, `frontRight`, `backLeft`, `backRight`, `pinpoint`.
Drive and localizer tuning lives in `TeamCode/.../pedroPathing/Constants.java`.
