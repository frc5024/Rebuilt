// package frc.robot.commands.Turret;

// import edu.wpi.first.wpilibj2.command.Command;

// //import com.kauailabs.navx.frc.AHRS;

// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.LimelightResults;
// import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Turret;
// import edu.wpi.first.networktables.NetworkTableInstance;

// public class LockontoTargetCommand extends Command {

//     private final Turret turretSubsystem; 
//     // private final Limelight limelightSubsystem;

//     double tx = LimelightHelpers.getTX("");

//     LimelightResults results = LimelightHelpers.getLatestResults("");

//     // LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
//     // double id = tag.fiducialID;    

//     boolean hasTarget = LimelightHelpers.getTV("");


//     public LockontoTargetCommand(Turret turretSubsystem) {
//         this.turretSubsystem = turretSubsystem;
//         // this.limelightSubsystem = limelightSubsystem;
//         // this.tx = tx;

//         addRequirements(turretSubsystem);
//     }

    

//     @Override
//     public void initialize() {
        

//         var limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
//         double tv = limelightTable.getEntry("tv").getDouble(0.0);
//         double tx = limelightTable.getEntry("tx").getDouble(0.0);

//         if(tv == 1) {
//             // double robotHeading = turretSubsystem.getRobotHeading();

//             // double turretTargetAngle = robotHeading + tx;

//             //turretSubsystem.setTargetAngle(turretTargetAngle);
//         } else {
//             //turretSubsystem.setTargetAngle(turretSubsystem.getTurretAngle());
//         }

//     }

//     public void execute() {

//         //System.out.println("BUTTONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
//         boolean hasTarget = LimelightHelpers.getTV("");
//         System.out.println(hasTarget);

        
//         // System.out.println(limelightSubsystem.getAprilTagID());

//         // if(limelightSubsystem.isTargetVisible() == true) {
//         //     System.out.println("YAYYYYYYYYYYYYYYYbYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY");
//         // }
//     }

//     @Override
//     public boolean isFinished() {
//         // Finish when the turret is locked to the target angle
//         return turretSubsystem.isAtTargetAngle();
//     }
// }
