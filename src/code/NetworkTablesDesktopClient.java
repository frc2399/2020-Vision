//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//
//public class NetworkTablesDesktopClient {
//  public static void main(String[] args) {
//    new NetworkTablesDesktopClient().run();
//  }
//
//  public void run() {
//    NetworkTableInstance inst = NetworkTableInstance.getDefault();
//    NetworkTable table = inst.getTable("datatable");
//    NetworkTableEntry xEntry = table.getEntry("horizontalAngle");
//    inst.startClientTeam(2399);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
//    inst.startDSClient();  // recommended if running on DS computer; this gets the robot IP from the DS
//    while (true) {
//      try {
//        Thread.sleep(1000);
//      } catch (InterruptedException ex) {
//        System.out.println("interrupted");
//        return;
//      }
//      xEntry.set();
//    }
//  }
//}