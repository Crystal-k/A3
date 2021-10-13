package sysu.sdcs.sensordatacollector;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by justk on 2018/6/13.
 */

public class SensorData {
    public static List<String[]> gyroscopeSensorData = new ArrayList<String[]>();
    public static List<String[]> accelerometerSensorData = new ArrayList<String[]>();
    public static List<String[]> magneticFieldSensorData = new ArrayList<String[]>();
    public static List<String[]> gravitySensorData = new ArrayList<String[]>();

    public static List<String[]> linearAccelerationSensorData = new ArrayList<String[]>();
    public static List<String[]> rotationVectorSensorData = new ArrayList<String[]>();

    public static List<String[]> orientationSensorData = new ArrayList<String[]>();

    public static List<String> timestamps = new ArrayList<String>();

    public static List<float[]> eulerAngleCData = new ArrayList<>();
    public static List<float[]> eulerAngleGData = new ArrayList<>();
    public static List<float[]> eulerAngleData = new ArrayList<>();
    public static List<Float> eulerYawData = new ArrayList<>();
    public static List<Float> eulerCYawData = new ArrayList<>();
    public static List<Float> eulerGYawData = new ArrayList<>();


    public static void clear(){
        gyroscopeSensorData.clear();
        accelerometerSensorData.clear();
        magneticFieldSensorData.clear();
        gravitySensorData.clear();

        linearAccelerationSensorData.clear();
        rotationVectorSensorData.clear();

        orientationSensorData.clear();

        timestamps.clear();

        eulerAngleCData.clear();
        eulerAngleGData.clear();
        eulerAngleData.clear();
        eulerYawData.clear();
        eulerCYawData.clear();
        eulerGYawData.clear();
    }

    public static void addSensorData(String[] gyroData, String[] aData, String[] mData, String gravData[],
                                     String[] lData, String[] rData, String[] oData, String captime,
                                     float[] eu_angle_c, float eu_c_yaw, float[] eu_angle_g, float eu_g_yaw, float[] eu_angle, float eu_yaw){
        gyroscopeSensorData.add(gyroData);
        accelerometerSensorData.add(aData);
        magneticFieldSensorData.add(mData);
        gravitySensorData.add(gravData);

        linearAccelerationSensorData.add(lData);
        rotationVectorSensorData.add(rData);

        orientationSensorData.add(oData);

        timestamps.add(captime);

        eulerAngleCData.add(eu_angle_c);
        eulerAngleGData.add(eu_angle_g);
        eulerAngleData.add(eu_angle);
        eulerYawData.add(eu_yaw);
        eulerCYawData.add(eu_c_yaw);
        eulerGYawData.add(eu_g_yaw);
    }

    public static String getFileHead(){
        return  "frame,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z," +
                "gravity_x,gravity_y,gravity_z," +
                "linear_x,linear_y,linear_z,rotation_x,rotation_y,rotation_z,rotation_w," +
                "orientation_yaw, orientation_pitch, orientation_roll," +
                "timestamp," +
                "eu_c_angle_yaw, eu_c_angle_pitch, eu_c_angle_roll," +
                "eu_c_yaw," +
                "eu_g_angle_yaw, eu_g_angle_pitch, eu_g_angle_roll," +
                "eu_g_yaw," +
                "eu_angle_yaw, eu_angle_pitch, eu_angle_roll," +
                "eu_yaw" +
                "\n";
    }

    public static String getAllDataStr(){
        String data = "";
        for(int i = 0; i < magneticFieldSensorData.size() ; i++){
            String[] gyro = gyroscopeSensorData.get(i);
            String[] acc = accelerometerSensorData.get(i);
            String[] mag = magneticFieldSensorData.get(i);
            String[] gravity = gravitySensorData.get(i);

            String[] linear = linearAccelerationSensorData.get(i);
            String[] rotation = rotationVectorSensorData.get(i);

            String[] orientation = orientationSensorData.get(i);

            float[] eu_angle_c = eulerAngleCData.get(i);
            float[] eu_angle_g = eulerAngleGData.get(i);
            float[] eu_angle = eulerAngleData.get(i);
            float eu_yaw = eulerYawData.get(i);
            float eu_c_yaw = eulerCYawData.get(i);
            float eu_g_yaw = eulerGYawData.get(i);


            String one_detail = "" + (i+1) + ","
                    + gyro[0] + "," + gyro[1] + "," + gyro[2] + ","
                    + acc[0] + "," + acc[1] + "," + acc[2] + ","
                    + mag[0] + "," + mag[1] + "," + mag[2] + ","
                    + gravity[0] + "," + gravity[1] + "," + gravity[2] + ","
                    + linear[0] + "," + linear[1] + "," + linear[2] + ","
                    + rotation[0] + "," + rotation[1] + "," + rotation[2] + "," + rotation[3] + ","
                    + orientation[0] + "," + orientation[1] + "," + orientation[2] + ","
                    + timestamps.get(i) + ","
                    + eu_angle_c[0] + "," + eu_angle_c[1] + "," + eu_angle_c[2] + ","
                    + eu_c_yaw + ","
                    + eu_angle_g[0] + "," + eu_angle_g[1] + "," + eu_angle_g[2] + ","
                    + eu_g_yaw + ","
                    + eu_angle[0] + "," + eu_angle[1] + "," + eu_angle[2] + ","
                    + eu_yaw + "\n" ;
            data = data + one_detail;
        }

        return data;
    }

    public static String null2zero(String item){
        if(item == null || item.equals(""))
            return "0";
        return item;
    }

}
