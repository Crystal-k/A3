package sysu.sdcs.sensordatacollector;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
//import android.support.annotation.NonNull;
import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
//import android.support.v7.app.AppCompatActivity;

import android.os.Bundle;
import android.text.method.ScrollingMovementMethod;
import android.util.Log;
import android.view.View;
import android.view.inputmethod.InputMethodManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.lang.Math;


public class MainActivity extends AppCompatActivity implements SensorEventListener{

    private static final int REQ_CODE_PERMISSION_EXTERNAL_STORAGE = 0x1111;
    private static final int REQ_CODE_PERMISSION_SENSOR = 0x2222;

    private SensorManager sensorManager;

    private Sensor gyroscopeSensor;
    private Sensor accelerometerSensor;
    private Sensor magneticFieldSensor;
    private Sensor gravitySensor;

    private Sensor linearAccelerationSensor;
    private Sensor rotationVectorSensor;

    private Sensor orientationSensor;

    private Button btn_control;
    private EditText edt_path;
    private TextView tv_state;
    private TextView tv_record;

    private ScheduledFuture future;
    private String file_name = "";
    private String cap_records = "";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        init();
        btn_control.setOnClickListener(btn_listener);

//        TextView textview = (TextView) findViewById(R.id.degree);
//        textview.setMovementMethod(ScrollingMovementMethod.getInstance());
//        textview.append("onCreate\n");
    }

    public void init(){
        btn_control = findViewById(R.id.btn_control);
        edt_path = findViewById(R.id.edt_pathID);
        tv_state = findViewById(R.id.state);
        tv_record = findViewById(R.id.record);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        gyroscopeSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        accelerometerSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magneticFieldSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

        linearAccelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        rotationVectorSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        orientationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
//        permissionCheck();
    }

    public void permissionCheck(){
        if(ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED){
            //申请WRITE_EXTERNAL_STORAGE权限
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
                    REQ_CODE_PERMISSION_EXTERNAL_STORAGE);
        }
        if(ContextCompat.checkSelfPermission(this, Manifest.permission.BODY_SENSORS)
                != PackageManager.PERMISSION_GRANTED){
            //申请BODY_SENSOR权限
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.BODY_SENSORS},
                    REQ_CODE_PERMISSION_SENSOR);
        }
    }

    //权限申请
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        switch (requestCode) {
            case REQ_CODE_PERMISSION_EXTERNAL_STORAGE: {
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // User agree the permission
                } else {
                    // User disagree the permission
                    Toast.makeText(MainActivity.this, "请打开存储权限", Toast.LENGTH_LONG).show();
                }
            }
            case REQ_CODE_PERMISSION_SENSOR: {
                if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // User agree the permission
                }
                else {
                    // User disagree the permission
                    Toast.makeText(this, "请打开传感器权限", Toast.LENGTH_LONG).show();
                }
            }
            break;
        }
    }

    private View.OnClickListener btn_listener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            if(edt_path.getText().toString().equals("") ||
                    edt_path.getText().toString() == null) {
                Toast.makeText(MainActivity.this, "path ID 不能为空", Toast.LENGTH_SHORT).show();
            }
            else if(btn_control.getText().toString().equals("start")){
                InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
                if (imm != null) {
                    imm.hideSoftInputFromWindow(getWindow().getDecorView().getWindowToken(), 0);
                }

                tv_state.setText("传感器数据正在采集中\n" + "当前采集路径为：path-" + edt_path.getText().toString());
                btn_control.setText("stop");
//                file_name = "path " + edt_path.getText().toString() + "-" + (UUIDUtil.generateRandomString(4))+ ".csv";
                file_name = "path " + edt_path.getText().toString() + "-" + getCurrentTime() + ".csv";
                FileUtil.saveSensorData(file_name, SensorData.getFileHead());
                ScheduledExecutorService service = Executors.newScheduledThreadPool(5);
                future = service.scheduleAtFixedRate(new DataSaveTask(file_name), 5, 5, TimeUnit.SECONDS);
            }
            else{
                future.cancel(true);
                if(FileUtil.saveSensorData(file_name, SensorData.getAllDataStr())){
                    cap_records += file_name + "\n";
                    tv_record.setText(cap_records);
                    tv_state.setText("");
                    Toast.makeText(MainActivity.this, "传感器数据保存成功", Toast.LENGTH_SHORT).show();
                }
                else
                    Toast.makeText(MainActivity.this, "传感器数据保存失败", Toast.LENGTH_SHORT).show();
                SensorData.clear();
                btn_control.setText("start");
                tv_state.setText("点击按钮开始采集\n");
            }
        }
    };


    // == SensorEventListener =================================
    private static int[] SYNC = {0,0,0,0,0,0,0};

    private String[] gyroscopeData;
    private String[] accelerometerData;
    private String[] magneticFieldData;
    private String[] gravityData;

    private String[] linearAccelerationData;
    private String[] rotationVectorData;

    private String[] orientationData;

    protected void onResume() {
        super.onResume();
        if(!sensorManager.registerListener(this, gyroscopeSensor, SensorManager.SENSOR_DELAY_FASTEST))
            Toast.makeText(MainActivity.this, "陀螺仪不可用", Toast.LENGTH_SHORT).show();
        if(!sensorManager.registerListener(this, accelerometerSensor, SensorManager.SENSOR_DELAY_FASTEST ))
            Toast.makeText(MainActivity.this, "加速度传感器不可用", Toast.LENGTH_SHORT).show();
        if(!sensorManager.registerListener(this, magneticFieldSensor, SensorManager.SENSOR_DELAY_FASTEST))
            Toast.makeText(MainActivity.this, "磁场传感器不可用", Toast.LENGTH_SHORT).show();
        if(!sensorManager.registerListener(this, gravitySensor, SensorManager.SENSOR_DELAY_FASTEST))
            Toast.makeText(MainActivity.this, "重力传感器不可用", Toast.LENGTH_SHORT).show();
        if(!sensorManager.registerListener(this, linearAccelerationSensor, SensorManager.SENSOR_DELAY_FASTEST))
            Toast.makeText(MainActivity.this, "线性加速度传感器不可用", Toast.LENGTH_SHORT).show();
        if(!sensorManager.registerListener(this, rotationVectorSensor, SensorManager.SENSOR_DELAY_FASTEST))
            Toast.makeText(MainActivity.this, "旋转矢量传感器不可用", Toast.LENGTH_SHORT).show();
        if(!sensorManager.registerListener(this, orientationSensor, SensorManager.SENSOR_DELAY_FASTEST))
            Toast.makeText(MainActivity.this, "方向传感器不可用", Toast.LENGTH_SHORT).show();
//
//        TextView textview = (TextView) findViewById(R.id.degree);
//        textview.append("onResume\n");
    }

    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    public boolean sync(){
        for(int i = 0 ; i < SYNC.length ; i++){
            if(SYNC[i] == 0)
                return false;
        }
        return true;
    }

    public void reset(){
        for(int i = 0 ; i < SYNC.length ; i++){
            SYNC[i] = 0;
        }
    }

    public String getCurrentTime(){
        return new SimpleDateFormat("yyyyMMdd_HHmmss_SSS").format(new Date());
    }


    // == A3 =================================
    //加速度传感器数据
    public float accValues[] = new float[3];
    //地磁传感器数据
    public float magValues[] = new float[3];
    //旋转矩阵
    public float r_c[] = new float[9];

    //旋转矢量传感器数据
    public float rotaVecValues[] = new float[4];
    public float r_g[] = new float[9];

    private static final int UNBOUNDED = 9999;
    private float groValues[] = new float[3];
    private float linearValues[] = new float[3];
    private float gravityValues[] = new float[3];
    private double omega;
    private double linearAcc;
    private double g;

    // Create a constant to convert nanoseconds to seconds.
    private static final float NS2S = 1.0f / 1000000000.0f;
    private double timestamp = 0;
    private double eG = 0;  //陀螺仪姿态估计累计误差
    private double eg = 0;

    private double tW=0;
    private static final float DETECTION_WINDOW = (float) 2; //检测窗口大小（s）
    float eulerAngle_c[] = new float[3];
    private static List<Float> cYaw = new ArrayList<>();
    private static List<Float> cPitch = new ArrayList<>();
    private static List<Float> cRoll = new ArrayList<>();

    float eulerAngle_g[] = new float[3];
    private static List<float[]> gEuler = new ArrayList<>();
    private static List<Float> gYaw = new ArrayList<>();
    private static List<Float> gPitch = new ArrayList<>();
    private static List<Float> gRoll = new ArrayList<>();

    private int windowStart_c = 0;
    private int windowStart_1 = 0;
    private int windowStart_2 = 0;
    private static final int WC = 0;
    private static final int W1= 1;
    private static final int W2 = 2;
    private float pc = 0;
    private float pg = 0;

    private double eC = 0;  //及时姿态估计误差
    private double e1 = 0;
    private double e2 = 0;

    public float r_f[] = new float[9];
    float eulerAngle[] = new float[3];

//    public int flag = 1;
//    public int change = 0;
//
//    public float rotaVecValues_f[] = new float[4];
//    public float r_g_f[] = new float[9];
//    public float r_test[] = new float[9];
//    public float eulerAngle_test[] = new float[3];
//    public float[] daertaRotationMatrix = new float[9];


    @Override
    public void onSensorChanged(SensorEvent event){
        switch(event.sensor.getType()){
            case Sensor.TYPE_GYROSCOPE:{
                groValues = event.values.clone();
                omega = calMagnitude(groValues);
                omega = (omega/(2*Math.PI))*360;
//            System.out.println("omega: "+ omega);

                gyroscopeData = new String[3];
                gyroscopeData[0] = ""+event.values[0];
                gyroscopeData[1] = ""+event.values[1];
                gyroscopeData[2] = ""+event.values[2];
                SYNC[0] = 1;
//                Log.d("capsensordata_gyro", gyroscopeData[1]);
                break;
            }
            case Sensor.TYPE_ACCELEROMETER:{
                accValues = event.values.clone();

                accelerometerData = new String[3];
                accelerometerData[0] = ""+event.values[0];
                accelerometerData[1] = ""+event.values[1];
                accelerometerData[2] = ""+event.values[2];
                SYNC[1] = 1;
//                Log.d("capsensordata_a", accelerometerData[1]);
                break;
            }
            case Sensor.TYPE_MAGNETIC_FIELD:{
                magValues = event.values.clone();

                magneticFieldData = new String[3];
                magneticFieldData[0] = ""+event.values[0];
                magneticFieldData[1] = ""+event.values[1];
                magneticFieldData[2] = ""+event.values[2];
                SYNC[2] = 1;
//                Log.d("capsensordata_m", magneticFieldData[1]);
                break;
            }
            case Sensor.TYPE_GRAVITY:{
                gravityValues = event.values.clone();
                g = calMagnitude(gravityValues);
//            System.out.println("g: "+ g);

                gravityData = new String[3];
                gravityData[0] = ""+event.values[0];
                gravityData[1] = ""+event.values[1];
                gravityData[2] = ""+event.values[2];
                SYNC[3] = 1;
//                Log.d("capsensordata_grav", gravityData[1]);
                break;
            }
            case Sensor.TYPE_LINEAR_ACCELERATION:{
                linearValues = event.values.clone();
                linearAcc = calMagnitude(linearValues);
//            System.out.println("linear_acc: "+ linearAcc);

                linearAccelerationData = new String[3];
                linearAccelerationData[0] = ""+event.values[0];
                linearAccelerationData[1] = ""+event.values[1];
                linearAccelerationData[2] = ""+event.values[2];
                SYNC[4] = 1;
//                Log.d("capsensordata_l", linearAccelerationData[1]);
                break;
            }
            case Sensor.TYPE_ROTATION_VECTOR:{
                rotaVecValues = event.values.clone();

                rotationVectorData = new String[4];
                rotationVectorData[0] = ""+event.values[0];
                rotationVectorData[1] = ""+event.values[1];
                rotationVectorData[2] = ""+event.values[2];
                rotationVectorData[3] = ""+event.values[3];
                SYNC[5] = 1;
//                Log.d("capsensordata_r", rotationVectorData[1]);
                break;
            }
            case Sensor.TYPE_ORIENTATION:{
                orientationData = new String[3];
                orientationData[0] = ""+event.values[0];
                orientationData[1] = ""+event.values[1];
                orientationData[2] = ""+event.values[2];
                SYNC[6] = 1;
//                Log.d("capsensordata_r", rotationVectorData[1]);
                break;
            }
            default:
                break;
        }

        if(sync()) {
//            if(flag!=0){
//                SensorManager.getRotationMatrix(r_c, null, accValues, magValues);
//                SensorManager.getOrientation(r_c, eulerAngle_c);
//                cYaw.add(eulerAngle_c[0]); //弧度-Π~Π
//                cPitch.add(eulerAngle_c[1]);
//                cRoll.add(eulerAngle_c[2]);
//
//                SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
//                SensorManager.getOrientation(r_g, eulerAngle_g);
//                gEuler.add(eulerAngle_g);
//                gYaw = getbycolumn(gEuler,0);
//                gPitch = getbycolumn(gEuler,1);
//                gRoll = getbycolumn(gEuler,2);
//
////                r_g_f = r_g;
//                r_f = r_g;
//                SensorManager.getOrientation(r_f, eulerAngle);
//
//                flag = 0;
//            }
            SensorManager.getRotationMatrix(r_c, null, accValues, magValues);
            SensorManager.getOrientation(r_c, eulerAngle_c);
            cYaw.add(eulerAngle_c[0]); //弧度-Π~Π
            cPitch.add(eulerAngle_c[1]);
            cRoll.add(eulerAngle_c[2]);

            SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
            SensorManager.getOrientation(r_g, eulerAngle_g);
            gEuler.add(eulerAngle_g);
            gYaw = getbycolumn(gEuler,0);
            gPitch = getbycolumn(gEuler,1);
            gRoll = getbycolumn(gEuler,2);

            if (timestamp != 0) {
//                if(change!=0){
////                    getVectorFromRotationMatrix(rotaVecValues_f,r_g_f);
////                    daertaRotationMatrix(daertaRotationMatrix, rotaVecValues_f, rotaVecValues);
////
////                    SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
////                    getNewRotationMatrix(r_g_f,r_g,daertaRotationMatrix);
////
////                    SensorManager.getOrientation(r_g_f, eulerAngle_g);
////                    gEuler.add(eulerAngle_g);
////                    gYaw = getbycolumn(gEuler,0);
////                    gPitch = getbycolumn(gEuler,1);
////                    gRoll = getbycolumn(gEuler,2);
//
////                    SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
//                    SensorManager.getOrientation(r_g, eulerAngle_g);
//                    gEuler.add(eulerAngle_g);
//                    gYaw = getbycolumn(gEuler,0);
//                    gPitch = getbycolumn(gEuler,1);
//                    gRoll = getbycolumn(gEuler,2);
////                    test
//                    SensorManager.getRotationMatrixFromVector(r_test, rotaVecValues);
//                    SensorManager.getOrientation(r_test, eulerAngle_test);
////                    System.out.println("eulerAngle_g: "+ eulerAngle_test[0]/(2*Math.PI)*360);
////                    System.out.println("eulerAngle_f: "+ eulerAngle_g[0]/(2*Math.PI)*360);
//                }
//                else{
////                    if(daerta()){
////                        SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
////                        getNewRotationMatrix(r_g_f,r_g,daertaRotationMatrix);
////                    }
////                    else{
////                        SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
////                        r_g_f = r_g;
////                    }
//                    SensorManager.getRotationMatrixFromVector(r_g, rotaVecValues);
//                    SensorManager.getOrientation(r_g, eulerAngle_g);
//                    gEuler.add(eulerAngle_g);
//                    gYaw = getbycolumn(gEuler,0);
//                    gPitch = getbycolumn(gEuler,1);
//                    gRoll = getbycolumn(gEuler,2);
//                }
                final double dT = (event.timestamp - timestamp) * NS2S;
//                System.out.println("dT "+ dT);
                eg = funcOmega(omega) * dT + funcAcc(linearAcc, g) *dT;
//                System.out.println("eg: "+ eg);

                eG = eG + eg;
                eg = 0;

                tW = tW +dT;
                if(tW >= DETECTION_WINDOW){
                    float pg1;
                    float pg2;

                    pc = calP(cYaw, gYaw, WC);
                    pg1 = calP(cPitch, gPitch, W1);
                    pg2 = calP(cRoll, gRoll, W2);
                    pg=(pg1+pg2)/2.0f;

                    e1 = -32.14 * pc + 19.93;
                    e2 = -12.86 * pg + 11.57;
                    eC = Math.max(e1, e2);
//
//                    System.out.println("###################################################");
//                    System.out.println("pc: "+ pc);
//                    System.out.println("pg: "+ pg);
//                    System.out.println("eC: "+ eC);
//                    System.out.println("eG: "+ eG);
//                    System.out.println("###################################################");

                    a3OppoCali(eC, eG, pc, pg);
                    SensorManager.getOrientation(r_f, eulerAngle);

                    tW = 0;
                    eG = 0;
                }else{
                    r_f = r_g;
                    SensorManager.getOrientation(r_f, eulerAngle);
//                    System.out.println("===== eulerAngle=eulerAngle_g =====");
                }
            }
            else{
                r_f = r_g;
                SensorManager.getOrientation(r_f, eulerAngle);
            }
            timestamp = event.timestamp;

            System.out.println("eulerAngle: "+ eulerAngle[0] + "=" + eulerAngle[0]/(2*Math.PI)*360);

            TextView textview = (TextView) findViewById(R.id.degree);
            double value = Double.valueOf(orientationData[0]);
            String result1 = String.format("%.2f", value);
            String result2 = String.format("%.2f", eulerAngle[0]/(2*Math.PI)*360);
            String s = "o:"+ result1 + "\n" + "c:" + result2;
            textview.setText(s);

            String captime = getCurrentTime();
            SensorData.addSensorData(gyroscopeData, accelerometerData, magneticFieldData, gravityData, linearAccelerationData, rotationVectorData,
                    orientationData, captime,
                    eulerAngle_c, eulerAngle_c[0], eulerAngle_g, eulerAngle_g[0], eulerAngle, eulerAngle[0]);
            reset();
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public double calMagnitude(float[] threeAxis){
        double dataMagnitude;

        dataMagnitude = Math.sqrt(threeAxis[0]*threeAxis[0] + threeAxis[1]*threeAxis[1] + threeAxis[2]*threeAxis[2]);

        return dataMagnitude;
    }

    public double funcOmega(double omega){
        if(omega < 240){
            return 0.0003*omega;
        }else{
            return UNBOUNDED;
        }
    }

    public double funcAcc(double linearAcc, double g){
        if(linearAcc < 2*g){
            return 0.001*linearAcc;
        }else{
            return UNBOUNDED;
        }
    }

    public List<Float> getbycolumn(List<float[]> array, int column){
        int rowlength = array.size();
        List templist = new ArrayList();

        for(int i=0;i<rowlength;i++)
            templist.add(array.get(i)[column]);
        return templist;
    }

    public float calP(List<Float> f1, List<Float> f2, int type){
        float result;
        float var = 0;

        if(f1.size()==f2.size()){
            CalSta cal = new CalSta();
            int len;
            int start = 0;
            len = f1.size();
            float[] temp = new float[len];

            switch (type){
                case WC:{
                    start = windowStart_c;
                    break;
                }
                case W1:{
                    start = windowStart_1;
                    break;
                }
                case W2:{
                    start = windowStart_2;
                    break;
                }
                default:
                    break;
            }

            for(int i = start; i<len;i++){
                temp[i]= f2.get(i) - f1.get(i);
            }

//            System.out.println("len:" + (len-start));

            var = cal.getVariance(temp);

            switch (type){
                case WC:{
                    windowStart_c=len;
                    break;
                }
                case W1:{
                    windowStart_1=len;
                    break;
                }
                case W2:{
                    windowStart_2=len;
                    break;
                }
            }

        } else{
            System.out.println("两个数组不等长！");
            return 0;
        }

        result = (float) (1/Math.pow(2, var));
        return result;
    }

    public class CalSta{
        float getSum(float[] data) {
            float sum = 0;
            for (int i = 0; i < data.length; i++)
                sum = sum + data[i];
            return sum;
        }
        float getMean(float[] data) {
            float mean = 0;
            mean = getSum(data) / data.length;
            return mean;
        }
        float getVariance(float[] data) {
            float variance = (float) 0.0;
            for (int i = 0; i < data.length; i++) {
                variance = (float) (variance + (Math.pow((data[i] - getMean(data)), 2)));
            }
            variance = variance / data.length;
            return variance;
        }
    }

    public void a3OppoCali(double eC, double eG, float pc,float pg){
        if(pc > 0.2 && pg > 0.2){
            if(eC<eG){
                r_f = r_c;
                r_g = r_c;
//                change = 1;
                System.out.println("==================== eulerAngle=eulerAngle_c ==================");
            }else{
                r_f = r_g;
//                System.out.println("===== eulerAngle=eulerAngle_g =====");
            }
        }else{
            r_f = r_g;
//            System.out.println("===== eulerAngle=eulerAngle_g =====");
        }
    }

//    public void getVectorFromRotationMatrix(float[] rotationVector, float[] R){
//        float r11 = R[0];
//        float r12 = R[1];
//        float r13 = R[2];
//        float r21 = R[3];
//        float r22 = R[4];
//        float r23 = R[5];
//        float r31 = R[6];
//        float r32 = R[7];
//        float r33 = R[8];
//        float w;
//
//        w = (float) (0.5 * Math.sqrt(1+r11+r22+r33));
//        rotationVector[3] = w;
//        rotationVector[0] = (r32-r23)/(4*w);
//        rotationVector[1] = (r13-r31/(4*w));
//        rotationVector[2] = (r21-r12)/(4*w);
//    }
//
//    public float daertaSetae(float[] newRotationVector, float[] oldRotationVector){
//        float a1 = newRotationVector[0];
//        float b1 = newRotationVector[1];
//        float c1 = newRotationVector[2];
//        float d1 = newRotationVector[3];
//        float a2 = oldRotationVector[0];
//        float b2 = oldRotationVector[1];
//        float c2 = oldRotationVector[2];
//        float d2 = oldRotationVector[3];
//
//        float temp = a1*a2+b1*b2+c1*c2+d1*d2;
//        float daertaSetae = (float) (2 * Math.acos(Math.abs(temp)));
//        change = 0;
//
//        return daertaSetae;
//    }
//
//    public void daertaRotationMatrix(float[] daertaRotationMatrix, float[] newRotationVector, float[] oldRotationVector){
//        float[] newRotationMatrix = new float[9];
//        float[][] _newRotationMatrix = new float[3][3];
//        float[] oldRotationMatrix = new float[9];
//        float[][] _oldRotationMatrix = new float[3][3];
//
//        SensorManager.getRotationMatrixFromVector(newRotationMatrix, newRotationVector);
//        SensorManager.getRotationMatrixFromVector(oldRotationMatrix, oldRotationVector);
//
//        int  k = 0;
//        for(int i=0;i<3;i++){
//            for(int j=0;j<3;j++){
//                _newRotationMatrix[i][j]=newRotationMatrix[k];
//                _oldRotationMatrix[i][j]=oldRotationMatrix[k];
//                k++;
//            }
//        }
//
//        getMrinv(_oldRotationMatrix,3);
//        matrixMulti(daertaRotationMatrix, _newRotationMatrix,_oldRotationMatrix);
//        change = 0;
//    }

    ////////////////////////////////////////////////////////////////////////
    //函数：Mrinv
    //功能：求矩阵的逆
    //参数：n---整数，矩阵的阶数
    //a---Double型n*n二维数组，开始时为原矩阵，返回时为逆矩阵
    ////////////////////////////////////////////////////////////////////////
//    public static void getMrinv(float[][] a, int n) {
//        int i, j, row, col, k;
//        float max, temp;
//        int[] p = new int[n];
//        float[][] b = new float[n][n];
//        for (i = 0; i < n; i++) {
//            p[i] = i;
//            b[i][i] = 1;
//        }
//
//        for (k = 0; k < n; k++) {
//            // 找主元
//            max = 0;
//            row = col = i;
//            for (i = k; i < n; i++)
//                for (j = k; j < n; j++) {
//                    temp = Math.abs(b[i][j]);
//                    if (max < temp) {
//                        max = temp;
//                        row = i;
//                        col = j;
//                    }
//                }
//            // 交换行列，将主元调整到 k 行 k 列上
//            if (row != k) {
//                for (j = 0; j < n; j++) {
//                    temp = a[row][j];
//                    a[row][j] = a[k][j];
//                    a[k][j] = temp;
//                    temp = b[row][j];
//                    b[row][j] = b[k][j];
//                    b[k][j] = temp;
//                }
//                i = p[row];
//                p[row] = p[k];
//                p[k] = i;
//            }
//            if (col != k) {
//                for (i = 0; i < n; i++) {
//                    temp = a[i][col];
//                    a[i][col] = a[i][k];
//                    a[i][k] = temp;
//                }
//            }
//            // 处理
//            for (j = k + 1; j < n; j++)
//                a[k][j] /= a[k][k];
//            for (j = 0; j < n; j++)
//                b[k][j] /= a[k][k];
//            a[k][k] = 1;
//
//            for (j = k + 1; j < n; j++) {
//                for (i = 0; i < k; i++)
//                    a[i][j] -= a[i][k] * a[k][j];
//                for (i = k + 1; i < n; i++)
//                    a[i][j] -= a[i][k] * a[k][j];
//            }
//            for (j = 0; j < n; j++) {
//                for (i = 0; i < k; i++)
//                    b[i][j] -= a[i][k] * b[k][j];
//                for (i = k + 1; i < n; i++)
//                    b[i][j] -= a[i][k] * b[k][j];
//            }
//            for (i = 0; i < k; i++)
//                a[i][k] = 0;
//            a[k][k] = 1;
//        }
//        // 恢复行列次序；
//        for (j = 0; j < n; j++)
//            for (i = 0; i < n; i++)
//                a[p[i]][j] = b[i][j];
//    }
//
//    public void matrixMulti(float[] result, float a[][], float b[][]) {
//        if (a.length == b[0].length){
//            //c矩阵的行数y，与列数x
//            int N=a.length;
//            int index=0;
//            float[][] _result = new float[N][N];
//            for (int i = 0; i < N; i++){
//                for (int j = 0; j < N; j++){
//                    //c矩阵的第i行第j列所对应的数值，等于a矩阵的第i行分别乘以b矩阵的第j列之和
//                    for (int k = 0; k < N; k++){
//                        _result[i][j] += a[i][k] * b[k][j];
//                    }
//                    result[index++] = _result[i][j];
//                }
//            }
//        }
//    }
//
//    public void getNewRotationMatrix(float[] newRotationMatrix, float[] oldRotationMatrix, float[] daertaRotationMatrix){
//        float[][] _daertaRotationMatrix = new float[3][3];
//        float[][] _oldRotationMatrix = new float[3][3];
//
//        int  k = 0;
//        for(int i=0;i<3;i++){
//            for(int j=0;j<3;j++){
//                _daertaRotationMatrix[i][j]=daertaRotationMatrix[k];
//                _oldRotationMatrix[i][j]=oldRotationMatrix[k];
//                k++;
//            }
//        }
//
//
//        matrixMulti(newRotationMatrix,_daertaRotationMatrix,_oldRotationMatrix);
//    }
//
//    public boolean daerta(){
//        for(int i = 0 ; i < daertaRotationMatrix.length ; i++){
//            if(daertaRotationMatrix[i] != 0)
//                return true;
//        }
//        return false;
//    }
}