#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define MSG_BUFFER_SIZE	(100)
char msg[MSG_BUFFER_SIZE];

// Define the size of the array
#define OUTPUT_SIZE 4
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
    
}

void loop() {
  sensors_event_t accelData, gyroData, magData;
bno.getEvent(&accelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    float accelX = accelData.acceleration.x;
    float accelY = accelData.acceleration.y;
    float accelZ =accelData.acceleration.z;
    float gyroX = gyroData.gyro.x;
    float gyroY = gyroData.gyro.y;
    float gyroZ = gyroData.gyro.z;
    float magX = magData.magnetic.x;
    float magY = magData.magnetic.y;
    float magZ = magData.magnetic.z;
    
    //snprintf (msg, MSG_BUFFER_SIZE, "(%f,%f,%f,%f,%f,%f,%f,%f,%f)", accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX,magY, magZ);
    //Serial.print("Publish message: ");
    //Serial.println(msg);
    
  // Example input values (Replace with actual sensor readings or inputs)
  double input[9];
    input[0]=(accelX - (1.922148))/2.365882	; 
    input[1]=(accelY - (0.087752))/4.913305;
    input[2]=(accelZ - (2.411477))/7.451476;
    input[3]=(gyroX - (0.012577))/0.144687;
    input[4]=(gyroY - (-0.000957))/0.094472;
    input[5]=(gyroZ - (0.002154))/0.078474;
    input[6]=(magX - (18.673920))/11.116386;
    input[7]=(magY - (3.166527))/22.484901;
    input[8]=(magZ - (3.507236))/13.409552;

  double output[OUTPUT_SIZE];
  
  // Call the score function
  score(input, output);

  // Print the output for debugging
  for (int i = 0; i < OUTPUT_SIZE; i++) {
    Serial.print(output[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Delay before next loop iteration
  delay(1000);
}

void score(double *input, double *output) {
    double var0[OUTPUT_SIZE];

    if (input[2] <= -1.0534516870975494) {
        var0[0] = 0.0; var0[1] = 0.0; var0[2] = 1.0; var0[3] = 0.0;
    } else {
        if (input[2] <= 0.8973160088062286) {
            if (input[6] <= -0.8368567824363708) {
                var0[0] = 0.0; var0[1] = 0.0; var0[2] = 0.0; var0[3] = 1.0;
            } else {
                var0[0] = 0.0; var0[1] = 1.0; var0[2] = 0.0; var0[3] = 0.0;
            }
        } else {
            if (input[2] <= 0.9919678866863251) {
                var0[0] = 1.0; var0[1] = 0.0; var0[2] = 0.0; var0[3] = 0.0;
            } else {
                if (input[7] <= -0.040778934955596924) {
                    var0[0] = 0.0; var0[1] = 1.0; var0[2] = 0.0; var0[3] = 0.0;
                } else {
                    var0[0] = 0.0; var0[1] = 0.0; var0[2] = 0.0; var0[3] = 1.0;
                }
            }
        }
    }

    // Copiar la salida
    output[0] = var0[0];
    output[1] = var0[1];
    output[2] = var0[2];
    output[3] = var0[3];
}