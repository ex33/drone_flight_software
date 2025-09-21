  // if (allan_variance) {
  //   unsigned long now = micros();
  //   if (now - prevTime >= sampleInterval) {
  //     prevTime += sampleInterval; //Helps slightly prevent drift from using delay(10)

  //     imu.getEvent(&imu_a, &imu_g, &imu_temp);

  //     // print CSV line
  //     Serial.print(now); Serial.print(",");
  //     Serial.print(imu_a.acceleration.x,6); Serial.print(",");
  //     Serial.print(imu_a.acceleration.y,6); Serial.print(",");
  //     Serial.print(imu_a.acceleration.z,6); Serial.print(",");
  //     Serial.print(imu_g.gyro.x,6); Serial.print(",");
  //     Serial.print(imu_g.gyro.y,6); Serial.print(",");
  //     Serial.println(imu_g.gyro.z,6);
  //     //Run pio device monitor --baud 115200 --quiet > allan_variance_imu_data.csv
  //   };
  // };

    // Flags
  //allan_variance = 1; // Mode to Test  

  //prevTime = micros();

  //Serial.print(1e6/100,10);

//   bool allan_variance;
// unsigned long prevTime = 0;
// const unsigned long sampleInterval = 10000; //100 micro-s is ~100Hz (0.01s)