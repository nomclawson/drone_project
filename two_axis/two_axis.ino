#include "two_axis_drone.h"
#include "MPU_6050_registers.h"

// We know that the slave adress for this IMU is 0x68
#define SLAVE_ADDR 0x68

void setup() {
	/********************************************
	 * Enable interrupts
	 ********************************************/
	// Set pins D8, D9, D10, and D12 to trigger an interrupt on state change, using function "ISR"
	attachInterrupt(digitalPinToInterrupt(8), ISR, CHANGE); 
	attachInterrupt(digitalPinToInterrupt(9), ISR, CHANGE); 
	attachInterrupt(digitalPinToInterrupt(10), ISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(12), ISR, CHANGE);
	/*********
	 * Low level equivalent code:
	 *	PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
	 *	PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
	 *	PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.                                             
	 *	PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
	 *	PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.  
	 *********/

	pinMode(13,OUTPUT);  //D13 as output
	digitalWrite(13,LOW); //D13 set to LOW
	/*********
	 * Low level equivalent:
	 *	DDRB |= B00100000;  //D13 as output
	 *	PORTB &= B11011111; //D13 set to LOW
	**********/


	/********************************************
	 * Attach prop motors
	 ********************************************/
	L_F_prop.attach(4); //left front motor
	L_B_prop.attach(5); //left back motor
	R_F_prop.attach(7); //right front motor 
	R_B_prop.attach(6); //right back motor 
	
	/*in order to make sure that the ESCs won't enter into config mode
	*I send a 1000us pulse to each ESC.*/
	L_F_prop.writeMicroseconds(1000); 
	L_B_prop.writeMicroseconds(1000);
	R_F_prop.writeMicroseconds(1000); 
	R_B_prop.writeMicroseconds(1000);

	

	Wire.begin();                           //begin the wire comunication
	Wire.beginTransmission(SLAVE_ADDR);     //begin, Send the slave adress (in this case 68)              
	Wire.write(MPU_6050_PWR_MGMT_1);        //make the reset (place a 0 into the 6B register)
	Wire.write(0x00);
	Wire.endTransmission(true);             //end the transmission

	Wire.beginTransmission(SLAVE_ADDR);     //begin, Send the slave adress (in this case 68) 
	Wire.write(MPU_6050_GYRO_CONFIG);       //We want to write to the GYRO_CONFIG register (1B hex)
	Wire.write(0x10);                       //Set the register bits as 00010000 (100dps full scale)
	Wire.endTransmission(true);             //End the transmission with the gyro

	Wire.beginTransmission(SLAVE_ADDR);     //Start communication with the address found during search.
	Wire.write(MPU_6050_ACCEL_CONFIG);      //We want to write to the ACCEL_CONFIG register (1A hex)
	Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
	Wire.endTransmission(true);  

	Serial.begin(9600);
	delay(1000);
	time = millis();                         //Start counting time in milliseconds


	/*Here we calculate the gyro data error before we start the loop
	* I make the mean of 200 values, that should be enough*/
	if(gyro_error==0)
	{
		for(int i=0; i<200; i++)
		{
			Wire.beginTransmission(SLAVE_ADDR);		//begin, Send the slave adress (in this case 68) 
			Wire.write(MPU_6050_GYRO_XOUT_H);		// Send address for the gyro data registers
			Wire.endTransmission(false);
			Wire.requestFrom(SLAVE_ADDR,4,true);	// Ask for 4 registers (XOUT_H, XOUT_L, YOUT_H, and YOUT_L)
				
			Gyr_rawX=Wire.read()<<8|Wire.read();	// Read two bytes (the x-axis data) from the slave.
			Gyr_rawY=Wire.read()<<8|Wire.read();	// Read two more bytes (the y-axis data) from the slave

			/*---X---*/
			Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
			/*---Y---*/
			Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
			if(i==199)
			{
				Gyro_raw_error_x = Gyro_raw_error_x/200;
				Gyro_raw_error_y = Gyro_raw_error_y/200;
				gyro_error=1;
			}
		}
	}//end of gyro error calculation   


	/*Here we calculate the acc data error before we start the loop
	* I make the mean of 200 values, that should be enough*/
	if(acc_error==0)
	{
		for(int a=0; a<200; a++)
		{
			Wire.beginTransmission(SLAVE_ADDR);
			Wire.write(MPU_6050_ACCEL_XOUT_H);		// Send address for accelerometer data registers
			Wire.endTransmission(false);
			Wire.requestFrom(SLAVE_ADDR,6,true);	// Request 6 registers. (XOUT_H, XOUT_L, YOUT_H, YOUT_L, ZOUT_H, and ZOUT_L)
			
			Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value is two bytes
			Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
			Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;

			
			/*---X---*/
			Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
			/*---Y---*/
			Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg)); 
			
			if(a==199)
			{
			Acc_angle_error_x = Acc_angle_error_x/200;
			Acc_angle_error_y = Acc_angle_error_y/200;
			acc_error=1;
			}
		}
	}//end of acc error calculation  
}//end of setup function

void loop() {


	/////////////////////////////I M U/////////////////////////////////////
	timePrev = time;  // the previous time is stored before the actual time read
	time = millis();  // actual time read
	elapsedTime = (time - timePrev) / 1000;     

	/*The tiemStep is the time that elapsed since the previous loop. 
	*This is the value that we will use in the formulas as "elapsedTime" 
	*in seconds. We work in ms so we have to divide the value by 1000 
	to obtain seconds*/

	/*Reed the values that the MPU_6050 gives.
	* The slave adress for this IMU is defined with SLAVE_ADDR.
	* We have to put this value in the RequestFrom and the 
	* begin functions .*/

	//////////////////////////////////////Gyro read/////////////////////////////////////
	Wire.beginTransmission(SLAVE_ADDR);		//begin, Send the slave adress (in this case 68) 
	Wire.write(MPU_6050_GYRO_XOUT_H);		//First address of the Gyro data
	Wire.endTransmission(false);
	Wire.requestFrom(SLAVE_ADDR,4,true);	// Ask for 4 registers (XOUT_H, XOUT_L, YOUT_H, and YOUT_L)

	Gyr_rawX=Wire.read()<<8|Wire.read();	// Once again read two bytes for each axis
	Gyr_rawY=Wire.read()<<8|Wire.read();
	/*Now in order to obtain the gyro data in degrees/seconds we have to divide first
	the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
	/*---X---*/
	Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
	/*---Y---*/
	Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;  
	/*Now we integrate the raw value in degrees per seconds in order to obtain the angle
	* If you multiply degrees/seconds by seconds you obtain degrees */
	/*---X---*/
	Gyro_angle_x = Gyr_rawX*elapsedTime;
	/*---X---*/
	Gyro_angle_y = Gyr_rawY*elapsedTime;



	//////////////////////////////////////Acc read/////////////////////////////////////
	Wire.beginTransmission(SLAVE_ADDR);		//begin, Send the slave adress (in this case 68) 
	Wire.write(MPU_6050_ACCEL_XOUT_H);		// Send address for accelerometer data registers
	Wire.endTransmission(false);			//keep the transmission and next
	Wire.requestFrom(SLAVE_ADDR,6,true);	//We ask for next 6 registers (XOUT_H, XOUT_L, YOUT_H, YOUT_L, ZOUT_H, and ZOUT_L) 

	/*We have asked for the 0x3B register. The IMU will send a brust of one byte registers.
	* The amount of bytes to read is specify in the requestFrom function.
	* In this case we request 6 registers. Each value is stored in
	* two 8bits registers, low values and high values. For that we request the 6 of them  
	* and smash them together in pairs. For that we shift the high values to the left 8 bits 
	* (<<) and use an or (|) operation to include the low values.
	If we read the datasheet, for a range of +/-8g, we have to divide the raw values by 4096*/    
	Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
	Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
	Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 

	/*Now in order to obtain the Acc angles we use euler's formula with acceleration values
	after that we substract the error value found before*/  
	/*---X---*/
	Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
	/*---Y---*/
	Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;   


	//////////////////////////////////////Total angle and filter/////////////////////////////////////
	/*---X axis angle---*/
	Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
	/*---Y axis angle---*/
	Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;







	/*///////////////////////////P I D///////////////////////////////////*/
	roll_desired_angle = map(input_ROLL,1000,2000,-10,10);
	pitch_desired_angle = map(input_PITCH,1000,2000,-10,10);

	/*First calculate the error between the desired angle and 
	*the real measured angle*/
	roll_error = Total_angle_y - roll_desired_angle;
	pitch_error = Total_angle_x - pitch_desired_angle;   

	/*Next the proportional value of the PID is just a proportional constant
	*multiplied by the error*/

	roll_pid_p = roll_kp*roll_error;
	pitch_pid_p = pitch_kp*pitch_error;
	/*The integral part should only act if we are close to the
	desired position but we want to fine tune the error. That's
	why I've made a if operation for an error between -2 and 2 degree.
	To integrate we just sum the previous integral value with the
	error multiplied by  the integral constant. This will integrate (increase)
	the value each loop till we reach the 0 point*/
	if(-3 < roll_error <3)
	{
		roll_pid_i = roll_pid_i+(roll_ki*roll_error);  
	}
	if(-3 < pitch_error <3)
	{
		pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);  
	}

	/*The last part is the derivate. The derivate acts upon the speed of the error.
	As we know the speed is the amount of error that produced in a certain amount of
	time divided by that time. For taht we will use a variable called previous_error.
	We substract that value from the actual error and divide all by the elapsed time. 
	Finnaly we multiply the result by the derivate constant*/
	roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
	pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);

	/*The final PID values is the sum of each of this 3 parts*/
	roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
	pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;

	/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
	tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
	have a value of 2000us the maximum value taht we could substract is 1000 and when
	we have a value of 1000us for the PWM signal, the maximum value that we could add is 1000
	to reach the maximum 2000us. But we don't want to act over the entire range so -+400 should be enough*/
	if(roll_PID < -400){roll_PID=-400;}
	if(roll_PID > 400) {roll_PID=400; }
	if(pitch_PID < -400){pitch_PID=-400;}
	if(pitch_PID > 400) {pitch_PID=400;}

	/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
	pwm_R_F  = 115 + input_THROTTLE - roll_PID - pitch_PID;
	pwm_R_B  = 115 + input_THROTTLE - roll_PID + pitch_PID;
	pwm_L_B  = 115 + input_THROTTLE + roll_PID + pitch_PID;
	pwm_L_F  = 115 + input_THROTTLE + roll_PID - pitch_PID;

	/*Once again we map the PWM values to be sure that we won't pass the min
	and max values. Yes, we've already maped the PID values. But for example, for 
	throttle value of 1300, if we sum the max PID value we would have 2300us and
	that will mess up the ESC.*/
	//Right front
	if(pwm_R_F < 1100) 
	{
	  pwm_R_F= 1100;
	}
	if(pwm_R_F > 2000)
	{
	  pwm_R_F=2000;
	}

	//Left front
	if(pwm_L_F < 1100)
	{
	  pwm_L_F= 1100;
	}
	if(pwm_L_F > 2000)
	{
	  pwm_L_F=2000;
	}

	//Right back
	if(pwm_R_B < 1100)
	{
	  pwm_R_B= 1100;
	}
	if(pwm_R_B > 2000)
	{
	  pwm_R_B=2000;
	}

	//Left back
	if(pwm_L_B < 1100)
	{
	  pwm_L_B= 1100;
	}
	if(pwm_L_B > 2000)
	{
	  pwm_L_B=2000;
	}

	roll_previous_error = roll_error; //Remember to store the previous error.
	pitch_previous_error = pitch_error; //Remember to store the previous error.

	/********* DEBUG *******
	Serial.print("RF: ");
	Serial.print(pwm_R_F);
	Serial.print("   |   ");
	Serial.print("RB: ");
	Serial.print(pwm_R_B);
	Serial.print("   |   ");
	Serial.print("LB: ");
	Serial.print(pwm_L_B);
	Serial.print("   |   ");
	Serial.print("LF: ");
	Serial.print(pwm_L_F);

	Serial.print("   |   ");
	Serial.print("Xº: ");
	Serial.print(Total_angle_x);
	Serial.print("   |   ");
	Serial.print("Yº: ");
	Serial.print(Total_angle_y);
	Serial.println(" ");
	*************************/







	/*now we can write the values PWM to the ESCs only if the motor is activated
	*/

	if(mot_activated)
	{
		L_F_prop.writeMicroseconds(pwm_L_F); 
		L_B_prop.writeMicroseconds(pwm_L_B);
		R_F_prop.writeMicroseconds(pwm_R_F); 
		R_B_prop.writeMicroseconds(pwm_R_B);
	}
	if(!mot_activated)
	{
		L_F_prop.writeMicroseconds(1000); 
		L_B_prop.writeMicroseconds(1000);
		R_F_prop.writeMicroseconds(1000); 
		R_B_prop.writeMicroseconds(1000);
	}

	if(input_THROTTLE < 1100 && input_YAW > 1800 && !mot_activated)
	{
		if(activate_count==200)
		{
			mot_activated=1;   
			PORTB |= B00100000; //D13 LOW   
		}
		activate_count=activate_count+1;
	}
	if(!(input_THROTTLE < 1100 && input_YAW > 1800) && !mot_activated)
	{
		activate_count=0;    
	}

	if(input_THROTTLE < 1100 && input_YAW < 1100 && mot_activated)
	{
		if(des_activate_count==300)
		{
			mot_activated=0;       
			PORTB &= B11011111; //D13 LOW   
		}
		des_activate_count=des_activate_count+1;
	}
	if(!(input_THROTTLE < 1100 && input_YAW < 1100) && mot_activated)
	{
	  des_activate_count=0;
	}

}













/******************************************
 * Interrupt Service Routine - 
 * Handles digital state changes on pins D8, D9, D10, and D12
 * *****************************************/
ISR(PCINT0_vect){
//First we take the current count value in micro seconds using the micros() function
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We use an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_ROLL = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }



  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_PITCH = current_count - counter_2;                             
  }


  
  ///////////////////////////////////////Channel 3
  if(PINB & B00000100 ){                             //pin D10 - B00000100                                         
    if(last_CH3_state == 0){                                             
      last_CH3_state = 1;                                                  
      counter_3 = current_count;                                               
    }
  }
  else if(last_CH3_state == 1){                                             
    last_CH3_state = 0;                                                    
    input_THROTTLE = current_count - counter_3;                            

  }


  
  ///////////////////////////////////////Channel 4
  if(PINB & B00010000 ){                             //pin D12  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_YAW = current_count - counter_4;                            
  }


 
}
 
