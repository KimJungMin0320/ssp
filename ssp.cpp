#include "ssp.h"

MPU6050 mpu;

#define FRONT_LED_PIN 10
#define REAR_LED_PIN 9

#define LEFT_MD_A 22
#define LEFT_MD_B 23
#define RIGHT_MD_A 24
#define RIGHT_MD_B 25

#define LEFT_MOTOR_EN 4
#define RIGHT_MOTOR_EN 5

#define NUM_TX_BYTES 5
#define NUM_RX_BYTES 17

#define S_DIN 42
#define S_SCLK 43
#define S_SYNCN 44
#define IN_SEN_EN 26

unsigned char TX_buf[NUM_TX_BYTES] = {0x76, 0x00, 0xF0, 0x00, 0xF0};
unsigned char TX_stop_buf[NUM_TX_BYTES] = {0x76, 0x00, 0x0F, 0x00, 0x0F};
unsigned char RX_buf[NUM_RX_BYTES];

unsigned char text[] = "\r\n Welcome! Arduino Mega\r\n UART Test Program.\r\n";

boolean ultrasonic_result = false;
boolean line_tracing = false;

int SensorA[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int SensorD[8] = {30,31,32,33,34,35,36,37};

uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

int start[2];
int pre_end_x1;
int pre_end_x2;
float end_x_mean;
int end[2];

void move_stop(){
	analogWrite(LEFT_MOTOR_EN, 0);
	analogWrite(RIGHT_MOTOR_EN, 0);
}

void move_forward_speed(int left, int right)
{
	digitalWrite(LEFT_MD_A, HIGH);
	digitalWrite(LEFT_MD_B, LOW);

	digitalWrite(RIGHT_MD_A, LOW);
	digitalWrite(RIGHT_MD_B, HIGH);

	analogWrite(LEFT_MOTOR_EN, left);
	analogWrite(RIGHT_MOTOR_EN, right);
}

void move_backward_speed(int left, int right)
{
	digitalWrite(LEFT_MD_A, LOW);
	digitalWrite(LEFT_MD_B, HIGH);

	digitalWrite(RIGHT_MD_A, HIGH);
	digitalWrite(RIGHT_MD_B, LOW);

	analogWrite(LEFT_MOTOR_EN, left);
	analogWrite(RIGHT_MOTOR_EN, right);
}

void turn_left_speed(int left, int right)
{
	digitalWrite(LEFT_MD_A, LOW);
	digitalWrite(LEFT_MD_B, HIGH);

	digitalWrite(RIGHT_MD_A, LOW);
	digitalWrite(RIGHT_MD_B, HIGH);

	analogWrite(LEFT_MOTOR_EN, left);
	analogWrite(RIGHT_MOTOR_EN, right);
}

void turn_right_speed(int left, int right)
{
	digitalWrite(LEFT_MD_A, HIGH);
	digitalWrite(LEFT_MD_B, LOW);

	digitalWrite(RIGHT_MD_A, HIGH);
	digitalWrite(RIGHT_MD_B, LOW);

	analogWrite(LEFT_MOTOR_EN, left);
	analogWrite(RIGHT_MOTOR_EN, right);
}

void turn_pivot_left_speed(int left, int right)
{
	digitalWrite(LEFT_MD_A, LOW);
	digitalWrite(LEFT_MD_B, HIGH);

	digitalWrite(RIGHT_MD_A, LOW);
	digitalWrite(RIGHT_MD_B, HIGH);

	analogWrite(LEFT_MOTOR_EN, left);
	analogWrite(RIGHT_MOTOR_EN, right);
}

void turn_pivot_right_speed(int left, int right)
{
	digitalWrite(LEFT_MD_A, HIGH);
	digitalWrite(LEFT_MD_B, LOW);

	digitalWrite(RIGHT_MD_A, HIGH);
	digitalWrite(RIGHT_MD_B, LOW);

	analogWrite(LEFT_MOTOR_EN, left);
	analogWrite(RIGHT_MOTOR_EN, right);
}

void DAC_setting(unsigned int data)
{
	int z;
	digitalWrite(S_SCLK,HIGH);
	delayMicroseconds(1);
	digitalWrite(S_SCLK,LOW);
	delayMicroseconds(1);
	digitalWrite(S_SYNCN,LOW);
	delayMicroseconds(1);
	for(z=15;z>=0;z--)
	{
		digitalWrite(S_DIN,(data>>z)&0x1);
		digitalWrite(S_SCLK,HIGH);
		delayMicroseconds(1);
		digitalWrite(S_SCLK,LOW);
		delayMicroseconds(1);
	}
	digitalWrite(S_SYNCN,HIGH);
}

void DAC_CH_Write(unsigned int ch, unsigned int da)
{
	unsigned int data = ((ch << 12) & 0x7000) | ((da << 4) & 0x0FF0);
	DAC_setting(data);
}

void line_tracing_enable()
{
	line_tracing = true;
	Serial.write("Line tracing is enabled..");
}

void line_tracing_disable()
{
	line_tracing = false;
	move_stop();
	Serial.write("Line tracing is disabled..");
}

void front_led_control(boolean x){
	digitalWrite(FRONT_LED_PIN, x);
}

void rear_led_control(boolean x){
	digitalWrite(REAR_LED_PIN, x);
}

void ultrasonic_sensor_read() {
	ultrasonic_result = false;
	Serial1.write(TX_buf, NUM_TX_BYTES);
}

void setup()
{
	int z;
	int dac_val_min[8] = {362, 445, 500, 660, 609, 596, 615, 353};
	int dac_val_max[8] = {74, 76, 84, 130, 117, 125, 135, 67};

	Wire.begin();

	int i = 0;
	Serial.begin(115200);

	while (text[i] != '\0')
		Serial.write(text[i++]);
	Serial.write("Received cmds: ");

	Serial1.begin(115200);

	pinMode(FRONT_LED_PIN, OUTPUT);
	pinMode(REAR_LED_PIN, OUTPUT);

	pinMode(LEFT_MD_A, OUTPUT);
	pinMode(LEFT_MD_B, OUTPUT);
	pinMode(RIGHT_MD_A, OUTPUT);
	pinMode(RIGHT_MD_B, OUTPUT);
	pinMode(LEFT_MOTOR_EN, OUTPUT);
	pinMode(RIGHT_MOTOR_EN, OUTPUT);

	digitalWrite(LEFT_MD_A, LOW);
	digitalWrite(LEFT_MD_B, LOW);
	digitalWrite(RIGHT_MD_A, LOW);
	digitalWrite(RIGHT_MD_B, LOW);
	digitalWrite(LEFT_MOTOR_EN, LOW);
	digitalWrite(RIGHT_MOTOR_EN, LOW);

	pinMode(IN_SEN_EN,OUTPUT);
	pinMode(S_DIN,OUTPUT);
	pinMode(S_SCLK,OUTPUT);
	pinMode(S_SYNCN,OUTPUT);
	digitalWrite(S_SCLK,LOW);
	digitalWrite(S_SYNCN,HIGH);
	digitalWrite(IN_SEN_EN,HIGH);

	for (z=0; z<8; z++)
		pinMode(SensorD[z], INPUT);

	DAC_setting(0x9000); //for Write-Through Mode

	for (z=0; z<8; z++)
	{
		int mean_val = (dac_val_min[z]+dac_val_max[z])/2;

		DAC_CH_Write(z, mean_val >> 2);
	}

	Serial.println("Initializing I2C devices・");
	mpu.initialize();

	Serial.println("Testing device connections・");
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	Serial.println("Initializing DMP・");
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		Serial.println("Enabling DMP・");
		mpu.setDMPEnabled(true);
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		Serial.print("DMP Initialization failed (code ");
		Serial.print(devStatus);
		Serial.println(")");
	}
}

void loop()
{
	if (line_tracing == true)
	{
		unsigned char sensor_data = 0;
		int z;

		for(z=0;z<8;z++)
		{
			unsigned int val = digitalRead(SensorD[z]);
			sensor_data |= (val << z);
		}
		sensor_data = ~sensor_data;

		Serial.print(sensor_data, HEX);
		Serial.write(" ");

		switch (sensor_data)
		{
		case 0x18:
		case 0x10:
		case 0x08:
		case 0x38:
		case 0x1c:
		case 0x3c:
			move_forward_speed(140, 140);
			break;

		case 0x0c:
		case 0x04:
		case 0x06:
		case 0x0e:
		case 0x1e:
		case 0x0f:
			turn_right_speed(200, 0);
			break;

		case 0x30:
		case 0x20:
		case 0x60:
		case 0x70:
		case 0x78:
		case 0xf0:
			turn_left_speed(0, 200);
			break;

		case 0x07:
		case 0x03:
		case 0x02:
		case 0x01:
			turn_pivot_right_speed(200, 80);
			break;

		case 0xc0:
		case 0x40:
		case 0x80:
		case 0xe0:
			turn_pivot_left_speed(80, 200);
			break;

		case 0x00:
		case 0xff:
			move_stop();
			break;

		default:
			move_stop();
			break;
		}
		delay(5);
	}
}

void infrared_sensor_read()
{
	int z;
	for(z=7;z>=0;z--)
	{
		unsigned int val = analogRead(SensorA[z]);

		Serial.print(val);
		Serial.print(" ");
	}

	Serial.println("");

	for(z=7;z>=0;z--)
	{
		unsigned int val = digitalRead(SensorD[z]);
		Serial.print(val);
		Serial.print(" ");
	}
}

void gyro_accel_read()
{
	while(1)
	{
		mpu.resetFIFO();
		mpuIntStatus = mpu.getIntStatus();
		fifoCount = mpu.getFIFOCount();

		if(mpuIntStatus & 0x02)
		{
			while(fifoCount < packetSize)
				fifoCount = mpu.getFIFOCount();
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			break;
		}
	}
	Serial.print(" yaw : ");
	Serial.print(180 - (ypr[0] * 180/M_PI)); //yaw
	Serial.print(" pitch : ");
	Serial.print(ypr[1] * 180/M_PI); // pitch
	Serial.print(" roll : ");
	Serial.println(ypr[2] * 180/M_PI); //roll
}

void serialEvent1(){
	unsigned char z, tmp = 0;
	Serial1.readBytes((char *)RX_buf, NUM_RX_BYTES);

	if ((RX_buf[0] == 0x76) && (RX_buf[1] == 0x00) && (ultrasonic_result == false)){
		for (z = 2; z < NUM_RX_BYTES-1; z++)
			tmp += RX_buf[z];

		tmp = tmp & 0xFF;

		if (RX_buf[NUM_RX_BYTES-1] == tmp){
			Serial.println("FRONT");
			for (z=4; z < 11; z++){
				Serial.print(" F");
				Serial.print(z-4);
				Serial.print(": ");
				Serial.print(RX_buf[z]);
			}

			Serial.println("\nBACK");
			for (z=11; z < NUM_RX_BYTES-1; z++){
				Serial.print(" B");
				Serial.print(z-11);
				Serial.print(": ");
				Serial.print(RX_buf[z]);
			}
		}
		ultrasonic_result = true;
		Serial1.write(TX_stop_buf, NUM_TX_BYTES);
	}
}

void serialEvent()
{
	int command = Serial.read();
	Serial.print(command, DEC);
	Serial.print("\n");

	switch (command)
	{
	case 1:
		start[0]=0;
		start[1]=0;
		break;
	case 2:
		start[0]=0;
		start[1]=1;
		break;
	case 3:
		start[0]=0;
		start[1]=2;
		break;
	case 4:
		start[0]=1;
		start[1]=0;
		break;
	case 5:
		start[0]=1;
		start[1]=1;
		break;
	case 6:
		start[0]=1;
		start[1]=2;
		break;
	case 7:
		start[0]=2;
		start[1]=0;
		break;
	case 8:
		start[0]=2;
		start[1]=1;
		break;
	case 9:
		start[0]=2;
		start[1]=2;
		break;
	case 10:
		pre_end_x1=0;
		pre_end_x2=1;
		end_x_mean=(pre_end_x1+pre_end_x2)/2;
		if(end_x_mean-start[0]>0){
			end[0]=pre_end_x1;
		}
		else{
			end[0]=pre_end_x2;
		}
		end[1]=0;
		break;
	case 11:
		pre_end_x1=1;
		pre_end_x2=2;
		end_x_mean=(pre_end_x1+pre_end_x2)/2;
		if(end_x_mean-start[0]>0){
			end[0]=pre_end_x1;
		}
		else{
			end[0]=pre_end_x2;
		}
		end[1]=0;
		break;
	case 12:
		end[0]=2;
		end[1]=0;
		break;
	case 13:
		pre_end_x1=0;
		pre_end_x2=1;
		end_x_mean=(pre_end_x1+pre_end_x2)/2;
		if(end_x_mean-start[0]>0){
			end[0]=pre_end_x1;
		}
		else{
			end[0]=pre_end_x2;
		}
		end[1]=1;
		break;
	case 14:
		pre_end_x1=1;
		pre_end_x2=2;
		end_x_mean=(pre_end_x1+pre_end_x2)/2;
		if(end_x_mean-start[0]>0){
			end[0]=pre_end_x1;
		}
		else{
			end[0]=pre_end_x2;
		}
		end[1]=1;
		break;
	case 15:
		end[0]=2;
		end[1]=1;
		break;
	case 16:
		pre_end_x1=0;
		pre_end_x2=1;
		end_x_mean=(pre_end_x1+pre_end_x2)/2;
		if(end_x_mean-start[0]>0){
			end[0]=pre_end_x1;
		}
		else{
			end[0]=pre_end_x2;
		}
		end[1]=2;
		break;
	case 17:
		pre_end_x1=1;
		pre_end_x2=2;
		end_x_mean=(pre_end_x1+pre_end_x2)/2;
		if(end_x_mean-start[0]>0){
			end[0]=pre_end_x1;
		}
		else{
			end[0]=pre_end_x2;
		}
		end[1]=2;
		break;
	case 18:
		end[0]=2;
		end[1]=2;
		break;
	case 20:
		line_tracing_enable();
		break;
	case 21:
		line_tracing_disable();
		break;
	default:
		move_stop();
		front_led_control(false);
		rear_led_control(false);
	}
}
