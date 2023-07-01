/*
  Robot xe hai bánh tự cân bằng
  Programmed by Phùng Minh Đức
  Date:20/6/2023

  Dùng Arduino nano  kết hợp CNC Shield V4
  Điều khiển động cơ dùng 2 x A4988 
  2 Đông cơ bước : size 42 1.8 step
  Cảm biến góc nghiêng : MPU6050

  thoi gian 1 xung = 20*x us = 0.00002*x s
  
    nếu chọn vi bước là 1/16
                1 vong =3200 xung----> thoi gian 1 vong ---> 3200*0.00002*x s
                                                  V---------> 60 s
                                                  V=60/(32*0.002*x)
              
                x=5 ----> v= 187.5 vong/ phut
                x=50----> v= 18.75 vong/phut
*/
#include "stmpu6050.h"
SMPU6050 mpu6050;
// ĐỊNH NGHĨA CHÂN CNC SHIELD V4
//                chân ARDUINO   ký hiệu trên          chân PORT AVR
//                             board Arduino nano       Atmega 328P
# define Enable       8            //D8                 //PORTB 0                                      
# define Step_2       6            //D6                 //PORTD 6                    
# define Step_1       5            //D5                 //PORTD 5                                    
# define Dir_2        3            //D3                 //PORTD 3                    
# define Dir_1        2            //D2                 //PORTD 2  

//     HÀM KHAI BÁO CÁC CHÂN ARDUINO NANO
//....................................
void  pin_INI() {
  pinMode(Enable, OUTPUT);
  pinMode(Step_1, OUTPUT);
  pinMode(Step_2, OUTPUT);
  pinMode(Dir_1, OUTPUT);
  pinMode(Dir_2, OUTPUT);
  digitalWrite(Enable, LOW);
}
//     HÀM KHAI BÁO TIMER2
//....................................
void timer_INI() {

  TCCR2A = 0;                                                               //Gán giá trị thanh ghi TCCR2A là 0
  TCCR2B = 0;                                                               //Gán giá trị thanh ghi TCCR2B là 0
  TCCR2B |= (1 << CS21);                                                    //Set bit CS21 ở thanh ghi TCCR2B lên 1 
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000Hz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode Chế độ CTC bộ đếm được xóa về 0 khi giá trị bộ đếm (TCNT0) khớp với OCR0A
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
}

int8_t Dir_M1, Dir_M2;                                               //Biến xác định hoạt động của động cơ và chiều quay Dir_Mx >0 quay thuận , Dir_Mx <0 quay ngược Dir_Mx =0 motor ngừng quay
volatile int Count_timer1, Count_timer2;                       //đếm các lần TIMER xuất hiện trong chu kỳ xung STEP
volatile int32_t Step1, Step2;
int16_t Count_TOP1, Count_BOT1, Count_TOP2, Count_BOT2;  //vị trí cuối của phần cao và cuối phần thấp trong 1 chu kỳ xung STEP
float Input_L, Input_R, Offset, Output, I_L, I_R, Input_lastL, Input_lastR, Output_L, Output_R, M_L, M_R, Motor_L, Motor_R, Vgo;
float Kp = 25;
float Ki = 0.01;
float Kd = 0.5;
unsigned long loop_timer;

//     CHƯƠNG TRÌNH NGẮT CỦA TIMER2
//....................................
ISR(TIMER2_COMPA_vect) {
  //tạo xung STEP cho MOTOR1
  if (Dir_M1 != 0) {                                                          //nếu MOTOR cho phép quay
    Count_timer1++;
    if (Count_timer1 <= Count_TOP1)PORTD |= 0b00100000;                        //nếu là nhịp nằm trong phần cao trong xung STEP
    else PORTD &= 0b11011111;                                                 //nếu là nhịp nằm trong phần thấp của xung STEP
    if (Count_timer1 > Count_BOT1) {
      Count_timer1 = 0;                             //nếu là nhịp cuối của 1 xung STEP
      if (Dir_M1 > 0)Step1++;
      else if (Dir_M1 < 0)Step1--;
    }
  }

  //tạo xung STEP cho MOTOR2
  if (Dir_M2 != 0) {
    Count_timer2++;
    if (Count_timer2 <= Count_TOP2)PORTD |= 0b01000000;
    else PORTD &= 0b10111111;
    if (Count_timer2 > Count_BOT2) {
      Count_timer2 = 0;
      if (Dir_M2 > 0)Step2++;
      else if (Dir_M2 < 0)Step2--;
    }
  }

}

//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR_RIGHT
//....................................
void Speed_R(int16_t x) {
  if (x < 0) {
    Dir_M1 = -1;
    PORTD &= 0b11111011;
  }
  else if (x > 0) {
    Dir_M1 = 1;
    PORTD |= 0b00000100;
  }
  else Dir_M1 = 0;

  Count_BOT1 = abs(x);
  Count_TOP1 = Count_BOT1 / 2;
}

//     HÀM TỐC ĐỘ DI CHUYỂN MOTOR_LEFT
//....................................
void Speed_L(int16_t x) {
  if (x < 0) {
    Dir_M2 = -1;
    PORTD &= 0b11110111;
  }
  else if (x > 0) {
    Dir_M2 = 1;
    PORTD |= 0b00001000;
  }
  else Dir_M2 = 0;

  Count_BOT2 = abs(x);
  Count_TOP2 = Count_BOT2 / 2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  mpu6050.init(0x68);
  Serial.begin(9600);               //Khai báo Serial
  pin_INI();                        //Khai báo PIN Arduino đấu nối 3 DRIVER A4988
  timer_INI();                      //Khai báo TIMER2
  delay(500);
    loop_timer = micros() + 4000;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  float AngleY = mpu6050.getYAngle();
  Serial.println(AngleY);
  Offset = -1.8;
  Vgo = 0.05;
  //Dùng PID cho MOTOR_L
  Input_L = AngleY + Offset - Vgo ;                             //Vgo<0  chạy lui,Vgo >0 chạy tới
  I_L += Input_L * Ki;
  I_L = constrain(I_L, -400, 400);
  Output_L = Kp * Input_L + I_L + Kd * (Input_L - Input_lastL);
  Input_lastL = Input_L;                                           //Lưu làm độ lệch trước cho vòng loop sau
  //Khống chế OUTPUT theo sự phi tuyến MOTOR_L
  if (Output_L > -5 && Output_L < 5)Output_L = 0;
  Output_L = constrain(Output_L, -400, 400);


  //Dùng PID cho MOTOR_R
  Input_R = AngleY + Offset - Vgo; //Vgo<0  chạy lui,Vgo >0 chạy tới
  I_R += Input_R * Ki;
  I_R = constrain(I_R, -400, 400);
  Output_R = Kp * Input_R + I_R + Kd * (Input_R - Input_lastR);
  Input_lastR = Input_R;
  //Khống chế OUTPUT theo sự phi tuyến MOTOR_R
  if (Output_R > -5 && Output_R < 5)Output_R = 0;
  Output_R = constrain(Output_R, -400, 400);

  //Khắc phục sự phi tuyến của MOTOR_L
  if (Output_L > 0)M_L = 405 - (1 / (Output_L + 9)) * 5500;
  else if (Output_L < 0)  M_L = -405 - (1 / (Output_L - 9)) * 5500;
  else M_L = 0;

  //Khắc phục sự phi tuyến của MOTOR_R
  if (Output_R > 0)M_R = 405 - (1 / (Output_R + 9)) * 5500;
  else if (Output_R < 0)M_R = -405 - (1 / (Output_R - 9)) * 5500;
  else M_R = 0;

  //Làm ngược giá trị truyền vào hàm Speed_L()
  if (M_L > 0)Motor_L = 396.55 - M_L;
  else if (M_L < 0)Motor_L = -396.55 - M_L;
  else Motor_L = 0;

  //Làm ngược giá trị truyền vào hàm Speed_R()
  if (M_R > 0)Motor_R = 396.55 - M_R;
  else if (M_R < 0)Motor_R = -396.55 - M_R;
  else Motor_R = 0;

  //cho 2 MOTOR chạy
  Speed_L(Motor_L);
  Speed_R(Motor_R);

  while(loop_timer > micros());
  loop_timer += 4000;
}
