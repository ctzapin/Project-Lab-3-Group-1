//Name: Scan-Room Project
//Class: Project Lab 3
//Professor: William Ray
//Date: 09/24/2018
//Author: Weston L. Salinas
//Summary: This is the code for the MSP430G2553 programmable microcontroller.
//External Devices used on this project include: LIDAR Lite V3, Dagu Rover 5
//Chassis, BlueSMiRF Silver Bluetooth Module, and Two Hobby Servo Motors.
//The goal of this project is for the rover that this code will be controlling
//to have two main modes of operation: Scanning Mode, and Path-Follower Mode
//the Scanning mode will increment one servo from 0 to 180 degrees and at each
//angle get a distance measurement with the LIDAR Lite V3. the code will then
//relay that information to a computer GUI over Bluetooth Serial Communications
//The second mode, Path Follower Mode, waits for the user to send a series of
//characters and stores those characters in an array. It will then follow out the
//received directions sending the correct signals to the H-Bridge.
//Input Output Diagram/////////////////////////////////////////////////////
//                  -------
//      (VCC)    --|       |--   (Ground)
//      (P1.0)   --|       |--   (XIN)
//RX Pin(P1.1)  -->|MSP430 |--   (XOUT)
//TX Pin(P1.2)  <--|g2553  |--   (TEST)
//EN A  (P1.3)  <--|       |--   (RST)
//EN B  (P1.4)  <--|       |<--  (P1.7) LIDAR ECHO Input
//IN1   (P1.5)  <--|       |-->  (P1.6) LIDAR Servo Output PWM
//IN2   (P2.0)  <--|       |-->  (P2.5) LIDAR TRIGGER Output
//IN3   (P2.1)  <--|       |-->  (P2.4) Delivery Servo Output PWM
//IN4   (P2.2)  <--|       |<--  (P2.3) Current Sensing Pin
//                  -------
//Includes/////////////////////////////////////////////////////////////////
#include <msp430g2553.h>                                                   //Include MSP430g2553 header file
#include <stdio.h>                                                         //Include standard IO header file
#include <stdlib.h>                                                        //Include Standard Library header file
#include <math.h>                                                          //include math header for power function
//Global Variables/////////////////////////////////////////////////////////
char Received_Number = 0;                                                  //The byte character number that is received.
int Boolean_Number = 0;                                                    //Flag for when rover receives a number
long Total_Number = 0;                                                      //The calculated duration of time that the rover will do an action
long The_Miliseconds = 0;                                                  //Number of milliseconds that the timer has incremented
int Boolean_Left = 0;                                                      //Flag for going left
int Boolean_Right = 0;                                                     //Flag for going right
int Boolean_Forward = 0;                                                   //Flag for going forward
int Boolean_Begin_Scanning = 0;                                            //Flag for Start Scanning Mode
int Boolean_Stop_Scanning = 0;                                             //Flag for Stop Scanning Mode
int Boolean_Begin_Path_Follower = 0;                                       //Flag For Begin path Follower Mode
int Boolean_Stop_Path_Follower = 0;                                        //Flag For Stop Path Follower Mode
int Boolean_Prepare_For_Directions = 0;                                    //Flag for preparing for directions
int Boolean_Finish_Directions = 0;                                         //Flag for finishing directions
int Forward_Distance = 0;                                                  //Forward distance (cm)
int Left_Distance = 0;                                                     //Left distance (cm)
int Right_Distance = 0;                                                    //Right distance (cm)
int Miliseconds = 0;                                                       //milliseconds counter
int Distance1 = 0;                                                         //distance1 value from LIDAR
int Distance2 = 0;                                                         //distance2 value from LIDAR
int Distance3 = 0;                                                         //distance3 value from LIDAR
int Final_Distance = 0;                                                    //Averaged distance value from LIDAR
int Current_Too_High_Boolean = 0;                                          //current sensing boolean
long Sensor = 0;                                                           //Unaltered amount of microseconds output from the LIDAR
int Angle = 0;                                                             //Angle of the servo in degrees
char number[20];                                                           //Each Character of the Distance value being transmitted
int Went_Low = 0;                                                          //Boolean for when falling edge is detected on echo pin
char Direction[100];                                                       //List of Directions
int Num_Of_Digits = 0;                                                     //Number of digits that a duration is.
unsigned char NumberOne = 0x31;
//Function Declarations/////////////////////////////////////////////////////
void Carry_Out_Directions();
void Initial_Setup();
void General_Pin_Setup();
void UART_Pin_Mux();
void UART_Baud_Setup();
void UART_Interrupt_Setup();
void Servo_Pos(int pos);
void Scanning_Mode();
void Path_Follower_Mode();
void Scan_Room();
void Turn_Left(long milliseconds);
void Turn_Right(long milliseconds);
void Go_Forward(long milliseconds);
void Transmit_Number(int num);
void Delay_ms(long milliseconds);
long Calculate_Delay(int Start_Index);
void Delivery_Servo_Pos(int pos);
//Main Loop////////////////////////////////////////////////////////////////
int main(void)
{
   Initial_Setup();
   General_Pin_Setup();
   UART_Pin_Mux();
   UART_Baud_Setup();
   UART_Interrupt_Setup();

   while(1)                                                                //Waiting for serial input
   {
       if (Boolean_Begin_Scanning)                                         //If Begin Scanning variable is set to 1
       {
           Boolean_Begin_Scanning = 0;                                     //Set Boolean to 0
           Boolean_Stop_Scanning = 0;                                      //Set Boolean to 0
           Scanning_Mode();                                                //scanning mode function
       }
       if (Boolean_Begin_Path_Follower)                                    //If Begin path follower mode is set to 1
       {
           Boolean_Begin_Path_Follower = 0;                                //Set Boolean to 0
           Boolean_Stop_Path_Follower = 0;                                 //Set Boolean to 0
           Path_Follower_Mode();                                           //path follower function
       }

   }
}
//Initial Setup Function//////////////////////////////////////////////////
void Initial_Setup()
{
    //General Timer Setup/////////////////////////////////////////////////
    WDTCTL = WDTPW + WDTHOLD;                                              // Stop watch dog timer
    DCOCTL = 0;                                                            // reset clock to zero
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;                                                  // submainclock 1mhz
    P1IFG  = 0x00;                                                         //clear all interrupt flags
    //Servo Timer Setup///////////////////////////////////////////////////
    TA0CCR0 = 20000-1;                                                     // PWM Period TA0.1
    TA0CCR1 = 1500;                                                        // CCR1 PWM duty cycle
    TA0CCTL1 = OUTMOD_7;                                                   // CCR1 reset/set
    TA0CTL   = TASSEL_2 + MC_1;                                            // SMCLK, up mode
    //Delivery Servo Setup////////////////////////////////////////////////
    P2SEL |= BIT4;
    P2DIR |= BIT4;                                                         //Set P2.4 to output-direction
    P2OUT &= ~BIT4;                                                        //P2.4 set low
    TA1CCTL2  = OUTMOD_7;                                                  // Reset/set
    TA1CTL    = TASSEL_2 + MC_1;                                           // SMCLK, timer in up-mode
    TA1CCR0 = 20000-1;                                                     // PWM Period
    TA1CCR2  = 1500;                                                       //  TA0CCR1 PWM Duty Cycle
    //Milisecond Timer Setup//////////////////////////////////////////////
    TA1CTL = TASSEL_2 + MC_1;                                              // SMCLK, upmode
    TA1CCR0 = 1000;                                                        //Set TA1.0 to 1000 for milisecond counter
    TA1CCTL0 = CCIE;                                                       //TA1.0 interrupt enabled
    return;
}
//Pin Setup FUnction//////////////////////////////////////////////////////
void General_Pin_Setup()
{
    P1DIR &= ~(BIT7);
    P1DIR |= (BIT0 + BIT3 + BIT4 + BIT5 + BIT6);                          //all outputs port 1
    P2DIR |= (BIT0 + BIT1 + BIT2 + BIT4 + BIT5); //new                         //all outputs p2
    P1OUT &= ~(BIT3 + BIT4 + BIT5 + BIT6);                                //initialize to 0
    P2OUT &= ~(BIT0 + BIT1 + BIT2 + BIT4 + BIT5);//new                         //initialize to 0
    P2OUT |= (BIT5);                                                      //initialize to one so that the lidar sensor isnt taking reading
    P1SEL |= BIT6;                                                        // P1.6 select TA0.1 option
    P2SEL &= ~BIT3;
    return;
}
//UART Pin Multiplexing///////////////////////////////////////////////////
void UART_Pin_Mux()
{
    P1SEL |= (BIT1 + BIT2);                                                //Set P1.1 is RX pin and P1.2 is TX pin
    P1SEL2 |= (BIT1 + BIT2);                                               //Set P1.1 is RX pin and P1.2 is TX pin
    return;
}
//UART Baud Rate Setup////////////////////////////////////////////////////
void UART_Baud_Setup()
{
    UCA0CTL1 = UCSWRST;                                                    //UCA0 set in reset to be configured
    UCA0CTL1 |= UCSSEL_2;                                                  //SMCLK
    UCA0BR0 = 0x08;                                                        //115200
    UCA0BR1 = 0x00;                                                        //115200
    UCA0MCTL = UCBRS_2 + UCBRS0;                                           //Set baud rate generator for 115200 Modulation
    UCA0CTL1 &= ~(UCSWRST);                                                //take USCI out of reset
    return;
}
//UART Interrupt Setup////////////////////////////////////////////////////
void UART_Interrupt_Setup()
{
    IE2 |= UCA0RXIE;                                                       //Enable RX interrupt
    IE2 |= UCA0TXIE;                                                       // Enable the Transmit interrupt
    _BIS_SR(GIE);                                                          // Enable Global Interrupts
    return;
}
//Servo Position Setup////////////////////////////////////////////////////
void Servo_Pos(int pos){
    TA0CCR1 = (pos+35)*11;                                                 // 11 = 1 degree
}

void Delivery_Servo_Pos(int pos){//new
    TA1CCR2  = (pos+35)*11;
}//new
void Delay_ms(long milliseconds)
{
    The_Miliseconds = 0;                                                   //initialize miliseconds to 0
    while (The_Miliseconds < milliseconds)                                 //while the counter is less than the set value
    {
        if (Current_Too_High_Boolean == 1)
                {
                    Stop_Moving();
                    Current_Too_High_Boolean = 0;
                    return;
                }
    }
    return;
}
//Scanning Mode///////////////////////////////////////////////////////////
void Scanning_Mode()
{
    while(1)
    {
        Scan_Room();                                                      //get initial scan of room
        Servo_Pos(0);                                                     //set the servo back to 180
        Delay_ms(600);                                                    //set delay to let servo get to the new position
        Angle = 0;                                                        //set the angle of the servo starting position back to 0

        if (Forward_Distance >= 40)                                       //if front isnt obstructed
        {
            Go_Forward(1050);                                             //go forward
        }
        else if ((Left_Distance < 40) && (Forward_Distance < 40) && (Right_Distance >= 40) )//if left is obstructed and forward is obstructed but right isnt
        {
            Turn_Right(828);                                             //go right
        }
        else if ((Left_Distance >= 40) && (Forward_Distance < 40) && (Right_Distance < 40) )//if left is not obstructed
        {
            Turn_Left(828);                                              //go left
        }
        else if ((Left_Distance < 40) && (Forward_Distance < 40) && (Right_Distance < 40) ) //if all three are obstructed
        {
            Turn_Right(828);                                             //go right
            Delay_ms(100);
            Turn_Right(828);                                             //go right
        }
        else if ((Left_Distance >= 40) && (Forward_Distance < 40) && (Right_Distance >= 40) )//if front is only obstructed
        {
            Turn_Right(828);                                             //go right
        }
        if (Boolean_Stop_Scanning == 1)                                   //if stop scanning character received
        {
            Boolean_Stop_Scanning = 0;                                    //stop scanning back to 0
            return;
        }
    }
}
//Path Follower Mode//////////////////////////////////////////////////////
void Path_Follower_Mode()
{
  while(1)                                                               //infinite while loop
  {
    while (Boolean_Prepare_For_Directions == 0)                          //while prepare for directions is 0
    {
        if (Boolean_Stop_Path_Follower == 1)                             //if the stop path follower command is received
        {
            Boolean_Stop_Path_Follower = 0;                              //set it back to 0
            return;                                                      //exit function
        }
    }
    Boolean_Prepare_For_Directions = 1;
    int Current_Num = 0;                                                 //current direction being received = 0
    int i;                                                               // int i setting whole array back to '\0'
    for (i = 0 ; i<100; i++)
    {
        Direction[i]='\0';                                               //set each previous direction back to 0
    }

    while(1)                                                             //while receiving directions
    {
        if (Boolean_Left == 1)                                           //if left received
        {
            Boolean_Left = 0;                                            //set it back to 0
            Direction[Current_Num] = 'l';                                //add to directions
            Current_Num = Current_Num + 1;                               //increase counter
        }
        if (Boolean_Right == 1)                                          //if right received
        {
            Boolean_Right = 0;                                           //set it back to 0
            Direction[Current_Num] = 'r';                                //add to directions
            Current_Num = Current_Num + 1;                               //increase counter
        }
        if (Boolean_Forward == 1)                                        //if forward received
        {
            Boolean_Forward = 0;                                         //set it back to 0
            Direction[Current_Num] = 'f';                                //add to directions
            Current_Num = Current_Num + 1;                               //increase counter
        }
        if (Boolean_Number == 1)                                         //if a possible number was received
        {

            IE2 &= ~UCA0RXIE;                                            //disable rx interrupt for a bit so it doesn't overwrite
            if (Received_Number == '0' || Received_Number == '1' ||Received_Number == '2' ||Received_Number == '3' ||Received_Number == '4'||Received_Number == '5' ||Received_Number == '6' ||Received_Number == '7' ||Received_Number == '8' ||Received_Number == '9' )//if its a number indeed
            {
                Direction[Current_Num] = Received_Number;                //add number to the array
                Current_Num = Current_Num + 1;                           //increment counter
                Received_Number = '\0';                                  //set received number back to nothing
            }
            Boolean_Number = 0;                                          //set flag back to 0
            IE2 |= UCA0RXIE;                                             //re enable the interrupt
        }

        if (Boolean_Finish_Directions == 1)                              //if directions finished flag set high
        {
            Boolean_Finish_Directions = 0;                               //set it low again
            Carry_Out_Directions();                                      //do the directions
            break;                                                       //break while loop
        }

        if (Boolean_Stop_Path_Follower == 1)                             //if path follower mode is stopped
        {
            Boolean_Stop_Path_Follower = 0;                              //set flag back low
            return;                                                      //exit function
        }
    }
  }

}
//Carry Out Directions Function////////////////////////////////////////////////
void Carry_Out_Directions()
{
    long duration = 0;                                                        //set direction to 0
    int direction_num = 0;                                                    //set the counter to 0
    for (direction_num = 0; direction_num < 100; direction_num = direction_num + 1 )//for 100 directions (or until the for loop is broken from)
    {
       if (Direction[direction_num] == '\0')                                  //if an empty character is in the directions
       {
           break;                                                             //break from for loop
       }
       else if(Direction[direction_num] == 'l')                               //if left character is next direction
       {
           duration = Calculate_Delay(direction_num +1);                      //calculate the amount of time and set it to duration
           Turn_Left(duration);                                               //turn left for the calculated duration
           direction_num = direction_num + Num_Of_Digits;                     //set the index to the value right after the last number
       }
       else if(Direction[direction_num] == 'r')                               //if right character is next direction
       {
           duration = Calculate_Delay(direction_num+1);                       //calculate the amount of time and set it to duration
           Turn_Right(duration);                                              //turn right for calculated amount of time
           direction_num = direction_num + Num_Of_Digits;                     //set the index to the value right after the last number
       }
       else if(Direction[direction_num] == 'f')                               //if forward character is next direction
       {
           duration = Calculate_Delay(direction_num+1);                       //calculate the amount of time and set it to duration
           Go_Forward(duration);                                              //go forward for calculated duration
           direction_num = direction_num + Num_Of_Digits;                     //set the index to the index right after the last digit
       }

    }
    P2SEL |= BIT4;
    TA1CCR0 = 20000-1;
    Delivery_Servo_Pos(20);//new
    Delay_ms(200);
    Delivery_Servo_Pos(180);//new
    Delay_ms(200);
    TA1CCR0 = 1000;
    P2SEL &= ~BIT4;

}
//Calculate Delay Function///////////////////////////////////////////////////
//This function will take all of the digits received after a direction and then
//will calculate what the number is and return a long of the duration.
long Calculate_Delay(int Start_Index)
{
    int index = Start_Index;                                                 //set index to inputed start index
    Num_Of_Digits = 0;                                                       //number of digits counted to 0
    Total_Number = 0;                                                        //total number read = 0
    while(1)                                                                 //while loop counting number of digits
    {
        if (Direction[index] == '0' || Direction[index] == '1' || Direction[index] == '2' || Direction[index] == '3' || Direction[index] == '4' || Direction[index] == '5' || Direction[index] == '6' || Direction[index] == '7' || Direction[index] == '8'|| Direction[index] == '9')//if its a numeric value
        {
            Num_Of_Digits = Num_Of_Digits + 1;                               //increment number of digits
            index = index + 1;                                               //index = index + 1
        }
        else                                                                 //if its not a number then you must have reached end of duration val
        {
          break;                                                             //break from while loop
        }
    }
    index = Start_Index;                                                     //set index back to start index
    int Current_Digit = (Num_Of_Digits - 1);                                 //set current digit to number of digits - 1

    for (Current_Digit;Current_Digit >=0; Current_Digit = Current_Digit - 1) //for loop calculating the long value of the duration characters in array
    {
        if (Direction[index] == '1')                                         //if its a '1'
        {
            Total_Number = Total_Number + 1*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '2')                                     //if its a '2'
        {
            Total_Number = Total_Number + 2*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '3')                                     //if its a '3'
        {
            Total_Number = Total_Number + 3*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '4')                                     //if its a '4'
        {
            Total_Number = Total_Number + 4*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '5')                                     //if its a '5'
        {
            Total_Number = Total_Number + 5*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '6')                                     //if its a '6'
        {
            Total_Number = Total_Number + 6*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '7')                                     //if its a '7'
        {
            Total_Number = Total_Number + 7*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '8')                                     //if its a '8'
        {
            Total_Number = Total_Number + 8*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '9')                                     //if its a '9'
        {
            Total_Number = Total_Number + 9*(pow(10, Current_Digit));
            index = index + 1;
        }
        else if(Direction[index] == '0')                                     //if its a '10'
        {
            Total_Number = Total_Number;
            index = index + 1;

        }

    }


    return(Total_Number);
}
//Scan Room Function//////////////////////////////////////////////////////
void Scan_Room()

{
    for(Angle = 0; Angle < 181 ; Angle = Angle + 10)
    {

        Servo_Pos(Angle);                                                  //move the servo to the Angle value
        Delay_ms(200);                                                     //give the servo time to move to position
        Final_Distance = 0;                                                //Averaged distance = 0
        Distance1 = 0;                                                     //Distance 1 = 0
        Distance2 = 0;                                                     //Distance 2 = 0
        Distance3 = 0;                                                     //Distance 3 = 0
        P1IE &= ~0x01;                                                     // disable interrupt
        P2OUT &= ~(BIT5);                                                  // generate PWM  (P2.5 gets this value)
        Delay_ms(100);                                                     // for 10us
        P1DIR &= ~(BIT7);                                                  // make pin P1.7 input (ECHO)
    //While Loop Measuring distances inside acceptable tolerance////////////
        while (Final_Distance >= (Distance1 + (.05 * Final_Distance)) || Final_Distance == 0 || Final_Distance <= (Distance1 - (.05 * Final_Distance))) // the average has to be within +/- 2 cm of the first distance to be accepted
        {
            Final_Distance = 0;                                            //Averaged distance = 0
            Distance1 = 0;                                                 //Distance 1 = 0
            Distance2 = 0;                                                 //Distance 2 = 0
            Distance3 = 0;                                                 //Distance 3 = 0
    //Distance 1 Measurement///////////////////////////////////////////////
            P1IFG = 0x00;                                                  // clear flag just in case anything happened before
            P1IE |= (BIT7);                                                // enable interupt on ECHO pin
            P1IES &= ~(BIT7);                                              // rising edge on ECHO pin
            Went_Low = 0;                                                  // went low = 0
            while (Went_Low == 0)                                          //while the falling edge hasnt been found
            {
                if (Went_Low == 1)                                         //if falling edge is found
                {
                    break;                                                 //break
                }
            }
            Distance1 = ((.08979618104 * Sensor) - 6.94633);               // converting echo length to distance in cm's

    //Distance 2 Measurement///////////////////////////////////////////////
            P1IFG = 0x00;                                                  // clear flag just in case anything happened before
            P1IE |= (BIT7);                                                // enable interupt on ECHO pin
            P1IES &= ~(BIT7);                                              // rising edge on ECHO pin
            Went_Low = 0;                                                  // went low = 0
            while (Went_Low == 0)                                          //while the falling edge hasnt been found
            {
                if (Went_Low == 1)                                         //if falling edge is found
                {
                    break;                                                 //break
                }
            }
            Distance2 = ((.08979618104 * Sensor) - 6.94633);               // converting echo length to distance in cm's
    //Distance 3 Measurement///////////////////////////////////////////////
            P1IFG = 0x00;                                                  // clear flag just in case anything happened before
            P1IE |= (BIT7);                                                // enable interupt on ECHO pin
            P1IES &= ~(BIT7);                                              // rising edge on ECHO pin
            Went_Low = 0;                                                  // went low = 0
            while (Went_Low == 0)                                          //while the falling edge hasnt been found
            {
                if (Went_Low == 1)                                         //if falling edge is found
                {
                    break;                                                 //break
                }
            }
            Distance3 = ((.08979618104 * Sensor) - 6.94633);               // converting echo length to distance in cm's
     //Finding Average Distance////////////////////////////////////////////
            Final_Distance = (Distance1 + Distance2 + Distance3 )/3;       //find the average of the three measured distances
        }
        P2OUT |= (BIT5);                                                   //make pin 2.5 high to stop measurements
        P1IE &= ~0x01;                                                     //Disable interrupt

        if (Angle == 180)                                                    //if the angle is 0 (right)
        {
            Right_Distance = Final_Distance;                               //set right_distance to the averaged distance
        }
        if (Angle == 90)                                                   //if angle is 90 degrees (forward)
        {
            Forward_Distance = Final_Distance;                             //set forward_distance to averaged distance
        }
        if (Angle == 0)                                                  //if angle is 180 degrees (left)
        {
            Left_Distance = Final_Distance;                                //set left_distance to averaged distance
        }


        UCA0TXBUF = 'C';                                                   //transmit the C character indicating incoming angle
        Delay_ms(100);
        Transmit_Number(180-Angle);                                        //transmit angle with transmit number function
        Delay_ms(100);
        UCA0TXBUF = 'D';                                                   //transmit D character indicating incoming distance
        Delay_ms(100);
        Transmit_Number(Final_Distance);                                   //transmit averaged distance using transmit number functio
        Delay_ms(100);                                         //delay
        UCA0TXBUF = ',';
    }
}

//Turn Rover left for set amount of milliesconds///////////////////////////
void Turn_Left(long milliseconds)
{
    P1OUT |= (BIT3 + BIT4 + BIT5);                                         //set appropriate pins to go left
    P2OUT |= (BIT1);
    Delay_ms(milliseconds);                                                //delay for right amount of milliseconds
    P1OUT &= ~(BIT3 + BIT4 + BIT5);                                        //set appropriate pins back low
    P2OUT &= ~(BIT1);
    UCA0TXBUF = 'L';                                                       //transmit 'L'
    Transmit_Number(milliseconds);                                         //Transmit the number of milliseconds it turned for
    Delay_ms(100);
    UCA0TXBUF = ',';
    return;
}
//turn rover right for set amount of milliseconds//////////////////////////
void Turn_Right(long milliseconds)
{
    P1OUT |= (BIT3 + BIT4);                                               //set appropriate pins to go right
    P2OUT |= (BIT0 + BIT2);
    Delay_ms(milliseconds);                                               //delay for right amount of milliseconds
    P1OUT &= ~(BIT3 + BIT4);                                              //set appropriate pins back low
    P2OUT &= ~(BIT0 +BIT2);
    UCA0TXBUF = 'R';                                                      //Transmit 'R'
    Transmit_Number(milliseconds);                                        //Transmit Number of milliseconds it turned for
    Delay_ms(100);
    UCA0TXBUF = ',';
    return;
}
//Move Rover Forward for set milliseconds//////////////////////////////////
void Go_Forward(long milliseconds)
{
    P1OUT |= (BIT3 + BIT4 + BIT5);                                        //set appropriate pins to go forward
    P2OUT |= (BIT2);
    Delay_ms(milliseconds);                                               //delay for right amount of milliseconds
    P1OUT &= ~(BIT3 + BIT4 + BIT5);                                       //set appropriate pins back low
    P2OUT &= ~(BIT0 +BIT2);
    P2OUT &= ~(BIT2);
    UCA0TXBUF = 'F';                                                      //transmit 'F'
    Transmit_Number(milliseconds);                                        //Transmit Number of milliseconds it turned for
    Delay_ms(100);
    UCA0TXBUF = ',';
    return;
}
void Stop_Moving()
{
    P1OUT &= ~(BIT3 + BIT4 + BIT5);                                       //set appropriate pins back low
    P2OUT &= ~(BIT0 + BIT1 + BIT2);
}
//Transmit Integer digit by digit as characters/////////////////////////////
void Transmit_Number(int num)                                             //transmit an integer to the characters representing it
{
    int i;
    for (i = 0; i<11; i++)                                                //for loop clearing the values that were previously in number[]
    {
        if (i == 0)
            number[i]='0';                                                //set first one to 0
        else
            number[i]='\0';                                               //set rest to '\0'
    }
    int increment = 0;
    while (num > 0)                                                       //while loop going through each digit of the integer number
    {
     int digit = num % 10;                                                //taking modulus 10 of number

     if (digit == 0)
     number[increment] = '0';                                             //setting the number at index to '0'
     else if (digit == 1)
     number[increment] = '1';                                             //setting the number at index to '1'
     else if (digit == 2)
     number[increment] = '2';                                             //setting the number at index to '2'
     else if (digit == 3)
     number[increment] = '3';                                             //setting the number at index to '3'
     else if (digit == 4)
     number[increment] = '4';                                             //setting the number at index to '4'
     else if (digit == 5)
     number[increment] = '5';                                             //setting the number at index to '5'
     else if (digit == 6)
     number[increment] = '6';                                             //setting the number at index to '6'
     else if (digit == 7)
     number[increment] = '7';                                             //setting the number at index to '7'
     else if (digit == 8)
     number[increment] = '8';                                             //setting the number at index to '8'
     else if (digit == 9)
     number[increment] = '9';                                             //setting the number at index to '9'
     num /= 10;
     increment = increment+1;
    }
    for (increment+1;increment>(-1);increment-=1)                         //for loop transmitting the calculated number[]
    {
        UCA0TXBUF = number[increment];                                    //transmitting each character
        Delay_ms(100);                                         //delaying
    }

}
//Echo Pin Port Interrupt//////////////////////////////////////////////
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    if(P1IFG & BIT7)                                                  //is there interrupt pending?
        {
          if(!(P1IES & BIT7))                                         // is this the rising edge?
          {
            TACTL|=TACLR;                                             // clears timer A
            Miliseconds = 0;                                          //sets milliseconds counter to 0
            P1IES |= BIT7;                                            // falling edge
          }
          else
          {
           Sensor = (long)TAR + (long)Miliseconds*1000 ;              //calculating ECHO length
           Went_Low = 1;                                              //set went low back to 1
          }
    P1IFG &= ~BIT7;                                                   //clear flag
    }
}

//TX Interrupt///////////////////////////////////////////////////////
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    IFG2 &= ~UCA0TXIFG;                                               // Clear TX flag
}

//RX Interrupt///////////////////////////////////////////////////////
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if(UCA0RXBUF == 'A' )                                            //if the "A" character is received"
    {
        Boolean_Begin_Scanning = 1;
    }
    else if(UCA0RXBUF == 'B' )                                       //if the "B" character is received"
    {
        Boolean_Stop_Scanning = 1;
    }
    else if(UCA0RXBUF == 'a')                                        //if the "a" character is received"
    {
        Boolean_Begin_Path_Follower = 1;
    }
    else if(UCA0RXBUF == 'b')                                        //if the "b" character is received"
    {
        Boolean_Stop_Path_Follower = 1;
    }
    else if(UCA0RXBUF == 'c')                                        //if the "c" character is received"
    {
        Boolean_Prepare_For_Directions = 1;
    }
    else if(UCA0RXBUF == 'd')                                        //if the "d" character is received"
    {
        Boolean_Finish_Directions = 1;
    }
    else if(UCA0RXBUF == 'l')                                        //if the "l" character is received"
    {
        Boolean_Left = 1;
    }
    else if(UCA0RXBUF == 'r')                                        //if the "r" character is received"
    {
        Boolean_Right = 1;
    }
    else if(UCA0RXBUF == 'f')                                        //if the "f" character is received"
    {
        Boolean_Forward = 1;
    }
    else                                                             //this is presumably a number
    {
            Boolean_Number = 1;
            Received_Number = UCA0RXBUF;
    }
  IFG2 &= ~UCA0RXIFG;                                                //clear RX flag
}

//TA1.0 Timer Interrupt/////////////////////////////////////////////
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A (void)
{
  Miliseconds++;                                                     //increment miliseconds
  The_Miliseconds++;

  if ((P2IN & BIT3)==BIT3)
  {
      Current_Too_High_Boolean = 1;
  }
  else
  {
      Current_Too_High_Boolean = 0;
  }


}
//End of Program/////////////////////////////////////////////////////
