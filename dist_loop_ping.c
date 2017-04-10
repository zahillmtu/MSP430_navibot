//========================================================================== BOF
// FILE FUNC: Send a pulse out of the SRF04 to measure distance 
//------------------------------------------------------------------------------
#include "msp430x22x4.h"
#include "stdint.h"

#define PING_1_TRIG 0x2     // P2.1 = output for trigger pulse to pinger 1
#define PING_2_TRIG 0x4     // P2.2 = output for trigger pulse to pinger 2
#define PING_3_TRIG 0x10    // P2.4 = output for trigger pulse to pinger 3

#define PING_1_RESP 0x8     // P2.3 = input from pinger 1
#define PING_2_RESP 0x8     // P4.3 = input from pinger 2
#define PING_3_RESP 0x80    // P2.7 = input from pinger 3

volatile uint16_t pulseCount;         // Global Pulse Count
volatile uint32_t tStamp1 = 0;        //Time stamp One
volatile uint32_t tStamp2 = 0;        //time stamp Two
volatile uint32_t tLow = 0;          //timeStamp of going low
volatile uint32_t tDiff = 1;          //Diference in Time
volatile uint32_t tHigh = 0;          //length of time High
volatile uint32_t pd = 65536;         //rollover of TA
volatile uint32_t freq = 0;          //Calculated Frequency rounded down to Hz
volatile uint32_t duty = 0;          //Calculated Duty cycle
volatile uint32_t count = 0;
volatile uint32_t period = 0;
volatile uint32_t val = 0;

#pragma vector=TIMERA1_VECTOR
__interrupt void IsrPulseCntTACC1 (void) 
//------------------------------------------------------------------------------
// Func:  At TACCR1 IRQ, take timestamps, using timestamps to calculate frequnecy
//        and duty cycle of singnal input on P2.3. 
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
{ 
  switch (__even_in_range(TAIV, 10))          // I.D. source of TA IRQ
  {                 
    
    case TAIV_TACCR1:                         // handle chnl. 1 IRQ
      //P1OUT ^= 0x1;
	// trigger pulse	
	// while pin == low
	// snapshot time
	// while pin == high
	// snapshot time
	// subtract times
      
        //Rising Edge capture
        //Take Timestamps
        tStamp1 = TACCR1;                    //Save value of previous tStamp
         
        while((P2IN & 0x08)){};
        tStamp2 = TACCR1;                     //Load new timestamp value
        
        //Calculate tDiff
        if(tStamp1<tStamp2)
        {
          tDiff = tStamp2-tStamp1;            //No Rollover
        }
        else
        {
          tDiff = (pd-tStamp1)+tStamp2;    //Accounting for Rollover
        }

        P2IFG      &= ~0x08; //Clear P2.3 Interupt flag
        count++;
        exit();
      
 

      break;
    case TAIV_TACCR2:                  // ignore chnl. 2 IRQ
    case TAIV_TAIFG:                   // ignore TAR rollover IRQ
    default:                           // ignore everything else
  }
}

void InitPorts (void)
//------------------------------------------------------------------------------
// Func:  Initialize ports for I/O on TA1 Capture.
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
{
  P2SEL = 0;
  
  P1DIR  |= 0x1; //Initialize Port 1.0 to output (LED)
  P2DIR  &= ~PING_1_RESP; //Initialize Port 2.3 to input from pinger 1
  P4DIR  &= ~PING_2_RESP; //Initialize Port 4.3 to input from pinger 2
  P2DIR  &= ~PING_3_RESP; //Initialize Port 2.7 to input from pinger 3
  
  P2DIR |= PING_1_TRIG;  // P2.1 = output for trigger pulse to pinger 1
  P2OUT &= ~PING_1_TRIG; // P2.1 = Ensure Output is low
  P2DIR |= PING_2_TRIG;  // P2.2 = output for trigger pulse to pinger 2
  P2OUT &= ~PING_2_TRIG; // P2.2 = Ensure Output is low
  P2DIR |= PING_3_TRIG;  // P2.4 = output for trigger pulse to pinger 3
  P2OUT &= ~PING_3_TRIG; // P2.4 = Ensure Output is low
  
  
  
  // configure LED
  P1OUT &= ~ 0x01;
   
}

/*uint32_t sendPing1()
//------------------------------------------------------------------------------
// Func: Command the pinger to send out a pulse
// Args: The pinger to pulse
// Retn: The time a pulse took to return in microseconds
//------------------------------------------------------------------------------
{
  // Set trigger pulse high
  P2OUT |= 0x2;
  
  volatile uint32_t i = 0;
  for (i = 0; i < 1; i++){ }      // spin to keep pulse high
  
  // set trigger pulse low
  P2OUT &= ~0x2; 
  
  // Waits for rising edge on input
  while(!(P2IN & 0x08)){};
  
  return 0;
} 
*/


uint32_t sendPing(uint8_t pinger)
//------------------------------------------------------------------------------
// Func: Command the pinger to send out a pulse
// Args: The pinger to pulse
// Retn: The time a pulse took to return in microseconds
//------------------------------------------------------------------------------
{
  // Set trigger pulse high
    switch (pinger)
  {

    case 1:
      P2OUT |= PING_1_TRIG;
      break;
    case 2:
      P2OUT |= PING_2_TRIG;
      break;
    case 3:
      P2OUT |= PING_3_TRIG;
      break;
  }
  volatile uint32_t i = 0;
  for (i = 0; i < 1; i++){ }      // spin to keep pulse high
  
  // set trigger pulse low
  switch (pinger)
  {
    case 1:
      P2OUT &= ~PING_1_TRIG; 
      break;
    case 2:
      P2OUT &= ~PING_2_TRIG; 
      break;
    case 3:
      P2OUT &= ~PING_3_TRIG; 
      break;
  }
  
  // Waits for rising edge on input
    switch (pinger)
  {
    case 1:
      while(!(P2IN & PING_1_RESP)){};
      break;
    case 2:
      while(!(P4IN & PING_2_RESP)){};
      break;
    case 3:
      while(!(P2IN & PING_3_RESP)){};
      break;
  }
         
  tStamp1 = TAR;                    //Save value of previous tStamp
   
  // Waits for falling edge on input
    switch (pinger)
  {
    case 1:
      while((P2IN & PING_1_RESP)){};
      break;
    case 2:
      while((P4IN & PING_2_RESP)){};
      break;      
    case 3:
      while((P2IN & PING_3_RESP)){};
      break;
  }
  
  
  tStamp2 = TAR;                     //Load new timestamp value

  //Calculate tDiff
  if(tStamp1<tStamp2)
  {
    tDiff = tStamp2-tStamp1;            //No Rollover
  }
  else
  {
    tDiff = (pd-tStamp1)+tStamp2;    //Accounting for Rollover
  }

  period = (tDiff * 10) / 12;        // calculate period in microseconds  
 
  return period;
}

void test_distance(uint8_t pinger)
//------------------------------------------------------------------------------
// Func:  Set pinger to ping in a loop, when an object gets too close,
//        toggle the LED
// Args:  pinger - the pinger to poll
// Retn:  None
//------------------------------------------------------------------------------
{
    uint32_t time;
    while(1)
    {
        // send a ping
        time = sendPing(pinger);
  
        // if value returned is less than XXXX, toggle LED
        if (time < 5000)
        {
          //P1OUT ^= 0x1;                   // Toggle LED
        }
  
        // wait for at least 100 ms (this is just an uneducated guess)
        volatile uint32_t j = 0;
        for (j = 0; j < 20000; j++)
        {
          
        }
    }
}

void test_two_pingers_distance(uint8_t pingerA, uint8_t pingerB)
//------------------------------------------------------------------------------
// Func:  Set pingerA and pingerB to poll, if both return values lower than
//        XXXX toggle the LED
// Args:  pingerA - the first pinger to poll
//        pingerB - the second pinger to poll
// Retn:  None
//------------------------------------------------------------------------------
{
    uint32_t timeA;
    uint32_t timeB;
    P1OUT ^= 0x1; 
    
    while(1)
    {
        // send a ping
        timeA = sendPing(pingerA);

        // wait for at least 100 ms (this is just an uneducated guess)
        volatile uint32_t j = 0;
        for (j = 0; j < 20000; j++)
        {
          
        }
        
        // send a ping
        timeB = sendPing(pingerB);
        
        // wait for at least 100 ms (this is just an uneducated guess)        
        for (j = 0; j < 20000; j++)
        {
          
        }
        
        // if value returned is less than XXXX, toggle LED
        if ((timeA < 5000) && (timeB < 5000))
        {
          P1OUT ^= 0x1;                   // Toggle LED
        }
    }
}

void main(void)
//------------------------------------------------------------------------------
// Func:  Init I/O ports & IRQs, enter LowPwr Mode.
// Args:  None
// Retn:  None
//------------------------------------------------------------------------------
{ WDTCTL = WDTPW | WDTHOLD;           // Stop Watchdog Timer

  __bic_SR_register (GIE);
  InitPorts();                        // Configure I/O Pins
  
  // Config. UART Clock & Baud Rate for robot commands
  DCOCTL    = CALDCO_1MHZ;           // DCO = 1 MHz
  BCSCTL1   = CALBC1_1MHZ;           // DCO = 1 MHz
  UCA0CTL1 |= UCSSEL_2;              // UART use SMCLK

  UCA0MCTL = UCBRS0;                 // Map 1MHz -> 9600 (Tbl 15-4)
  UCA0BR0  = 104;                    // Map 1MHz -> 9600 (Tbl 15-4)
  UCA0BR1  = 0;                      // Map 1MHz -> 9600 (Tbl 15-4)

  UCA0CTL1 &= ~UCSWRST;              // Enab USCI (unreset)
  
  /*while ( !(IFG2 & UCA0TXIFG)) {};   // Confirm that Tx Buff is empty
  UCA0TXBUF = 0x00;                  // Init: send robot a stop command
  */
  
  //TACTL   = TASSEL_2 | ID_1 | MC_2;   // SMCLK | Div by 2 | Contin. Mode
  TACTL   = TASSEL_2 | MC_2;           // SMCLK | Contin. Mode
  TACCTL1 = CM_1 | CCIS0               // Pos Edge Cap | input = CCI1B 
          | CAP | SCS | CCIE;         // Cap Mode | Sync Cap | Enab IRQ

  //__bis_SR_register (GIE); 
  pulseCount = 0;                     // Init. input pulse counter
  
  
  volatile uint32_t i = 0;
    for (i = 0; i < 5000; i++){ }      
    
  test_two_pingers_distance(1, 2);
  //test_distance(2);
  //val = sendPing(1);
  
  _BIS_SR (LPM1_bits + GIE);          // Enter LPM1 w/ IRQs enab.
}


//========================================================================== EOF