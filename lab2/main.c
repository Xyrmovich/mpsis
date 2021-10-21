#include <msp430.h>

/*
* Task:
*  Default frequency : 55800Hz
*  Alternative frequency : 3467Hz (default frequency / 16)
*  Clock source: DCOCLKDIV
*  Divider: 4
*  Low Power Mode (LPM): 1
*/

volatile int short ALTERNATIVE_FREQUENCY_ON = 0;
volatile int short LPM_MODE_1_ON = 0;
volatile int short DELAY_CYCLES = 1000;

#pragma vector = PORT1_VECTOR
__interrupt void PORT1_S1(void)
{
    __delay_cycles(DELAY_CYCLES);

    if (LPM_MODE_1_ON)
    {
        _bic_SR_register_on_exit(LPM1_bits);
        LPM_MODE_1_ON = 0;
    }
    else
    {
        LPM_MODE_1_ON = 1;
        _bis_SR_register_on_exit(LPM1_bits);
    }

    P1IFG &= ~BIT7;
}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2_S2(void)
{
    __delay_cycles(DELAY_CYCLES);

    if (ALTERNATIVE_FREQUENCY_ON)
    {
        UCSCTL4 = SELM__DCOCLK;
        UCSCTL5 = DIVM__1;
        ALTERNATIVE_FREQUENCY_ON = 0;
    }
    else
    {
        UCSCTL4 = SELM__DCOCLKDIV;
        UCSCTL5 = DIVM__16;
        ALTERNATIVE_FREQUENCY_ON = 1;
    }

    P2IFG &= ~BIT2;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;

    P1DIR &= ~BIT7;
    P1OUT |= BIT7;
    P1REN |= BIT7;

    P2DIR &= ~BIT2;
    P2OUT |= BIT2;
    P2REN |= BIT2;

    __bis_SR_register(GIE); //Enabling CPU interrupts

    P1IES |= BIT7;
    P1IFG &= ~BIT7;
    P1IE |= BIT7;

    P2IES |= BIT2;
    P2IFG &= ~BIT7;
    P2IE |= BIT2;

    P7DIR |= BIT7;
    P7SEL |= BIT7;

    //DCOCLK = FLLREFCLK / FLLREFDIV * (FLLN + 1) * FLLD

    UCSCTL1 |= DCORSEL_0;
    UCSCTL2 |= (FLLD__1 & FLLN6);
    UCSCTL3 |= (SELREF__REFOCLK & FLLREFDIV__4);
    UCSCTL4 |= SELM__DCOCLK;
    UCSCTL5 |= DIVM__1;

    return 0;
}