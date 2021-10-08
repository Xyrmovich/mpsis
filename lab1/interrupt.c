#include <msp430.h>

int button1_flag = 0;
int button2_flag = 0;
int led1_state = 0;
int led2_state = 0;

#pragma vector = PORT1_VECTOR
__interrupt void button1Handler(void)
{
    if (button1_flag == 0)
    {
        P1IES &= ~BIT7;
        button1_flag = 1;
    }
    else
    {
        if (button2_flag == 1)
        {
            if (led1_state == 0)
            {
                P1OUT |= BIT3;
                led1_state = 1;
            }
            else
            {
                P1OUT &= ~BIT3;
                led1_state = 0;
            }
        }
        if (led2_state == 0)
        {
            P1OUT |= BIT4;
            led2_state = 1;
        }
        P1IES |= BIT7;
        button1_flag = 0;
    }
    volatile int i = 0;
    for (i = 0; i < 1000; i++)
    {
    }
    P1IFG = 0;
}

#pragma vector = PORT2_VECTOR
__interrupt void button2Handler(void)
{
    if (button2_flag == 0)
    {
        if (button1_flag == 0)
        {
            if (led2_state == 1)
            {
                P1OUT &= ~BIT4;
                led2_state = 0;
            }
        }
        P2IES &= ~BIT2;
        button2_flag = 1;
    }
    else
    {
        P2IES |= BIT2;
        button2_flag = 0;
    }
    volatile int i = 0;
    for (i = 0; i < 1000; i++)
    {
    }
    P2IFG = 0;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    P1DIR |= BIT3;
    P1DIR |= BIT4;
    P1OUT = 0;

    P1REN |= BIT7;
    P1OUT |= BIT7;

    P2REN |= BIT2;
    P2OUT |= BIT2;

    __bis_SR_register(GIE);
    P1IE |= BIT7;
    P2IE |= BIT2;
    P1IES |= BIT7;
    P2IES |= BIT2;
    P1IFG = 0;
    P2IFG = 0;

    __no_operation();

    return 0;
}