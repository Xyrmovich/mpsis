#include <msp430.h>

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    /**
     * Initial LEDs setup (LED 6 and LED 7)
     */
    P1DIR |= BIT3;
    P1DIR |= BIT4;
    P1OUT = 0;
    /**
     * Initial button setup
     */
    P1REN |= BIT7;
    P1OUT |= BIT7;

    P2REN |= BIT2;
    P2OUT |= BIT2;

    int button1_flag = 0;
    int button2_flag = 0;
    int led1_state = 0;

    while (1)
    {
        int button1 = (P1IN & BIT7) == 0;
        int button2 = (P2IN & BIT2) == 0;

        if (button2 == 1 && button2_flag == 0)
        {
            if (button1 == 0)
            {
                P1OUT &= ~BIT4;
            }
            button2_flag = 1;
        }
        if (button1 == 0 && button1_flag == 1)
        {
            P1OUT |= BIT4;
            if (button2 == 1)
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
            button1_flag = 0;
        }
        if (button1 == 1 && button1_flag == 0)
        {
            button1_flag = 1;
        }
        if (button2 == 0 && button2_flag == 1)
        {
            button2_flag = 0;
        }

        int i = 0;
        for(i = 0; i < 1000; ++i);
    }

    return 0;
}
