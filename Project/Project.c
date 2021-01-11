#include "msp.h"

void changeState(int switchState);
void takeAction(void);
static uint8_t curr_state = (uint8_t) 0x30;

// Configures UART
void config_UART(void)
{
	CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    // Configure UART pins
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

    // Configure UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
    // Baud Rate calculation
    // 12000000/(16*9600) = 78.125
    // Fractional portion = 0.125
    // User's Guide Table 21-4: UCBRSx = 0x10
    // UCBRFx = int ( (78.125-78)*16) = 2
    EUSCI_A0->BRW = 78;                     // 12000000/16/9600
    EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
            EUSCI_A_MCTLW_OS16;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt

    // Enable sleep on exit from ISR
    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

    // Enable global interrupt
    __enable_irq();

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}


// Configuers GPIO
void config_GPIO(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;          // Stop watchdog timer
	
	P1SEL0 &=~(uint8_t) ((1<<0) | (1<<1) | (1<<4));
	P1SEL1 &=~(uint8_t) ((1<<0) | (1<<1) | (1<<4));
	
	P2SEL0 &=~(uint8_t) (1<<0);
	P2SEL1 &=~(uint8_t) (1<<0);
	
	P1DIR &=~(uint8_t) ((1<<1) | (1<<4));
	P1DIR |=(uint8_t) (1<<0);
	P2DIR |=(uint8_t) (1<<0);
	
	P1OUT |= (uint8_t)((1<<1) | (1<<4));
	P1OUT &=~ (uint8_t)(1<<0);
	P2OUT &=~ (uint8_t)(1<<0);
	
	P1REN |= (uint8_t) ((1<<1) | (1<<4));
	
	// Interrupt Configuration
	P1IES |= (uint8_t) ((1<<1) | (1<<4));
	P1IFG &=~ (uint8_t) ((1<<1) | (1<<4));
	P1IE |= (uint8_t) ((1<<1) | (1<<4));
	
	//NVIC
	NVIC_SetPriority(PORT1_IRQn, 2);
	NVIC_ClearPendingIRQ(PORT1_IRQn);
	NVIC_EnableIRQ(PORT1_IRQn);
	
	//Global Enable Interrupt
	__ASM("CPSIE I");
	
}

int main(void)
{
	config_UART();
	config_GPIO();


    
    // Enter LPM0
    __sleep();
    __no_operation();                       // For debugger
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
			
			if(EUSCI_A0->RXBUF == 0x2D)
			{
				changeState(-1);
			}else if(EUSCI_A0->RXBUF == 0x2B)
			{
				changeState(1);
			}     
    }
}

// Button interrupt service routine
void PORT1_IRQHandler(void)
{
	if(P1IFG & (1<<1)) // Checking Button P1.1
	{
		P1IFG &=~ (uint8_t) (1<<1);
		changeState(-1);	// Going back to the prev state
	}
	
	if(P1IFG & (1<<4)) // Checking Button P1.4
	{
		P1IFG &=~ (uint8_t) (1<<4);
		changeState(1);	// Going back to the next state
	}
}

// Changes states
void changeState(int switchState)
{
	
	if(curr_state >= (uint8_t)0x30 && switchState == -1)
	{
		if(curr_state == (uint8_t)0x30)
		{
			curr_state = (uint8_t)0x33;
		}else
		{
			curr_state--;
		}
	}
	if(curr_state <= (uint8_t)0x33 && switchState == 1)
	{
		if(curr_state == (uint8_t)0x33)
		{
			curr_state = (uint8_t)0x30;
		}else
		{
			curr_state++;
		}
	}
	takeAction();
	
}

// Takes change upon the current state
void takeAction(void)
{
	if(curr_state == (uint8_t)0x30)
	{
		P1OUT &=~ (uint8_t)(1<<0);
		P2OUT &=~ (uint8_t)(1<<0);
	}else if(curr_state == (uint8_t)0x31)
	{
		P1OUT &=~ (uint8_t)(1<<0);
		P2OUT |= (uint8_t)(1<<0);
	}else if(curr_state == (uint8_t)0x32)
	{
		P1OUT |= (uint8_t)(1<<0);
		P2OUT &=~ (uint8_t)(1<<0);
	}else if(curr_state == (uint8_t)0x33)
	{
		P1OUT |= (uint8_t)(1<<0);
		P2OUT |= (uint8_t)(1<<0);
	}
	
	EUSCI_A0->TXBUF = 0x0A; // New Line
	EUSCI_A0->TXBUF = curr_state;
}
