/*
 * Copyright (c) 2009 Stanford University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL STANFORD
 * UNIVERSITY OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Pin abstraction on the SAM3U.
 *
 * @author wanja@cs.fau.de
 */

generic module HplSam3uGeneralIOPinP(uint32_t pio_addr, uint8_t bit)
{
	provides
	{
		interface GeneralIO as IO;
		interface HplSam3uGeneralIOPin as HplPin;
	}
}
implementation
{
	async command bool IO.get()
	{
		if ((call IO.isInput()) == 1) {
			/* Read bit from Pin Data Status Register */
			uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x03c));
			uint32_t currentpin = (currentport & (1 << bit)) >> bit;
			return ((currentpin & 1) == 1);
		} else {
			/* Read bit from Output Data Status Register */
			uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x038));
			uint32_t currentpin = (currentport & (1 << bit)) >> bit;
			return ((currentpin & 1) == 1);
		}
	}

	async command void IO.set()
	{
		/* Set bit in Set Output Data Register */
		*((volatile uint32_t *) (pio_addr + 0x030)) = (1 << bit);
	}

	async command void IO.clr()
	{
		/* Set bit in Clear Output Data Register */
		*((volatile uint32_t *) (pio_addr + 0x034)) = (1 << bit);
	}

	async command void IO.toggle()
	{
		if ((call IO.get()) == 1) {
			call IO.clr();
		} else {
			call IO.set();
		}
	}

	async command void IO.makeInput()
	{
		/* Set bit in Output Disable Register */
		*((volatile uint32_t *) (pio_addr + 0x014)) = (1 << bit);
    }

	async command void IO.makeOutput()
	{
		/* Set bit in Output Enable Register */
		*((volatile uint32_t *) (pio_addr + 0x010)) = (1 << bit);
    }

	async command bool IO.isOutput()
	{
		/* Read bit from Output Status Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x018));
		uint32_t currentpin = (currentport & (1 << bit)) >> bit;
		return ((currentpin & 1) == 1);
	}

	async command bool IO.isInput()
	{
		return (! (call IO.isOutput()));
	}

	async command void HplPin.enablePioControl()
	{
		/* Set bit in PIO Enable Register */
		*((volatile uint32_t *) (pio_addr + 0x000)) = (1 << bit);
	}

	async command void HplPin.disablePioControl()
	{
		/* Set bit in PIO Disable Register */
		*((volatile uint32_t *) (pio_addr + 0x004)) = (1 << bit);
	}

	async command bool HplPin.isEnabledPioControl()
	{
		/* Read bit from PIO Status Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x008));
		uint32_t currentpin = (currentport & (1 << bit)) >> bit;
		return ((currentpin & 1) == 1);
	}

	async command void HplPin.enableMultiDrive()
	{
		/* Set bit in Multi-Driver Enable Register */
		*((volatile uint32_t *) (pio_addr + 0x050)) = (1 << bit);
	}

	async command void HplPin.disableMultiDrive()
	{
		/* Set bit in Multi-Driver Disable Register */
		*((volatile uint32_t *) (pio_addr + 0x054)) = (1 << bit);
	}

	async command bool HplPin.isEnabledMultiDrive()
	{
		/* Read bit from Multi-Driver Status Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x058));
		uint32_t currentpin = (currentport & (1 << bit)) >> bit;
		return ((currentpin & 1) == 1);
	}

	async command void HplPin.enablePullUpResistor()
	{
		/* Set bit in Pull-Up Enable Register */
		*((volatile uint32_t *) (pio_addr + 0x064)) = (1 << bit);
	}

	async command void HplPin.disablePullUpResistor()
	{
		/* Set bit in Pull-Up Disable Register */
		*((volatile uint32_t *) (pio_addr + 0x060)) = (1 << bit);
	}

	async command bool HplPin.isEnabledPullUpResistor()
	{
		/* Read bit from Pull-Up Status Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x068));
		uint32_t currentpin = (currentport & (1 << bit)) >> bit;
		return ((currentpin & 1) == 0);
	}

	async command void HplPin.selectPeripheralA()
	{
		/* Read in Peripheral AB Select Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x070));
		/* Clear bit */
		currentport &= ~ (1 << bit);
		/* Write back to register */
		*((volatile uint32_t *) (pio_addr + 0x070)) = currentport;
	}

	async command void HplPin.selectPeripheralB()
	{
		/* Read in Peripheral AB Select Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x070));
		/* Set bit */
		currentport |= (1 << bit);
		/* Write back to register */
		*((volatile uint32_t *) (pio_addr + 0x070)) = currentport;
	}

	async command bool HplPin.isSelectedPeripheralA()
	{
		/* Read bit from Peripheral AB Select Register */
		uint32_t currentport = *((volatile uint32_t *) (pio_addr + 0x068));
		uint32_t currentpin = (currentport & (1 << bit)) >> bit;
		return ((currentpin & 1) == 0);
	}
}