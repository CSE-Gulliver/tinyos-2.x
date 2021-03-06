/**
 * "Copyright (c) 2009 The Regents of the University of California.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement
 * is hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
 * OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 */

/**
 * This is a low-level clock component controlling the different clock
 * systems.
 *
 * @author Thomas Schmid
 */

#include "sam3upmchardware.h"
#include "sam3usupchardware.h"

#define CLOCK_TIMEOUT 0xFFFFFFFF

module HplSam3uClockP
{
    provides
    {
        interface HplSam3uClock;
    }
}

implementation
{

    async command error_t HplSam3uClock.slckExternalOsc()
    {
        uint32_t timeout = 0;

        if(SUPC->sr.bits.oscsel == 0)
        {
            supc_cr_t cr;
            cr.flat = 0; // assure it is all zero!
            cr.bits.xtalsel = 1;
            cr.bits.key = SUPC_CR_KEY;

            SUPC->cr = cr;

            timeout = 0;
            while (!(SUPC->sr.bits.oscsel) && (timeout++ < CLOCK_TIMEOUT));
        }

        return SUCCESS;
    }

    async command error_t HplSam3uClock.slckRCOsc()
    {
        uint32_t timeout = 0;

        if(SUPC->sr.bits.oscsel == 1)
        {
            supc_cr_t cr;
            cr.flat = 0; // assure it is all zero!
            cr.bits.xtalsel = 0;
            cr.bits.key = SUPC_CR_KEY;

            SUPC->cr = cr;

            timeout = 0;
            while (!(SUPC->sr.bits.oscsel) && (timeout++ < CLOCK_TIMEOUT));
        }

       return SUCCESS;
    }

    async command error_t HplSam3uClock.mckInit48()
    {
        pmc_mor_t mor;
        pmc_mckr_t mckr;
        pmc_pllar_t pllar;
        uint32_t timeout = 0;

       // Check if MCK source is RC or XT
        if(PMC->mor.bits.moscsel == 0)
        {
            // it is RC, turn on XT
            mor.flat = 0; // make sure it is zreoed out
            mor.bits.key = PMC_MOR_KEY;
            mor.bits.moscxtst = 0x3F; // main oscillator startup time
            mor.bits.moscrcen = 1;    // enable the on-chip rc oscillator
            mor.bits.moscxten = 1;    // main crystal oscillator enable
            PMC->mor = mor;

            timeout = 0;
            while (!(PMC->sr.bits.moscxts) && (timeout++ < CLOCK_TIMEOUT));
        }

        // Switch to XT
        mor.flat = 0; // make sure it is zeroed
        mor.bits.key = PMC_MOR_KEY;
        mor.bits.moscxtst = 0x3F;
        mor.bits.moscrcen = 1;
        mor.bits.moscxten = 1;
        mor.bits.moscsel = 1;
        PMC->mor = mor;
        timeout = 0;
        while (!(PMC->sr.bits.moscsels) && (timeout++ < CLOCK_TIMEOUT));
        mckr = PMC->mckr;
        mckr.bits.css = PMC_MCKR_CSS_MAIN_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        // Initialize PLLA
        pllar.flat = 0; // make sure it is zeroed out
        pllar.bits.bit29 = 1; // we always have to do this!
        pllar.bits.mula = 0x3; // multiplication is MULA+1 => 12x4 = 48MHz
        pllar.bits.pllacount = 0x3F;
        pllar.bits.diva = 0x1; // divider is bypassed
        pllar.bits.stmode = PMC_PLLAR_STMODE_FAST_STARTUP;
        PMC->pllar = pllar;
        timeout = 0;
        while (!(PMC->sr.bits.locka) && (timeout++ < CLOCK_TIMEOUT));

        // Switch to fast clock
        mckr.bits.css = PMC_MCKR_CSS_MAIN_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        mckr.bits.pres = PMC_MCKR_PRES_DIV_1;
        mckr.bits.css = PMC_MCKR_CSS_PLLA_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        signal HplSam3uClock.mainClockChanged();

        return SUCCESS;
    }

    async command error_t HplSam3uClock.mckInit84()
    {
        pmc_mor_t mor;
        pmc_mckr_t mckr;
        pmc_pllar_t pllar;
        uint32_t timeout = 0;

       // Check if MCK source is RC or XT
        if(PMC->mor.bits.moscsel == 0)
        {
            // it is RC, turn on XT
            mor.flat = 0; // make sure it is zreoed out
            mor.bits.key = PMC_MOR_KEY;
            mor.bits.moscxtst = 0x3F; // main oscillator startup time
            mor.bits.moscrcen = 1;    // enable the on-chip rc oscillator
            mor.bits.moscxten = 1;    // main crystal oscillator enable
            PMC->mor = mor;

            timeout = 0;
            while (!(PMC->sr.bits.moscxts) && (timeout++ < CLOCK_TIMEOUT));
        }

        // Switch to XT
        mor.flat = 0; // make sure it is zeroed
        mor.bits.key = PMC_MOR_KEY;
        mor.bits.moscxtst = 0x3F;
        mor.bits.moscrcen = 1;
        mor.bits.moscxten = 1;
        mor.bits.moscsel = 1;
        PMC->mor = mor;
        timeout = 0;
        while (!(PMC->sr.bits.moscsels) && (timeout++ < CLOCK_TIMEOUT));
        mckr = PMC->mckr;
        mckr.bits.css = PMC_MCKR_CSS_MAIN_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        // Initialize PLLA
        pllar.flat = 0; // make sure it is zeroed out
        pllar.bits.bit29 = 1; // we always have to do this!
        pllar.bits.mula = 0x6; // multiplication is MULA+1 => 12x7 = 84MHz
        pllar.bits.pllacount = 0x3F;
        pllar.bits.diva = 0x1; // divider is bypassed
        pllar.bits.stmode = PMC_PLLAR_STMODE_FAST_STARTUP;
        PMC->pllar = pllar;
        timeout = 0;
        while (!(PMC->sr.bits.locka) && (timeout++ < CLOCK_TIMEOUT));

        // Switch to fast clock
        mckr.bits.css = PMC_MCKR_CSS_MAIN_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        mckr.bits.pres = PMC_MCKR_PRES_DIV_1;
        mckr.bits.css = PMC_MCKR_CSS_PLLA_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        signal HplSam3uClock.mainClockChanged();
        return SUCCESS;
    }

    async command error_t HplSam3uClock.mckInit12RC()
    {
        pmc_mor_t mor;
        pmc_mckr_t mckr;
        pmc_pllar_t pllar;
        uint32_t timeout = 0;

        // Check if MCK source is RC or XT
        if(PMC->mor.bits.moscsel == 1)
        {
            // it is XT, turn on RC
            mor.flat = 0; // make sure it is zreoed out
            mor.bits.key = PMC_MOR_KEY;
            mor.bits.moscrcf = 3;     // select 12 MHz RC 
            mor.bits.moscrcen = 1;    // enable the on-chip rc oscillator
            mor.bits.moscxten = 1;    // main crystal oscillator enable
            PMC->mor = mor;

            timeout = 0;
            while (!(PMC->sr.bits.moscrcs) && (timeout++ < CLOCK_TIMEOUT));
        }

        // Switch to RC
        mor.flat = 0; // make sure it is zeroed
        mor.bits.key = PMC_MOR_KEY;
        mor.bits.moscrcf = 3;     // select 12 MHz RC 
        mor.bits.moscrcen = 1;
        mor.bits.moscxten = 1;
        mor.bits.moscsel = 0;
        PMC->mor = mor;
        timeout = 0;
        while (!(PMC->sr.bits.moscsels) && (timeout++ < CLOCK_TIMEOUT));
        mckr = PMC->mckr;
        mckr.bits.pres = PMC_MCKR_PRES_DIV_1;
        mckr.bits.css = PMC_MCKR_CSS_MAIN_CLOCK;
        PMC->mckr = mckr;
        timeout = 0;
        while (!(PMC->sr.bits.mckrdy) && (timeout++ < CLOCK_TIMEOUT));

        // turn off external clock
        mor.bits.moscxten = 0;
        PMC->mor = mor;
        
        // Turn off PLL
        pllar.flat = 0; // make sure it is zeroed out
        pllar.bits.bit29 = 1; // this always has to be written as 1!
        PMC->pllar = pllar;

        signal HplSam3uClock.mainClockChanged();
        return SUCCESS;
    }

    async command uint32_t HplSam3uClock.getMainClockSpeed()
    {
        uint32_t speed = 0;
        switch(PMC->mckr.bits.css)
        {
            case PMC_MCKR_CSS_SLOW_CLOCK:
                speed = 32;
                break;

            case PMC_MCKR_CSS_MAIN_CLOCK:
                speed = PMC->mcfr.bits.mainf*2048/1000; // 0.48828 corresponds to 16 clock ticks of a 32kHz crystal.
                break;

            case PMC_MCKR_CSS_PLLA_CLOCK:
                if(PMC->pllar.bits.diva != 0)
                {
                    // note, the PLL multiplier is (mula + 1)
                    speed = PMC->mcfr.bits.mainf*2048/1000 * (PMC->pllar.bits.mula + 1) / PMC->pllar.bits.diva;
                    //speed = PMC->mcfr.bits.mainf*1000/488 * (PMC->pllar.bits.mula + 1) / PMC->pllar.bits.diva;
                } else
                    speed = 0;
                break;
            default:
                speed = 0;
        }

        return speed;
    }

    default async event void HplSam3uClock.mainClockChanged(){}
}
