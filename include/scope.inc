/*
 * This file is part of the sigrok-firmware-fx2lafw project.
 *
 * Copyright (C) 2009 Ubixum, Inc.
 * Copyright (C) 2015 Jochen Hoenicke
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include <i2c.h>

#define OE_CTL (((1 << CTL_BIT) << 4) | (1 << CTL_BIT)) /* OEx = CTLx = 1 */

static BOOL set_voltage(BYTE channel, BYTE val);

struct samplerate_info {
        BYTE rate;
        BYTE wait0;
        BYTE wait1;
        BYTE opc0;
        BYTE opc1;
        BYTE out0;
        BYTE ifcfg;
};

/* Change to support as many interfaces as you need. */
static BYTE altiface = 0;
static BOOL use_external_clock = FALSE;
static BYTE last_rate = 0;

static volatile WORD ledcounter = 0;

static volatile __bit dosud = FALSE;
static volatile __bit dosuspend = FALSE;

extern __code BYTE highspd_dscr;
extern __code BYTE fullspd_dscr;

void resume_isr(void) __interrupt(RESUME_ISR)
{
	CLEAR_RESUME();
}

void sudav_isr(void) __interrupt(SUDAV_ISR)
{
	dosud = TRUE;
	CLEAR_SUDAV();
}

void usbreset_isr(void) __interrupt(USBRESET_ISR)
{
	handle_hispeed(FALSE);
	CLEAR_USBRESET();
}

void hispeed_isr(void) __interrupt(HISPEED_ISR)
{
	handle_hispeed(TRUE);
	CLEAR_HISPEED();
}

void suspend_isr(void) __interrupt(SUSPEND_ISR)
{
	dosuspend = TRUE;
	CLEAR_SUSPEND();
}

void timer2_isr(void) __interrupt(TF2_ISR)
{
	/* Toggle the probe calibration pin, only accurate up to ca. 8MHz. */
	TOGGLE_CALIBRATION_PIN();

	if (ledcounter && (--ledcounter == 0))
		LED_CLEAR();

	TF2 = 0;
}

/**
 * Each LSB in the nibble of the byte controls the coupling per channel.
 *
 * Setting PE3 disables AC coupling capacitor on CH0.
 * Setting PE0 disables AC coupling capacitor on CH1.
 */
static void set_coupling(BYTE coupling_cfg)
{
	if (coupling_cfg & 0x01)
		IOE |= 0x08;
	else
		IOE &= ~0x08;

	if (coupling_cfg & 0x10)
		IOE |= 0x01;
	else
		IOE &= ~0x01;
}

static BOOL set_numchannels(BYTE numchannels)
{
	if (numchannels == 1 || numchannels == 2) {
		BYTE fifocfg = 7 + numchannels;
		EP2FIFOCFG = fifocfg;
		EP6FIFOCFG = fifocfg;
		return TRUE;
	}

	return FALSE;
}

static void clear_fifo(void)
{
	GPIFABORT = 0xff;
	SYNCDELAY3;
	FIFORESET = 0x80;
	SYNCDELAY3;
	FIFORESET = 0x82;
	SYNCDELAY3;
	FIFORESET = 0x86;
	SYNCDELAY3;
	FIFORESET = 0;
}

static void stop_sampling(void)
{
	GPIFABORT = 0xff;
	SYNCDELAY3;
	INPKTEND = (altiface == 0) ? 6 : 2;
}

static void start_sampling(void)
{
	int i;

	SET_ANALOG_MODE();

	clear_fifo();

	for (i = 0; i < 1000; i++);

	while (!(GPIFTRIG & 0x80))
		;

	SYNCDELAY3;
	GPIFTCB1 = 0x28;
	SYNCDELAY3;
	GPIFTCB0 = 0;
	GPIFTRIG = (altiface == 0) ? 6 : 4;

	/* Set green LED, don't clear LED afterwards (ledcounter = 0). */
	LED_GREEN();
	ledcounter = 0;
}

static void select_interface(BYTE alt)
{
	const BYTE *pPacketSize = \
		((USBCS & bmHSM) ? &highspd_dscr : &fullspd_dscr)
		+ (9 + (16 * alt) + 9 + 4);

	altiface = alt;

	if (alt == 0) {
		/* Bulk on EP6. */
		EP2CFG = 0x00;
		EP6CFG = 0xe0;
		EP6GPIFFLGSEL = 1;
		EP6AUTOINLENL = pPacketSize[0];
		EP6AUTOINLENH = pPacketSize[1];
	} else {
		/* Iso on EP2. */
		EP2CFG = 0xd8;
		EP6CFG = 0x00;
		EP2GPIFFLGSEL = 1;
		EP2AUTOINLENL = pPacketSize[0];
		EP2AUTOINLENH = pPacketSize[1] & 0x7;
		EP2ISOINPKTS = (pPacketSize[1] >> 3) + 1;
	}
}

static BOOL set_samplerate(BYTE rate)
{
	BYTE i = 0;

	if (!use_external_clock) {
		while (samplerates[i].rate != rate) {
			i++;
			if (i == sizeof(samplerates) / sizeof(samplerates[0]))
				return FALSE;
		}
		IFCONFIG = samplerates[i].ifcfg;
	} else {
		/* use highest sample rate using single instruction loop */
		i = 0;
		/* use external clock, IFLCK not driven as output */
		IFCONFIG = 0x4a;
	}


	AUTOPTRSETUP = 7;
	AUTOPTRH2 = 0xE4; /* 0xE400: GPIF waveform descriptor 0. */
	AUTOPTRL2 = 0x00;

	/*
	 * The program for low-speed, e.g. 1 MHz, is:
	 * wait 24, CTLx=0, FIFO
	 * wait 23, CTLx=1
	 * jump 0, CTLx=1
	 *
	 * The program for 24 MHz is:
	 * wait 1, CTLx=0, FIFO
	 * jump 0, CTLx=1
	 *
	 * The program for 30/48 MHz is:
	 * jump 0, CTLx=Z, FIFO, LOOP
	 *
	 * (CTLx is device-dependent, could be e.g. CTL0 or CTL2.)
	 */

	/* LENGTH / BRANCH 0-7 */
	EXTAUTODAT2 = samplerates[i].wait0;
	EXTAUTODAT2 = samplerates[i].wait1;
	EXTAUTODAT2 = 1;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;

	/* OPCODE 0-7 */
	EXTAUTODAT2 = samplerates[i].opc0;
	EXTAUTODAT2 = samplerates[i].opc1;
	EXTAUTODAT2 = 1; /* DATA=0 DP=1 */
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;

	/* OUTPUT 0-7 */
	EXTAUTODAT2 = samplerates[i].out0;
	EXTAUTODAT2 = OE_CTL;
	EXTAUTODAT2 = OE_CTL;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;

	/* LOGIC FUNCTION 0-7 */
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;
	EXTAUTODAT2 = 0;

	for (i = 0; i < 96; i++)
		EXTAUTODAT2 = 0;

	return TRUE;
}

static BOOL set_calibration_pulse(BYTE fs)
{
	switch (fs) {
	case 0:		// 100Hz
		RCAP2L = -10000 & 0xff;
		RCAP2H = (-10000 & 0xff00) >> 8;
		return TRUE;
	case 1:		// 1kHz
		RCAP2L = -1000 & 0xff;
		RCAP2H = (-1000 & 0xff00) >> 8;
		return TRUE;
	case 10:	// 1kHz
		RCAP2L = (BYTE)(-100 & 0xff);
		RCAP2H = 0xff;
		return TRUE;
	case 50:	// 50kHz
		RCAP2L = (BYTE)(-20 & 0xff);
		RCAP2H = 0xff;
		return TRUE;
	default:
		return FALSE;
	}
}

/* Set *alt_ifc to the current alt interface for ifc. */
BOOL handle_get_interface(BYTE ifc, BYTE *alt_ifc)
{
	(void)ifc;

	*alt_ifc = altiface;

	return TRUE;
}

/*
 * Return TRUE if you set the interface requested.
 *
 * Note: This function should reconfigure and reset the endpoints
 * according to the interface descriptors you provided.
 */
BOOL handle_set_interface(BYTE ifc,BYTE alt_ifc)
{
	if (ifc == 0)
		select_interface(alt_ifc);

	return TRUE;
}

BYTE handle_get_configuration(void)
{
	/* We only support configuration 0. */
	return 0;
}

BOOL handle_set_configuration(BYTE cfg)
{
	/* We only support configuration 0. */
	(void)cfg;

	return TRUE;
}

static void handle_i2c_read(void)
{
	BYTE *p = (BYTE *)EP0BUF;

	/* I2C addr */
	WORD wValue = SETUP_VALUE();
	WORD wLength = SETUP_LENGTH();

	BOOL ret = i2c_read(wValue & 0xff, wLength, p);
	p[0] = ret ? 0 : 1;

	/* Send the message. */
	EP0BCH = 0;
	SYNCDELAY4;
	EP0BCL = ret ? (wLength & 0xff) : 0;
	SYNCDELAY4;
}

static BOOL handle_i2c_write(void)
{
	/* I2C addr */
	WORD wValue = SETUP_VALUE();
	WORD wLength = SETUP_LENGTH();
	BYTE *p = (BYTE *)EP0BUF;

	return i2c_write(wValue & 0xff, 0, NULL, wLength, p);
}

BOOL handle_vendorcommand(BYTE cmd)
{
	if (cmd < 0xe8) {
		stop_sampling();

		/* Set red LED, clear after timeout. */
		LED_RED();
		ledcounter = 1000;
	}

	/* Clear EP0BCH/L for each valid command. */
	if (cmd >= 0xe0 && cmd <= 0xe8) {
		EP0BCH = 0;
		EP0BCL = 0;
		while (EP0CS & bmEPBUSY);
	}

	switch (cmd) {
	case 0xe0:
	case 0xe1:
		set_voltage(cmd - 0xe0, EP0BUF[0]);
		return TRUE;
	case 0xe2:
		set_samplerate(EP0BUF[0]);
		return TRUE;
	case 0xe3:
		if (EP0BUF[0] == 1)
			start_sampling();
		return TRUE;
	case 0xe4:
		set_numchannels(EP0BUF[0]);
		return TRUE;
	case 0xe5:
		SET_COUPLING(EP0BUF[0]);
		return TRUE;
	case 0xe6:
		SET_CALIBRATION_PULSE(EP0BUF[0]);
		return TRUE;
	case 0xe7:
		use_external_clock = EP0BUF[0] ? TRUE : FALSE;
		if (use_external_clock)
			set_samplerate(0);
		return TRUE;
	case 0xe8:
		handle_i2c_write();
		return TRUE;
	case 0xe9:
		handle_i2c_read();
		return TRUE;
	}

	return FALSE; /* Not handled by handlers. */
}

static void init(void)
{
	EP4CFG = 0;
	EP8CFG = 0;

	SET_ANALOG_MODE();

	/* In idle mode tristate all outputs. */
	GPIFIDLECTL = 0x00; /* Don't enable CTL0-5 outputs. */
	GPIFCTLCFG = 0x80; /* TRICTL=1. CTL0-2: CMOS outputs, tri-statable. */
	GPIFWFSELECT = 0x00;
	GPIFREADYSTAT = 0x00;

	stop_sampling();

	set_voltage(0, 1);
	set_voltage(1, 1);
	set_samplerate(1);
	set_numchannels(2);
	select_interface(0);
}

extern __code BYTE serial_num;
extern __code BYTE dev_strings_end;

/* use unique ID of FX2 as serial number, see KBA89285 */
void init_serial_num(void)
{
	BYTE i, uid_pos = 0;
	BYTE __xdata *snbuf = (BYTE __xdata *)&serial_num;
	BYTE __xdata *uniqid = (BYTE __xdata *)&UNIQID0;
	BYTE len = &dev_strings_end - &serial_num;

	const char h2a[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
			       '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

	/* search for ':' delimiter */
	for (i = 0; i < (len - 24); i += 2) {
		if (snbuf[i] == ':') {
			uid_pos = i + 4;
			break;
		}
	}

	if (!uid_pos)
		return;

	/* fill in UID */
	for (i = 0; i < 6; i++) {
		snbuf[uid_pos + 2] = h2a[uniqid[5 - i] >> 4];
		snbuf[uid_pos] = h2a[uniqid[5 - i] & 0x0f];
		uid_pos += 4;
	}
}

static void main(void)
{
	/* Save energy. */
	SETCPUFREQ(CLK_12M);

	init_serial_num();
	init();

	/* Set up interrupts. */
	USE_USB_INTS();

	ENABLE_SUDAV();
	ENABLE_USBRESET();
	ENABLE_HISPEED(); 
	ENABLE_SUSPEND();
	ENABLE_RESUME();

	/* Global (8051) interrupt enable. */
	EA = 1;

	/* Init timer2. */
	RCAP2L = -TIMER2_VAL & 0xff;
	RCAP2H = (-TIMER2_VAL & 0xff00) >> 8;
	T2CON = 0;
	ET2 = 1;
	TR2 = 1;

	RENUMERATE_UNCOND();

	PORTECFG = 0;
	PORTCCFG = 0;
	PORTACFG = 0;
	OEE = 0xff;
	OEC = 0xff;
	OEA = 0xff;

	while (TRUE) {
		if (dosud) {
			dosud = FALSE;
			handle_setupdata();
		}

		if (dosuspend) {
			dosuspend = FALSE;
			do {
				/* Make sure ext wakeups are cleared. */
				WAKEUPCS |= bmWU | bmWU2;
				SUSPEND = 1;
				PCON |= 1;
				__asm
				nop
				nop
				nop
				nop
				nop
				nop
				nop
				__endasm;
			} while (!remote_wakeup_allowed && REMOTE_WAKEUP());

			/* Resume (TRM 6.4). */
			if (REMOTE_WAKEUP()) {
				delay(5);
				USBCS |= bmSIGRESUME;
				delay(15);
				USBCS &= ~bmSIGRESUME;
			}
		}
	}
}
