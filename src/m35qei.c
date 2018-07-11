
#include <proc/p32mk1024mcf100.h>
#include "m35qei.h"
#include <xc.h>
#include <stdio.h>

int32_t m35_init_qei(void)
{
	/* port pin numbers from port_p32mk1024mcf100.h */
	QEA1R = 0x00;
	QEB1R = 0x0D;
	INDX1R = 0x00;
	HOME1R = 0x04;
	QEI1CON = 0;
	QEI1IOC = 0;
	QEI1IOCbits.QEAPOL = 1;
	QEI1IOCbits.QEBPOL = 1;
	QEI1IOCbits.IDXPOL = 1;
	QEI1IOCbits.SWPAB = 1;
	QEI1CONbits.PIMOD = 0;
	QEI1CONbits.QEIEN = 1;
	POS1CNT = 0;
	return 0;
}


