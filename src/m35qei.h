/* 
 * File:   m35qei.h
 * Author: root
 *
 * Created on July 8, 2018, 1:58 PM
 */

#ifndef M35QEI_H
#define	M35QEI_H

#ifdef	__cplusplus
extern "C" {
#endif

	/*
	 * QEI #1 pin connections
	 * 
	 * QEA1 RPA0
	 * QEB1 RPA14
	 * INDX1 RPB6
	 * HOME1 RPB9
	 */

	/*
	 * encoder device data
	 */
	typedef struct {
		int32_t pos, vel;
		uint32_t update;
	} QEI_DATA;

	int32_t m35_init_qei(void);

#ifdef	__cplusplus
}
#endif

#endif	/* M35QEI_H */

