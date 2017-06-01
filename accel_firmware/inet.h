/*
 * inet.h
 *
 *  Created on: 29. jun. 2016
 *      Author: vojis
 */

#ifndef INET_H_
#define INET_H_

#define LITTLE_ENDIAN

/* host-to-network short*/
uint16_t htons(uint16_t v);

/* host-to-network long*/
uint32_t htonl(uint32_t v);
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )

/* network-to-host short*/
uint16_t ntohs(uint16_t v);

/* network-to-host long*/
uint32_t ntohl(uint32_t v);

#endif /* INET_H_ */
