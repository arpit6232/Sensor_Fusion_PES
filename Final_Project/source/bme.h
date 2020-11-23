/*
 * bme.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */


#ifndef BME_H_
#define BME_H_

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#define _BME_INLINE	__attribute__((always_inline)) static inline
#elif defined(__CWCC__)
#define _BME_INLINE	static __inline
#endif

#define BME_AND_MASK	(1<<26)
#define BME_OR_MASK		(1<<27)
#define BME_XOR_MASK	(3<<26)
#define BME_BFI_MASK(BIT,WIDTH)		(1<<28) | (BIT<<23) | ((WIDTH-1)<<19)
#define BME_LAC1_MASK(BIT)			(1<<27) | (BIT<<21)
#define BME_LAS1_MASK(BIT)			(3<<26) | (BIT<<21)
#define BME_UBFX_MASK(BIT,WIDTH)	(1<<28) | (BIT<<23) | ((WIDTH-1)<<19)


/* Decorated Store: Logical AND */
static void BME_AND_B(volatile uint8_t *addr, register uint8_t wdata);
static void BME_AND_H(volatile uint16_t *addr, register uint16_t wdata);
static void BME_AND_W(volatile uint32_t *addr, register uint32_t wdata);

/* Decorated Store: Logical OR */
static void BME_OR_B(volatile uint8_t *addr, register uint8_t wdata);
static void BME_OR_H(volatile uint16_t *addr, register uint16_t wdata);
static void BME_OR_W(volatile uint32_t *addr, register uint32_t wdata);

/* Decorated Store: Logical XOR */
static void BME_XOR_B(volatile uint8_t *addr, register uint8_t wdata);
static void BME_XOR_H(volatile uint16_t *addr, register uint16_t wdata);
static void BME_XOR_W(volatile uint32_t *addr, register uint32_t wdata);

/* Decorated Store: Bit Field Insert */
static void BME_BFI_B(volatile uint8_t *addr, register uint8_t wdata, uint8_t bit, uint8_t width);
static void BME_BFI_H(volatile uint16_t *addr, register uint16_t wdata, uint8_t bit, uint8_t width);
static void BME_BFI_W(volatile uint32_t *addr, register uint32_t wdata, uint8_t bit, uint8_t width);

/* Decorated Load: Load and Clear 1-bit */
static uint8_t BME_LAC1_B(volatile uint8_t *addr, uint8_t bit);
static uint16_t BME_LAC1_H(volatile uint16_t *addr, uint8_t bit);
static uint32_t BME_LAC1_W(volatile uint32_t *addr, uint8_t bit);

/* Decorated Load: Load and Set 1-bit */
static uint8_t BME_LAS1_B(volatile uint8_t *addr, uint8_t bit);
static uint16_t BME_LAS1_H(volatile uint16_t *addr, uint8_t bit);
static uint32_t BME_LAS1_W(volatile uint32_t *addr, uint8_t bit);

/* Decorated Load: Unsigned Bit Field Extract */
static uint8_t BME_UBFX_B(volatile uint8_t *addr, uint8_t bit, uint8_t width);
static uint16_t BME_UBFX_H(volatile uint16_t *addr, uint8_t bit, uint8_t width);
static uint32_t BME_UBFX_W(volatile uint32_t *addr, uint8_t bit, uint8_t width);


_BME_INLINE void BME_AND_W (volatile uint32_t *addr, register uint32_t wdata)
{
	*(volatile uint32_t*)((uint32_t)addr | BME_AND_MASK) = wdata;
}

_BME_INLINE void BME_AND_H (volatile uint16_t *addr, register uint16_t wdata)
{
	*(volatile uint16_t*)((uint32_t)addr | BME_AND_MASK) = wdata;
}

_BME_INLINE void BME_AND_B (volatile uint8_t *addr, register uint8_t wdata)
{
	*(volatile uint8_t*)((uint32_t)addr | BME_AND_MASK) = wdata;
}

_BME_INLINE void BME_OR_W (volatile uint32_t *addr, register uint32_t wdata)
{
	*(volatile uint32_t*)((uint32_t)addr | BME_OR_MASK) = wdata;
}

_BME_INLINE void BME_OR_H (volatile uint16_t *addr, register uint16_t wdata)
{
	*(volatile uint16_t*)((uint32_t)addr | BME_OR_MASK) = wdata;
}

_BME_INLINE void BME_OR_B (volatile uint8_t *addr, register uint8_t wdata)
{
	*(volatile uint8_t*)((uint32_t)addr | BME_OR_MASK) = wdata;
}

_BME_INLINE void BME_XOR_W (volatile uint32_t *addr, register uint32_t wdata)
{
	*(volatile uint32_t*)((uint32_t)addr | BME_XOR_MASK) = wdata;
}

_BME_INLINE void BME_XOR_H (volatile uint16_t *addr, register uint16_t wdata)
{
	*(volatile uint16_t*)((uint32_t)addr | BME_XOR_MASK) = wdata;
}

_BME_INLINE void BME_XOR_B (volatile uint8_t *addr, register uint8_t wdata)
{
	*(volatile uint8_t*)((uint32_t)addr | BME_XOR_MASK) = wdata;
}

/* The width field is encoded as (width-1) */
_BME_INLINE void BME_BFI_W (volatile uint32_t *addr, register uint32_t wdata, uint8_t bit, uint8_t width)
{
	*(volatile uint32_t*)((uint32_t)addr | BME_BFI_MASK(bit,width)) = wdata;
}

_BME_INLINE void BME_BFI_H (volatile uint16_t *addr, register uint16_t wdata, uint8_t bit, uint8_t width)
{
	*(volatile uint16_t*)((uint32_t)addr | BME_BFI_MASK(bit,width)) = wdata;
}

_BME_INLINE void BME_BFI_B (volatile uint8_t *addr, register uint8_t wdata, uint8_t bit, uint8_t width)
{
	*(volatile uint8_t*)((uint32_t)addr | BME_BFI_MASK(bit,width)) = wdata;
}

_BME_INLINE uint32_t BME_LAC1_W (volatile uint32_t *addr, uint8_t bit)
{
	return *(volatile uint32_t*)((uint32_t)addr | BME_LAC1_MASK(bit));
}

_BME_INLINE uint16_t BME_LAC1_H (volatile uint16_t *addr, uint8_t bit)
{
	return *(volatile uint16_t*)((uint32_t)addr | BME_LAC1_MASK(bit));
}

_BME_INLINE uint8_t BME_LAC1_B (volatile uint8_t *addr, uint8_t bit)
{
	return *(volatile uint8_t*)((uint32_t)addr | BME_LAC1_MASK(bit));
}

_BME_INLINE uint32_t BME_LAS1_W (volatile uint32_t *addr, uint8_t bit)
{
	return *(volatile uint32_t*)((uint32_t)addr | BME_LAS1_MASK(bit));
}

_BME_INLINE uint16_t BME_LAS1_H (volatile uint16_t *addr, uint8_t bit)
{
	return *(volatile uint16_t*)((uint32_t)addr | BME_LAS1_MASK(bit));
}

_BME_INLINE uint8_t BME_LAS1_B (volatile uint8_t *addr, uint8_t bit)
{
	return *(volatile uint8_t*)((uint32_t)addr | BME_LAS1_MASK(bit));
}

/* The width field is encoded as (width-1) */
_BME_INLINE uint32_t BME_UBFX_W (volatile uint32_t *addr, uint8_t bit, uint8_t width)
{
	return *(volatile uint32_t*)((uint32_t)addr | BME_UBFX_MASK(bit,width));
}

_BME_INLINE uint16_t BME_UBFX_H (volatile uint16_t *addr, uint8_t bit, uint8_t width)
{
	return *(volatile uint16_t*)((uint32_t)addr | BME_UBFX_MASK(bit,width));
}

_BME_INLINE uint8_t BME_UBFX_B (volatile uint8_t *addr, uint8_t bit, uint8_t width)
{
	return *(volatile uint8_t*)((uint32_t)addr | BME_UBFX_MASK(bit,width));
}

#endif /* BME_H_ */
