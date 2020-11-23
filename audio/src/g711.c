/*
 * g711.c
 *
 * u-law, A-law and linear PCM conversions.
 */

/*
 * This source code is a product of Sun Microsystems, Inc. and is provided
 * for unrestricted use.  Users may copy or modify this source code without
 * charge.
 *
 * SUN SOURCE CODE IS PROVIDED AS IS WITH NO WARRANTIES OF ANY KIND INCLUDING
 * THE WARRANTIES OF DESIGN, MERCHANTIBILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE, OR ARISING FROM A COURSE OF DEALING, USAGE OR TRADE PRACTICE.
 *
 * Sun source code is provided with no support and without any obligation on
 * the part of Sun Microsystems, Inc. to assist in its use, correction,
 * modification or enhancement.
 *
 * SUN MICROSYSTEMS, INC. SHALL HAVE NO LIABILITY WITH RESPECT TO THE
 * INFRINGEMENT OF COPYRIGHTS, TRADE SECRETS OR ANY PATENTS BY THIS SOFTWARE
 * OR ANY PART THEREOF.
 *
 * In no event will Sun Microsystems, Inc. be liable for any lost revenue
 * or profits or other special, indirect and consequential damages, even if
 * Sun has been advised of the possibility of such damages.
 *
 * Sun Microsystems, Inc.
 * 2550 Garcia Avenue
 * Mountain View, California  94043
 */

/*
 * December 30, 1994:
 * Functions linear2alaw, linear2ulaw have been updated to correctly
 * convert unquantized 16 bit values.
 * Tables for direct u- to A-law and A- to u-law conversions have been
 * corrected.
 * Borge Lindberg, Center for PersonKommunikation, Aalborg University.
 * bli@cpk.auc.dk
 *
 */

#include "g711.h"

#define SIGN_BIT    (0x80) /* Sign bit for a A-law byte.        */
#define QUANT_MASK  (0xf) /* Quantization field mask.       */
#define NSEGS       (8) /* Number of A-law segments.        */
#define SEG_SHIFT   (4) /* Left shift for segment number.   */
#define SEG_MASK    (0x70) /* Segment field mask.               */

#define BIAS        (0x84) /* Bias for linear code.             */
// #define CLIP		8159

#define ZEROTRAP                /* turn on the trap as per the MIL-STD */
#define CLIP        32635

static const int exp_lut[256] = {
    0,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
    5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
    6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
    6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
    7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
};

/*
 * linear2alaw() - Convert a 16-bit linear PCM value to 8-bit A-law
 *
 * linear2alaw() accepts an 16-bit integer and encodes it as A-law data.
 *
 *		Linear Input Code	Compressed Code
 *	------------------------	---------------
 *	0000000wxyza			000wxyz
 *	0000001wxyza			001wxyz
 *	000001wxyzab			010wxyz
 *	00001wxyzabc			011wxyz
 *	0001wxyzabcd			100wxyz
 *	001wxyzabcde			101wxyz
 *	01wxyzabcdef			110wxyz
 *	1wxyzabcdefg			111wxyz
 *
 * For further information see John C. Bellamy's Digital Telephony, 1982,
 * John Wiley & Sons, pps 98-111 and 472-476.
 */

#define MAXIMUN 0x7fff
typedef unsigned char byte;

unsigned char
linear2alaw ( int pcm ) /* 2's complement (16-bit range) */
{
    // Get the sign bit.  Shift it for later use without further modification
    int sign = (pcm & 0x8000) >> 8;
    // If the number is negative, make it positive (now it's a magnitude)
    if (sign != 0) {
        pcm = -pcm;
    }
    // The magnitude must fit in 15 bits to avoid overflow
    if (pcm > MAXIMUN) {
        pcm = MAXIMUN;
    }

    /* Finding the "exponent"
     * Bits:
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * S 7 6 5 4 3 2 1 0 0 0 0 0 0 0 0
     * We want to find where the first 1 after the sign bit is.
     * We take the corresponding value from the second row as the exponent value.
     * (i.e. if first 1 at position 7 -> exponent = 2)
     * The exponent is 0 if the 1 is not found in bits 2 through 8.
     * This means the exponent is 0 even if the "first 1" doesn't exist.
     */
    int exponent = 7;
    int expMask;
    // Move to the right and decrement exponent until we hit the 1 or the exponent hits 0
    for (expMask = 0x4000; ((pcm & expMask) == 0) && (exponent > 0); exponent--, expMask >>= 1) {
    }

    /* The last part - the "mantissa"
     * We need to take the four bits after the 1 we just found.
     * To get it, we shift 0x0f :
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * S 0 0 0 0 0 1 . . . . . . . . . (say that exponent is 2)
     * . . . . . . . . . . . . 1 1 1 1
     * We shift it 5 times for an exponent of two, meaning
     * we will shift our four bits (exponent + 3) bits.
     * For convenience, we will actually just shift the number, then AND with 0x0f.
     *
     * NOTE: If the exponent is 0:
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * S 0 0 0 0 0 0 0 Z Y X W V U T S (we know nothing about bit 9)
     * . . . . . . . . . . . . 1 1 1 1
     * We want to get ZYXW, which means a shift of 4 instead of 3
     */
    int mantissa = (pcm >> ((exponent == 0) ? 4 : (exponent + 3))) & 0x0f;

    // The a-law byte bit arrangement is SEEEMMMM (Sign, Exponent, and Mantissa.)
    byte alaw = (byte) (sign | exponent << 4 | mantissa);

    // Last is to flip every other bit, and the sign bit (0xD5 = 1101 0101)
    return (byte) (alaw ^ 0xd5);
}

/*
 * alaw2linear() - Convert an A-law value to 16-bit linear PCM
 *
 */

int
alaw2linear ( unsigned char alaw )
{
    // Invert every other bit, and the sign bit (0xD5 = 1101 0101)
    alaw ^= 0xd5;

    // Pull out the value of the sign bit
    int sign = alaw & 0x80;
    // Pull out and shift over the value of the exponent
    int exponent = (alaw & 0x70) >> 4;
    // Pull out the four bits of data
    int data = alaw & 0x0f;

    // Shift the data four bits to the left
    data <<= 4;
    // Add 8 to put the result in the middle of the range (like adding a half)
    data += 8;

    // If the exponent is not 0, then we know the four bits followed a 1,
    // and can thus add this implicit 1 with 0x100.
    if (exponent != 0) {
        data += 0x100;
    }
    /* Shift the bits to where they need to be: left (exponent - 1) places
     * Why (exponent - 1) ?
     * 1 2 3 4 5 6 7 8 9 A B C D E F G
     * . 7 6 5 4 3 2 1 . . . . . . . . <-- starting bit (based on exponent)
     * . . . . . . . Z x x x x 1 0 0 0 <-- our data (Z is 0 only when exponent is 0)
     * We need to move the one under the value of the exponent,
     * which means it must move (exponent - 1) times
     * It also means shifting is unnecessary if exponent is 0 or 1.
     */
    if (exponent > 1) {
        data <<= (exponent - 1);
    }

    return (short) (sign == 0 ? data : -data);
}

// -----[ ulaw2linear() ]--------------------------------------------------------
// Convert a u-law value to 16-bit linear PCM
//
// First, a biased linear code is derived from the code word. An unbiased
// output can then be obtained by subtracting 33 from the biased code.
//
// Note that this function expects to be passed the complement of the
// original code word. This is in keeping with ISDN conventions.
//
// recibe:  pcm_val
// retorna: linear value
// ------------------------------------------------------------------------------
int
ulaw2linear ( int u_val )
{
    int t;

    /* Complement to obtain normal u-law value. */
    u_val = ~u_val;

    /* Extract and bias the quantization bits. Then
    ** shift up by the segment number and subtract out the bias.
    */
    t = ((u_val & QUANT_MASK) << 3) + BIAS;
    t <<= (u_val & SEG_MASK) >> SEG_SHIFT;

    return (u_val & SIGN_BIT) ? (BIAS - t) : (t - BIAS);
}

/*
** This routine converts from linear to ulaw
**
** Craig Reese: IDA/Supercomputing Research Center
** Joe Campbell: Department of Defense
** 29 September 1989
**
** References:
** 1) CCITT Recommendation G.711  (very difficult to follow)
** 2) "A New Digital Technique for Implementation of Any
**     Continuous PCM Companding Law," Villeret, Michel,
**     et al. 1973 IEEE Int. Conf. on Communications, Vol 1,
**     1973, pg. 11.12-11.17
** 3) MIL-STD-188-113,"Interoperability and Performance Standards
**     for Analog-to_Digital Conversion Techniques,"
**     17 February 1987
**
** Input: Signed 16 bit linear sample
** Output: 8 bit ulaw sample
*/

unsigned char
linear2ulaw ( int sample )
{
    int sign, exponent, mantissa;
    unsigned char ulawbyte;

    /* Get the sample into sign-magnitude. */
    sign = (sample >> 8) & 0x80;        /* set aside the sign	*/
    if (sign != 0) {
        sample = -sample;               /* get magnitude        */
    }
    if (sample > CLIP) {
        sample = CLIP;                  /* clip the magnitude	*/
    }
    /* Convert from 16 bit linear to ulaw. */
    sample = sample + BIAS;
    exponent = exp_lut[(sample >> 7) & 0xFF];
    mantissa = (sample >> (exponent + 3)) & 0x0F;
    ulawbyte = ~(sign | (exponent << 4) | mantissa);
#ifdef ZEROTRAP
    if (ulawbyte == 0) {
        ulawbyte = 0x02;                /* optional CCITT trap */
    }
#endif
    return ulawbyte;
}

#if 0
/**
 * @brief		Init the G711 uLaw
 * @param		void
 * @retval		void
 *
 */
bool g711_ulaw_init	( void )
{
	return true;
}
#endif
