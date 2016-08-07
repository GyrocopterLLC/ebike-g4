#ifndef DELI_MATH_H_
#define DELI_MATH_H_

/* deli_math.h
 * David's Ebike Library:
 * Fixed point math functions
 */


#ifndef GLOBAL_FXD
#define GLOBAL_FXD		20
#endif //GLOBAL_FXD

#define FXD(value)					((value) << GLOBAL_FXD)
#define BIG_FXD(value)				(((uint64_t)(value)) << GLOBAL_FXD)
#define FLT_TO_FXD(value)			((uint32_t)((value) * ((float)(1<<GLOBAL_FXD))))
#define FLT_TO_BIG_FXD(value)		((uint64_t)((value) * ((float)(1<<GLOBAL_FXD))))
#define FXD_TO_FLT(value)			(((float)(value)) / ((float)(1<<GLOBAL_FXD)))
#define FXD_MULT(value1, value2)	((uint32_t)((((uint64_t)(value1))*((uint64_t)(value2)))>>GLOBAL_FXD))
#define FXD_DIV(value1, value2)		((uint32_t)((((uint64_t)(value1))<<GLOBAL_FXD)/(value2)))



#endif /* DELI_MATH_H_ */
