#ifndef HELPERS_H
#define HELPERS_H

#define SET_BIT(REG, BIT)    ((REG) |=  (1U << (BIT)))
#define CLEAR_BIT(REG, BIT)  ((REG) &= ~(1U << (BIT)))
#define TOGGLE_BIT(REG, BIT) ((REG) ^=  (1U << (BIT)))
#define READ_BIT(REG, BIT)   (((REG) >> (BIT)) & 1U)

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x)    ((x) < 0 ? -(x) : (x))

#endif /* HELPERS_H */