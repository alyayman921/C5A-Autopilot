#ifndef MATRIX_H
#define MATRIX_H

#include <stdlib.h>
#ifdef DEBUGMAT
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct Matrix{
  int m;
  int n;
  float** data;
};

extern float sum;

struct Matrix matrix(int m, int n); // Returns unintialized matrix MxN Size
struct Matrix vector(int n); // Column vector
struct Matrix vec_row(int n);
struct Matrix vec_col(int m);
struct Matrix transpose(struct Matrix mat);
void mat_free_memory(struct Matrix *mat);
void mat_mult(struct Matrix matA,struct Matrix matB,struct Matrix* result);
void mat_add(struct Matrix* dst, struct Matrix* a, struct Matrix* b);
void mat_sub(struct Matrix* dst, struct Matrix* a, struct Matrix* b);
void mat_scalar_mul(struct Matrix* dst, struct Matrix* a, float s);
void mat_set_zero(struct Matrix* a);
void mat_copy(struct Matrix* dst, struct Matrix* src);
/*---------------- Private Functions ------------------------*/
void mat_allocate_mem(struct Matrix *mat); // Free the memory after use
float dot_product(int size,float* a,float* b);

#ifdef __cplusplus
}
#endif

#endif /* MATRIX_H */
