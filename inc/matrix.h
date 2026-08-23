#include <stdlib.h>
#ifdef DEBUG
#include <stdio.h>
#endif

struct Matrix{
  int m;
  int n;
  float** data;
};

struct Matrix matrix(int m, int n); // Returns unintialized matrix MxN Size

void allocate_mem(struct Matrix *mat); // Free the memory after use

void free_memory(struct Matrix *mat);

float dot_product(int size,float* a,float* b);

void mat_mult(struct Matrix matA,struct Matrix matB,struct Matrix* result);
