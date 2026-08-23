#include "matrix.h"

float sum = 0;

// Matrix Definition Method
struct Matrix matrix(int m, int n){
  struct Matrix mat;
  mat.m=m; mat.n=n;
  mat_allocate_mem(&mat);
  return mat;
}

// Vector Defintion Methods
struct Matrix vector(int n){ // Column vector
  struct Matrix vec;
  vec.m=n;
  vec.n=1;
  mat_allocate_mem(&vec);
  return vec;
}

struct Matrix vec_row(int n){
  struct Matrix vec;
  vec.m=1;
  vec.n=n;
  mat_allocate_mem(&vec);
  return vec;
}

struct Matrix vec_col(int m){
  struct Matrix vec;
  vec.m=m;
  vec.n=1;
  mat_allocate_mem(&vec);
  return vec;
}

/* -----------  Library Methods -------------------- */
struct Matrix transpose(struct Matrix mat){
  // Make Temporary Flipped Matrix
  struct Matrix tempMat;tempMat.m=mat.n;tempMat.n=mat.m;
  mat_allocate_mem(&tempMat);
  // Fill out the mat ij = ji
  for(int i=0;i<mat.m;i++){
    for(int j=0;j<mat.n;j++){
      tempMat.data[j][i]=mat.data[i][j];
    }
  }
  return tempMat;
}

void mat_free_memory(struct Matrix *mat){
  if(!mat->data) return;
  for(int i=0;i<mat->m;i++){
    free(mat->data[i]);
  }
  free(mat->data);
}

float dot_product(int size,float* a,float* b){
  float res=0.0;
  for (int i=0;i<size;i++){
    res+=a[i]*b[i];
  }
  return res;
}

void mat_allocate_mem(struct Matrix *mat){
  int n=(*mat).n;
  // Make M rows, each element of the row is a pointer to the column
  mat->data= (float**)malloc(sizeof(float*)*(*mat).m);
  for(int i=0;i<(*mat).m;i++){
    mat->data[i]= (float*)malloc(sizeof(float)*(*mat).n);
  }
}

void mat_mult(struct Matrix matA,struct Matrix matB,struct Matrix* result){
  int n=matA.n;
  // This Makes Result Matrix overwriteable, and doesn't assign memory for it every time if the dims are correct
  if(result->m!=matA.m &&result->n!=matB.n){
    mat_free_memory(result);
    result->m= matA.m;
    result->n= matB.n;
    mat_allocate_mem(result);
  }
  if (matA.n==matB.m){ // If dimensions are correct
    result->m=matA.m;
    result->n=matB.n;
    for (int i=0;i<matA.m;i++){ // Iterate over the M rows
      for (int j=0;j<matB.n;j++){ // Iterate over N columns
        sum=0;
        for (int k=0;k<matA.n;k++){
          sum+=matA.data[i][k]*matB.data[k][j];
        }
        result->data[i][j]=sum;
      }
    }
  }else{
    #ifdef DEBUGMAT
    printf("%s","Incompatible Matrix Dimension");
    #endif
  }
}

void mat_add(struct Matrix* dst, struct Matrix* a, struct Matrix* b) {
    for (int i = 0; i < a->m; i++)
        for (int j = 0; j < a->n; j++)
            dst->data[i][j] = a->data[i][j] + b->data[i][j];
}

void mat_sub(struct Matrix* dst, struct Matrix* a, struct Matrix* b) {
    for (int i = 0; i < a->m; i++)
        for (int j = 0; j < a->n; j++)
            dst->data[i][j] = a->data[i][j] - b->data[i][j];
}

void mat_scalar_mul(struct Matrix* dst, struct Matrix* a, float s) {
    for (int i = 0; i < a->m; i++)
        for (int j = 0; j < a->n; j++)
            dst->data[i][j] = a->data[i][j] * s;
}

void mat_set_zero(struct Matrix* a) {
    for (int i = 0; i < a->m; i++)
        for (int j = 0; j < a->n; j++)
            a->data[i][j] = 0.0f;
}

void mat_copy(struct Matrix* dst, struct Matrix* src) {
    for (int i = 0; i < src->m; i++)
        for (int j = 0; j < src->n; j++)
            dst->data[i][j] = src->data[i][j];
}

#ifdef DEBUGMAT
int main(){
  do{
    struct Matrix a; a.m=3;a.n=2; mat_allocate_mem(&a);
    a.data[0][0]=1; a.data[0][1]=2; a.data[1][0]=3; a.data[1][1]=4; a.data[2][0]=5; a.data[2][1]=6;
    struct Matrix result=transpose(a);
    printf("%f", result.data[0][0]);
    printf("%s"," ");
    printf("%f", result.data[1][0]);
    printf("%s"," ");
    printf("%f\n", result.data[0][1]);
    printf("%f", result.data[1][1]);
    printf("%s"," ");
    printf("%f", result.data[0][2]);
    printf("%s"," ");
    printf("%f\n", result.data[1][2]);
    printf("%s"," ");
    printf("%s\n", "---------------------");
    mat_free_memory(&a);
    mat_free_memory(&result);
  }while(false);
  return 0;
}
#endif
