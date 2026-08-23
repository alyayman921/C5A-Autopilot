#include "matrix.h"

struct Matrix matrix(int m, int n){
  struct Matrix mat;
  mat.m=m; mat.n=n;
  allocate_mem(&mat);
  return mat;
}

void allocate_mem(struct Matrix *mat){
  int n=(*mat).n;
  // Make M rows, each element of the row is a pointer to the column
  mat->data= (float**)malloc(sizeof(float*)*(*mat).m);
  for(int i=0;i<(*mat).m;i++){
    mat->data[i]= (float*)malloc(sizeof(float)*(*mat).n);
  }
}

void free_memory(struct Matrix *mat){
  for(int i=0;i<mat->n;i++){
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

void mat_mult(struct Matrix matA,struct Matrix matB,struct Matrix* result){
  int m=matA.m;
  int n=matA.n;
  result->m= matA.m;
  result->n= matB.n;
  allocate_mem(result);
  if (matA.n==matB.m){ // If dimensions are correct
    result->m=matA.m;
    result->n=matB.n;
    float *colJ= (float*)malloc(sizeof(float)*m);
    for (int i=0;i<m;i++){ // Iterate over the M rows
      for (int j=0;j<n;j++){ // Iterate over N columns
        for (int k=0;k<n;k++){
          colJ[k]=matB.data[k][j];
        }
        result->data[i][j]=dot_product(n, matA.data[i], colJ);
      }
    }
    free(colJ);
  }else{
    #ifdef DEBUG
    printf("%s","Incompatible Matrix Dimension");
    #endif
  }
}

// int main(){
//   while(true){
//     struct Matrix a;
//     a.m=3;a.n=2;
//     allocate_mem(&a);
//     a.data[0][0]=1;
//     a.data[0][1]=2;
//     a.data[1][0]=3;
//     a.data[1][1]=4;
//     a.data[2][0]=5;
//     a.data[2][1]=6;
//     struct Matrix b; // Identity
//     b.m=2;b.n=2;
//     allocate_mem(&b);
//     b.data[0][0]=1;
//     b.data[0][1]=0;
//     b.data[1][0]=0;
//     b.data[1][1]=2;

//     struct Matrix result;
//     mat_mult(a,b,&result);
//     #ifdef DEBUG
//     printf("%f", result.data[0][0]);
//     printf("%s"," ");
//     printf("%f\n", result.data[0][1]);
//     printf("%f", result.data[1][0]);
//     printf("%s"," ");
//     printf("%f\n", result.data[1][1]);
//     printf("%f", result.data[2][0]);
//     printf("%s"," ");
//     printf("%f\n", result.data[2][1]);
//     printf("%s\n", "---------------------");
//     #endif
//     free_memory(&a);free_memory(&b);free_memory(&result);
//   }
//   return 0;
// }
