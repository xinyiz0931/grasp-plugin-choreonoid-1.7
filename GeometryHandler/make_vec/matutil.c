#include <stdio.h>
#include <stdlib.h>
#ifndef SCALAR
    //#define SCALAR float
    #define SCALAR double
#endif
typedef SCALAR *vector, **matrix;

void error(char *message)
{
    fprintf(stderr, "\n%s\n", message);  exit(EXIT_FAILURE);
}

vector newvec(int n)
{
	return (vector)malloc(sizeof(float) * n);
}

matrix newmat(int nrow, int ncol)
{
    int i;
    matrix a;

    a = (matrix)malloc((nrow + 1) * sizeof(void *));
    if (a == NULL) return NULL;  /* �L���̈�s�� */
    for (i = 0; i < nrow; i++) {
        a[i] = (vector)malloc(sizeof(SCALAR) * ncol);
        if (a[i] == NULL) {
            while (--i >= 0) free(a[i]);
            free(a);  return NULL;  /* �L���̈�s�� */
        }
    }
    a[nrow] = NULL;  /* �s�̐����������f���邽�߂̍H�v */
    return (matrix)a;
}

vector new_vector(int n)
{
    vector v;

    v = newvec(n);
    if (v == NULL) error("�L���̈�s��.");
    return (vector)v;
}

matrix new_matrix(int nrow, int ncol)
{
    matrix a;

    a = newmat(nrow, ncol);
    if (a == NULL) error("�L���̈�s��.");
    return a;
}

void free_vector(vector v)
{
    free(v);
}

void free_matrix(matrix a)
{
    matrix b;

    b = a;
    while (*b != NULL) free(*b++);
    free(a);
}

double innerproduct(int n, vector u, vector v)
{
    int i, n5;
    double s;

    s = 0;  n5 = n % 5;
    for (i = 0; i < n5; i++) s += u[i]*v[i];
    for (i = n5; i < n; i += 5)
        s += u[i]*v[i] + u[i+1]*v[i+1] + u[i+2]*v[i+2]
                       + u[i+3]*v[i+3] + u[i+4]*v[i+4];
    return s;
}

void vecprint(vector v, int n, int perline, char *format)
{
    int j, k;

    k = 0;
    for (j = 0; j < n; j++) {
        printf(format, v[j]);
        if (++k >= perline) {  k = 0;  printf("\n");  }
    }
    if (k != 0) printf("\n");
}

void matprint(matrix a, int ncol, int perline, char *format)
{
    int i;

    for (i = 0; a[i] != NULL; i++) {
        vecprint(a[i], ncol, perline, format);
        if (ncol > perline) printf("\n");
    }
}


