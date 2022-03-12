/*********************************************************************
 *  File:         matrix_math.h
 *  Description:  Definitions of functions to perform matrix math
 *  Author:       Mike Lanighan
 *  Date:         2014
 *********************************************************************/
#include <math.h>
#include <stdio.h>

void matrix_copy(), matrix_mult(), matrix_transpose(), matrix_add();
void matrix_subtract(), pseudoinverse(), pseudoinverse_left(), nullspace();
int matrix_invert();
void matrix_eigenvals_vecs_2x2();
void matrix_eigen_vecs_2x2();
double determinant_nxn(int n, double A[n][n]);
// Homogeneous Transform operators
void HT_invert(), construct_wTb();






