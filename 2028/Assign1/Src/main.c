/******************************************************************************
 * @project        : CG/[T]EE2028 Assignment 1 Program Template
 * @file           : main.c
 * @original author: CK Tham, ECE NUS
 * @modified by    : Ni Qingqing & Hou Linxin, ECE NUS
 * @brief          : Main program body
 *
 * @description    : This code is based on work originally done by CK Tham.
 *                   Modifications were made to update functionality and
 *                   adapt to current course requirements.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "stdio.h"
#include <math.h> // Include to use roundf

// Necessary function to enable printf() using semihosting
extern void initialise_monitor_handles(void);

// Functions to be written in assembly
extern int* optimize(int* coeff_arr, int x0_int, int lambda_int);

// Optimization implementation in C for accurate result
void optimize_c(int a, int b, float x0, float lambda)
{
	float fp, xprev, change, x=x0;
	int round = 0;
	while (1)
	{
		fp = 2*a*x + b;
		xprev = x;
		change = -lambda*fp;
		x = x + change;
		round = round + 1;

//		printf("x: %.1f, fp: %.1f, change: %.1f\n", x, fp, change); //uncomment to see each step
		if (x==xprev) break;
	}
	printf("xsol : %.1f No. of rounds : %d \n\n", x, round);
	return;
}

// Optimization implementation in C for comparison with ASM output
void optimize_c_rounded(int a, int b, int x0, int lambda)
{
	int fp, xprev, change, x=x0;
	int round = 0;

	while (1)
	{
		fp = 2*a*x + b * 10;
		xprev = x;
		change = -lambda*fp/100;
		x = x + change;
		round = round + 1;

//		printf("x: %d, fp: %d, change: %d\n", x, fp, change); //uncomment to see each step
		if (x==xprev) break;
	}

	float x_f = x/10.0;
	printf("xsol : %.1f No. of rounds : %d \n\n", x_f, round);

	return;
}


int main(void)
{
	// Necessary function to enable printf() using semihosting
	initialise_monitor_handles();

	// modify the following lines for different test cases
	int a=3, b=4, c=-3;   // Polynomial coefficients
	float x0 = -6.7;      // Starting point
 	float lambda = 0.1;   // Learning Rate


 	// NOTE: DO NOT modify the code below

	/*
	 * Multiply by 10 to convert floats (1 decimal place) to integers for assembly,
	 * Divide by 10 to get the true float result back from assembly.
	 */
	int arr[3] = {a*10, b*10, c*10};  //array to pack scaled coefficients
	int x0_int = x0*10;               //Scale starting point
	int lambda_int = lambda*10;       //Scale learning rate


	// call optimize.s
	printf("ASM version:\n");
	int *xsol = optimize((int*)arr, x0_int, lambda_int);
	float xsol_float = xsol[0] / 10.0;
	int xsol_round = xsol[1];
	printf("xsol : %.1f No. of rounds : %d \n\n", xsol_float, xsol_round);

    // call optimize.c
	printf("C version (accurate):\n");
	optimize_c(a, b, x0, lambda);

	// call optimize_c_rounded
	printf("C version (reference):\n");
	optimize_c_rounded(arr[0], arr[1], x0_int, lambda_int);

}


