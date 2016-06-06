/****************************************
 * Guider math functions
 *         (c)2015 Rumen G.Bogdanovski
 ****************************************/

#include <stdlib.h>
#include <math.h>
#include <guider_math.h>

#ifdef DEBUG
	#include <stdio.h>
#endif


void _fft(const int n, const int offset, const int delta, const double (*x)[2], double (*X)[2], double (*_X)[2]);


void fft(const int n, const double (*x)[2], double (*X)[2]) {
	double (*_X)[2] = (double (*)[2])malloc(2 * n * sizeof(double));
	_fft(n, 0, 1, x, X, _X);
	free(_X);
}


void _fft(const int n, const int offset, const int delta, const double (*x)[2], double (*X)[2], double (*_X)[2]) {
	int n2 = n / 2;
	int k;
	double ccos, csin;
	int k00, k01, k10, k11;
	double tmp0, tmp1;

	if(n != 2) {
		_fft(n2, offset, 2 * delta, x, _X, X);
		_fft(n2, offset + delta, 2 * delta, x, _X, X);
		for(k = 0; k < n2; k++) {
			k00 = offset + k * delta;
			k01 = k00 + n2 * delta;
			k10 = offset + 2 * k * delta;
			k11 = k10 + delta;

			ccos = cos(PI_2 * k / (double)n);
			csin = sin(PI_2 * k / (double)n);
			tmp0 = ccos * _X[k11][RE] + csin * _X[k11][IM];
			tmp1 = ccos * _X[k11][IM] - csin * _X[k11][RE];

			X[k01][RE] = _X[k10][RE] - tmp0;
			X[k01][IM] = _X[k10][IM] - tmp1;
			X[k00][RE] = _X[k10][RE] + tmp0;
			X[k00][IM] = _X[k10][IM] + tmp1;
		}
	} else {
		k00 = offset;
		k01 = k00 + delta;

		X[k01][RE] = x[k00][RE] - x[k01][RE];
		X[k01][IM] = x[k00][IM] - x[k01][IM];
		X[k00][RE] = x[k00][RE] + x[k01][RE];
		X[k00][IM] = x[k00][IM] + x[k01][IM];
	}
}


void ifft(const int n, const double (*X)[2], double (*x)[2]) {
	int n2 = n / 2;
	int i;
	double tmp0, tmp1;

	fft(n, X, x);
	x[0][RE] = x[0][RE] / n;
	x[0][IM] = x[0][IM] / n;
	x[n2][RE] = x[n2][RE] / n;
	x[n2][IM] = x[n2][IM] / n;

	for(i = 1; i < n2; i++) {
		tmp0 = x[i][RE] / n;
		tmp1 = x[i][IM] / n;

		x[i][RE] = x[n-i][RE] / n;
		x[i][IM] = x[n-i][IM] / n;
		x[n-i][RE] = tmp0;
		x[n-i][IM] = tmp1;
	}
}


void corellate(const int n, const double (*x1)[2], const double (*x2)[2], double (*c)[2]) {
	int i;
	double (*X1)[2] = (double (*)[2])malloc(2 * n * sizeof(double));
	double (*X2)[2] = (double (*)[2])malloc(2 * n * sizeof(double));
	double (*C)[2] = (double (*)[2])malloc(2 * n * sizeof(double));

	fft(n, x1, X1);
	fft(n, x2, X2);

	/* pointwise multiply X1 conjugate with X2 here */
	for(i = 0; i < n; i++) {
		C[i][RE] = X1[i][RE] * X2[i][RE] + X1[i][IM] * X2[i][IM];
		C[i][IM] = X1[i][IM] * X2[i][RE] - X1[i][RE] * X2[i][IM];
	}

	ifft(n, C, c);

	free(X1);
	free(X2);
	free(C);
}


void corellate_fft(const int n, const double (*X1)[2], const double (*X2)[2], double (*c)[2]) {
	int i;
	double (*C)[2] = (double (*)[2])malloc(2 * n * sizeof(double));

	/* pointwise multiply X1 conjugate with X2 here, store in X1 */
	for(i = 0; i < n; i++) {
		C[i][RE] = X1[i][RE] * X2[i][RE] + X1[i][IM] * X2[i][IM];
		C[i][IM] = X1[i][IM] * X2[i][RE] - X1[i][RE] * X2[i][IM];
	}

	ifft(n, C, c);

	free(C);
}


double find_distance(const int n, const double (*c)[2]) {
	int i;
	const int n2 = n / 2;
	int max = 0;
	int prev = 0, next = 0;
	double max_subp = 0;

	for(i = 0; i < n; i++) {
		max = (c[i][0] > c[max][0]) ? i : max;
	}

	/* find previous and next positions to calculate quadratic interpolation */
	if ((max == 0) || (max == n2)) {
		prev = n - 1;
		next = 1;
	} else if(max == (n-1)) {
		prev = n - 2;
		next = 0;
	} else {
		prev = max - 1;
		next = max + 1;
	}

	/* find subpixel offset of the maximum position using quadratic interpolation */
	if ((2 * c[max][RE] - c[next][RE] - c[prev][RE]) != 0)
		max_subp = (c[next][RE] - c[prev][RE]) / (2 * (2 * c[max][RE] - c[next][RE] - c[prev][RE]));
	else
		max_subp = 0;

#ifdef DEBUG
	printf("max_subp = %5.2f max: %d -> %5.2f %5.2f %5.2f\n", max_subp, max, c[prev][0], c[max][0], c[next][0]);
#endif

	if(max == n2) {
		return max_subp;
	} else if(max > n2) {
		return (double)((max - n) + max_subp);
	} else if (max < n2) {
		return (double)(max + max_subp);
	} else { /* should not be reached */
		return 0;
	}
}

#define N_ROWS 2 /* use first 2 and last 2 lines as noise */
double noise_stddev(double *data, int width, int height) {
	int i;

	double mean = 0.0;
	double deviation = 0.0;
	int n = N_ROWS * width;
	int offset = (width * height) - n;

	for(i=0; i < n; i++) {
		mean += data[i];
		mean += data[i + offset];
	}

	mean=mean / (N_ROWS * n);

	for(i=0; i < n; i++) {
		deviation += (data[i] - mean) * (data[i] - mean);
		deviation += (data[i + offset] - mean) * (data[i + offset] - mean);
	}

	return sqrt(deviation/(N_ROWS * n));
}


double sigma_threshold(double *data, int width, int height, double nsigma) {
	int i;
	double snr = 0, sigma = 0, threshold = 0;

	double mean = 0.0;
	double max = 0.0;
	int n = width * height;

	for(i=0; i<n; i++) {
		mean += data[i];
		if(max < data[i]) max = data[i];
	}
	mean = mean / n;  /* mean is image mean */

	/* sigma is only noise sigma estimaged using first 2 and
	 * last 2 rows of the image (using 2 rows because of the
	 * intereased cameras)
	 */
	sigma = noise_stddev(data, width, height);

	/* threshold is somewhat higher than nsigma because sigma is for
	 * the noise and mean for the whole frame
	 */
	threshold = mean + nsigma * sigma;
	for(i=0; i<n; i++)
		data[i] = (data[i] > threshold) ? data[i] - threshold : 0;

	if (sigma != 0)
		snr = (max - mean) / sigma;
	else
		snr = (max - mean) / 0.001;

	return snr;
}


int next_power_2(const int n) {
	int k = 1;
	while (k < n) k <<= 1;
	return k;
}
