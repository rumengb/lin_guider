/****************************************
 * Guider math functions
 *         (c)2015 Rumen G.Bogdanovski
 ****************************************/

#ifndef __GUIDER_MATH_H
#define __GUIDER_MATH_H

typedef enum {
	none = 0,
	centroid,
	donuts,
	hole,
} guide_algorithm;

typedef struct {
	guide_algorithm algorithm;
	unsigned int width;
	unsigned int height;
	double snr;
	union {
		double (*fft_x)[2];
		double centroid_x;
	};
	union {
		double (*fft_y)[2];
		double centroid_y;
	};
} frame_digest;

typedef struct {
	double x;
	double y;
} corrections;

#ifdef __cplusplus
extern "C" {
#endif

#define RE (0)
#define IM (1)
#define PI_2 (6.2831853071795864769252867665590057683943L)

/* function prototypes */
void fft(const int n, const double (*x)[2], double (*X)[2]);
void ifft(const int n, const double (*X)[2], double (*x)[2]);
void corellate(const int n, const double (*x1)[2], const double (*x2)[2], double (*c)[2]);
void corellate_fft(const int n, const double (*X1)[2], const double (*X2)[2], double (*c)[2]);
double find_distance(const int n, const double (*c)[2]);
double noise_stddev(double *data, int width, int height);
double sigma_threshold(double *data, int width, int height, double nsigma);
int next_power_2(const int n);

#ifdef __cplusplus
}
#endif

#endif /* __GUIDER_MATH_H */
