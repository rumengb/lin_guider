/************************************************************************************************
 * Enhanced DONUTS guider API
 *          (c)2015 Rumen G.Bogdanovski
 *
 * DONUTS algorithm is described in:
 * McCormac, J.; Pollacco, D.; Skillen, I.; Faedi, F.; Todd, I.; Watson, C. A.,
 * Publications of the Astronomical Society of Pacific, Volume 125, Issue 927, pp. 548-556 (2013)
 *************************************************************************************************/

#include <stdlib.h>
#include <guider_math.h>
#include <donuts_guide.h>

#ifdef DEBUG
	#include <stdio.h>
#endif

int dg_new_frame_digest(const double *fdata, const unsigned int width, const unsigned int height, frame_digest *fdigest) {
	unsigned int i = 0, ci = 0 , li = 0, max = 0;
	double avg = 0, total = 0;
	double (*col_x)[2] = NULL;
	double (*col_y)[2] = NULL;

	if ((width < 3) || (height < 3)) return -1;
	if ((fdata == NULL) || (fdigest == NULL)) return -1;

	/* initialize digests so that dg_delete_frame_digest() does not fail */
	fdigest->fft_x = NULL;
	fdigest->fft_y = NULL;

	fdigest->width = next_power_2(width);
	fdigest->height = next_power_2(height);
	fdigest->fft_x = (double (*)[2])malloc(2 * fdigest->width * sizeof(double));
	fdigest->fft_y = (double (*)[2])malloc(2 * fdigest->height * sizeof(double));
	if ((fdigest->fft_x == NULL) || (fdigest->fft_y == NULL)) {
		dg_delete_frame_digest(fdigest);
		return -1;
	}

	col_x = (double (*)[2])calloc(2 * fdigest->width * sizeof(double), 1);
	if (col_x == NULL) {
		dg_delete_frame_digest(fdigest);
		return -1;
	}

	col_y = (double (*)[2])calloc(2 * fdigest->height * sizeof(double), 1);
	if (col_y == NULL) {
		dg_delete_frame_digest(fdigest);
		free(col_x);
		return -1;
	}

	fdigest->snr = sigma_threshold((double*)fdata, width, height, 3);
	/* collapse the frame in X and Y directions */
	ci = 0;
	li = 0;
	total = 0;
	max = width * height;
	for(i=0; i < max; i++) {
		col_x[ci][RE] += fdata[i];
		col_y[li][RE] += fdata[i];
		total += fdata[i];
		ci++;
		if (ci == width) {
			ci = 0;
			li++;
		}
	}

	/* !!!! remove background X and Y MAYBE ONT NEEDED !!! - needs proof! */
	/* remove background X */
	avg = total / width;
	for(i=0; i < width; i++) {
		col_x[i][RE] = (col_x[i][RE] > avg) ? col_x[i][RE] - avg : 0;
	}

	/* remove background Y */
	avg = total / height;
	for(i=0; i < height; i++) {
		col_y[i][RE] = (col_y[i][RE] > avg) ? col_y[i][RE] - avg : 0;
	}


#ifdef DEBUG
	printf("donuts col_x:");
	for(i=0; i < fdigest->width; i++) {
		printf(" %5.2f",col_x[i][RE]);
	}
	printf("\n");

	printf("donuts col_y:");
	for(i=0; i < fdigest->height; i++) {
		printf(" %5.2f",col_y[i][RE]);
	}
	printf("\n");
#endif

	fft(fdigest->width, col_x, fdigest->fft_x);
	fft(fdigest->height, col_y, fdigest->fft_y);
	fdigest->algorithm = donuts;

	free(col_x);
	free(col_y);
	return 0;
}


int dg_calculate_corrections(const frame_digest *ref, const frame_digest *current, corrections *c) {
	double (*c_buf)[2] = NULL;
	int max_dim = 0;

	if ((ref == NULL) || (current == NULL) || (c == NULL)) return -1;
	if ((ref->algorithm != donuts) || (current->algorithm != donuts)) return -1;
	if ((ref->width != current->width) || (ref->height != current->height)) return -1;

	max_dim = (ref->width > ref->height) ? ref->width : ref->height;

	c_buf = (double (*)[2])malloc(2 * max_dim * sizeof(double));
	if (c_buf == NULL) return -1;

	/* find X correction */
	corellate_fft(ref->width, ref->fft_x, current->fft_x, c_buf);
	c->x = find_distance(ref->width, c_buf);

	/* find Y correction */
	corellate_fft(ref->height, ref->fft_y, current->fft_y, c_buf);
	c->y = find_distance(ref->height, c_buf);

	free(c_buf);
	return 0;
}


int dg_delete_frame_digest(frame_digest *fdigest) {
	if (fdigest && (fdigest->algorithm == donuts)) {
		if (fdigest->fft_x) free(fdigest->fft_x);
		if (fdigest->fft_y) free(fdigest->fft_y);
		fdigest->width = 0;
		fdigest->height = 0;
		fdigest->snr = 0;
		fdigest->algorithm = none;
		return 0;
	} else {
		return -1;
	}
}
