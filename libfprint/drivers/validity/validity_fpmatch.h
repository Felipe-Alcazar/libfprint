/*
 * Host-side matching for small partial fingerprint images (06cb:00a2).
 *
 * Plain C99 + libm, no image library: preprocessing (2x bicubic upscale,
 * CLAHE, light blur), SIFT keypoints and descriptors, ratio-test matching
 * and a RANSAC similarity fit. The score is the number of geometrically
 * consistent matches, as in fp00a2/matcher.py.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#pragma once

#include <stdint.h>

#define FPM_DESC_LEN 128

typedef struct
{
  float x, y;        /* in upscaled image coordinates */
  float size;        /* keypoint diameter */
  float angle;       /* degrees, 0..360 */
  float response;    /* |DoG| at the extremum, for keeping the strongest */
  int     octave;
  float   work[3];             /* scratch during extraction */
  uint8_t desc[FPM_DESC_LEN];  /* SIFT descriptor, 0..255 like OpenCV's */
} FpmKeypoint;

typedef struct
{
  int          n;
  FpmKeypoint *kp;
} FpmFeatures;

/* Extract up to max_kp strongest keypoints from an 8-bit grayscale image.
 * Returns 0 on success; free with fpm_features_clear(). */
int  fpm_extract (const uint8_t *img, int width, int height, int max_kp, FpmFeatures *out);
void fpm_features_clear (FpmFeatures *f);

/* Number of matches consistent with one rotation + translation (scale kept
 * within 0.85..1.15), 0 when there is no such fit. Deterministic. */
int  fpm_score (const FpmFeatures *a, const FpmFeatures *b);
