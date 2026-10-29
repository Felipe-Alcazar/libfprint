/*
 * Host-side matching for small partial fingerprint images (06cb:00a2).
 * See fpmatch.h. The SIFT part follows Lowe's method with OpenCV's
 * default parameters (3 layers per octave, sigma 1.6, upsampled first
 * octave), so results track fp00a2/matcher.py, which used OpenCV.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#include "validity_fpmatch.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Preprocessing, as in matcher.py */
#define UPSCALE        2
#define CLAHE_CLIP     2.0f
#define CLAHE_TILES    4

/* SIFT (OpenCV defaults; contrast threshold as in matcher.py) */
#define N_LAYERS       3
#define SIGMA          1.6f
#define INIT_SIGMA     0.5f
#define CONTRAST_THR   0.02f
#define EDGE_THR       10.0f
#define IMG_BORDER     5
#define MAX_INTERP     5
#define ORI_BINS       36
#define ORI_SIG_FCTR   1.5f
#define ORI_RADIUS     (3 * ORI_SIG_FCTR)
#define ORI_PEAK_RATIO 0.8f
#define DESCR_WIDTH    4
#define DESCR_BINS     8
#define DESCR_SCL_FCTR 3.0f
#define DESCR_MAG_THR  0.2f
#define DESCR_FCTR     512.0f

/* Matching */
#define RATIO          0.75f
#define RANSAC_PX      (6.0f * UPSCALE)
#define RANSAC_ITERS   2000
#define RANSAC_CONF    0.99
#define SCALE_MIN      0.85f
#define SCALE_MAX      1.15f

typedef struct
{
  int    w, h;
  float *p;
} Img;

static Img
img_new (int w, int h)
{
  Img im = { w, h, calloc ((size_t) w * h, sizeof (float)) };

  return im;
}

#define AT(im, x, y) ((im).p[(size_t) (y) * (im).w + (x)])

static int
reflect101 (int i, int n)
{
  if (n == 1)
    return 0;
  while (i < 0 || i >= n)
    i = i < 0 ? -i : 2 * n - 2 - i;
  return i;
}

static int
clampi (int v, int lo, int hi)
{
  return v < lo ? lo : v > hi ? hi : v;
}

/* ---------------- preprocessing ---------------- */

static float
cubic_w (float x)
{
  const float a = -0.75f;

  x = fabsf (x);
  if (x <= 1)
    return ((a + 2) * x - (a + 3)) * x * x + 1;
  if (x < 2)
    return ((a * x - 5 * a) * x + 8 * a) * x - 4 * a;
  return 0;
}

/* OpenCV INTER_CUBIC for an integer factor, replicated border */
static void
upscale_cubic (const uint8_t *src, int w, int h, int k, uint8_t *dst)
{
  int W = w * k, H = h * k;

  for (int y = 0; y < H; y++)
    {
      float fy = (y + 0.5f) / k - 0.5f;
      int y0 = (int) floorf (fy);
      float ty = fy - y0;
      float wy[4] = { cubic_w (ty + 1), cubic_w (ty), cubic_w (1 - ty), cubic_w (2 - ty) };

      for (int x = 0; x < W; x++)
        {
          float fx = (x + 0.5f) / k - 0.5f;
          int x0 = (int) floorf (fx);
          float tx = fx - x0;
          float wx[4] = { cubic_w (tx + 1), cubic_w (tx), cubic_w (1 - tx), cubic_w (2 - tx) };
          float acc = 0;

          for (int j = 0; j < 4; j++)
            {
              int yy = clampi (y0 - 1 + j, 0, h - 1);
              float row = 0;
              for (int i = 0; i < 4; i++)
                row += wx[i] * src[yy * w + clampi (x0 - 1 + i, 0, w - 1)];
              acc += wy[j] * row;
            }
          dst[y * W + x] = (uint8_t) clampi ((int) lrintf (acc), 0, 255);
        }
    }
}

/* OpenCV-style CLAHE (clip 2.0, 4x4 tiles), image size divisible by tiles */
static void
clahe (uint8_t *img, int w, int h)
{
  int tw = w / CLAHE_TILES, th = h / CLAHE_TILES;
  int area = tw * th;
  int clip = (int) (CLAHE_CLIP * area / 256);
  float lut_scale = 255.0f / area;
  uint8_t lut[CLAHE_TILES][CLAHE_TILES][256];

  if (clip < 1)
    clip = 1;
  for (int ty = 0; ty < CLAHE_TILES; ty++)
    for (int tx = 0; tx < CLAHE_TILES; tx++)
      {
        int hist[256] = { 0 };
        int excess = 0, sum = 0;

        for (int y = ty * th; y < (ty + 1) * th; y++)
          for (int x = tx * tw; x < (tx + 1) * tw; x++)
            hist[img[y * w + x]]++;
        for (int i = 0; i < 256; i++)
          if (hist[i] > clip)
            {
              excess += hist[i] - clip;
              hist[i] = clip;
            }
        int batch = excess / 256, residual = excess - batch * 256;
        for (int i = 0; i < 256; i++)
          hist[i] += batch;
        if (residual > 0)
          {
            int step = 256 / residual > 1 ? 256 / residual : 1;
            for (int i = 0; i < 256 && residual > 0; i += step, residual--)
              hist[i]++;
          }
        for (int i = 0; i < 256; i++)
          {
            sum += hist[i];
            lut[ty][tx][i] = (uint8_t) clampi ((int) lrintf (sum * lut_scale), 0, 255);
          }
      }

  for (int y = 0; y < h; y++)
    {
      float tyf = (float) y / th - 0.5f;
      int ty1 = (int) floorf (tyf), ty2 = ty1 + 1;
      float ya = tyf - ty1, ya1 = 1 - ya;

      ty1 = clampi (ty1, 0, CLAHE_TILES - 1);
      ty2 = clampi (ty2, 0, CLAHE_TILES - 1);
      for (int x = 0; x < w; x++)
        {
          float txf = (float) x / tw - 0.5f;
          int tx1 = (int) floorf (txf), tx2 = tx1 + 1;
          float xa = txf - tx1, xa1 = 1 - xa;
          uint8_t v = img[y * w + x];

          tx1 = clampi (tx1, 0, CLAHE_TILES - 1);
          tx2 = clampi (tx2, 0, CLAHE_TILES - 1);
          float r = (lut[ty1][tx1][v] * xa1 + lut[ty1][tx2][v] * xa) * ya1 +
                    (lut[ty2][tx1][v] * xa1 + lut[ty2][tx2][v] * xa) * ya;
          img[y * w + x] = (uint8_t) clampi ((int) lrintf (r), 0, 255);
        }
    }
}

/* Separable Gaussian, reflect-101 border, kernel size as OpenCV picks it.
 * Each row/column is copied into a padded line so the inner loop has no
 * border checks. */
static void
gauss_blur (const Img *src, Img *dst, float sigma, int ksize)
{
  if (ksize <= 0)
    ksize = ((int) lrintf (sigma * 8 + 1)) | 1;
  int r = ksize / 2, w = src->w, h = src->h, n = w > h ? w : h;
  float *k = malloc (sizeof (float) * ksize), s = 0;
  float *line = malloc (sizeof (float) * (n + 2 * r)), *out = malloc (sizeof (float) * n);
  Img tmp = img_new (w, h);

  for (int i = 0; i < ksize; i++)
    {
      float x = i - r;
      k[i] = expf (-x * x / (2 * sigma * sigma));
      s += k[i];
    }
  for (int i = 0; i < ksize; i++)
    k[i] /= s;

  for (int y = 0; y < h; y++)
    {
      for (int x = -r; x < w + r; x++)
        line[x + r] = AT (*src, reflect101 (x, w), y);
      for (int x = 0; x < w; x++)
        {
          float acc = 0;
          for (int i = 0; i < ksize; i++)
            acc += k[i] * line[x + i];
          AT (tmp, x, y) = acc;
        }
    }
  for (int x = 0; x < w; x++)
    {
      for (int y = -r; y < h + r; y++)
        line[y + r] = AT (tmp, x, reflect101 (y, h));
      for (int y = 0; y < h; y++)
        {
          float acc = 0;
          for (int i = 0; i < ksize; i++)
            acc += k[i] * line[y + i];
          out[y] = acc;
        }
      for (int y = 0; y < h; y++)
        AT (*dst, x, y) = out[y];
    }
  free (tmp.p);
  free (line);
  free (out);
  free (k);
}

/* ---------------- SIFT ---------------- */

typedef struct
{
  int  n_oct;
  Img *gauss;   /* n_oct * (N_LAYERS + 3) */
  Img *dog;     /* n_oct * (N_LAYERS + 2) */
} Pyramid;

#define G(py, o, i) ((py)->gauss[(o) * (N_LAYERS + 3) + (i)])
#define D(py, o, i) ((py)->dog[(o) * (N_LAYERS + 2) + (i)])

static void
build_pyramid (const Img *base, Pyramid *py)
{
  float sig[N_LAYERS + 3];
  float k = powf (2.0f, 1.0f / N_LAYERS);

  py->n_oct = (int) lrintf (log2f ((float) (base->w < base->h ? base->w : base->h)) - 2);
  py->gauss = calloc ((size_t) py->n_oct * (N_LAYERS + 3), sizeof (Img));
  py->dog = calloc ((size_t) py->n_oct * (N_LAYERS + 2), sizeof (Img));

  sig[0] = SIGMA;
  for (int i = 1; i < N_LAYERS + 3; i++)
    {
      float prev = powf (k, i - 1) * SIGMA, total = prev * k;
      sig[i] = sqrtf (total * total - prev * prev);
    }
  for (int o = 0; o < py->n_oct; o++)
    for (int i = 0; i < N_LAYERS + 3; i++)
      {
        if (o == 0 && i == 0)
          {
            G (py, 0, 0) = img_new (base->w, base->h);
            memcpy (G (py, 0, 0).p, base->p, sizeof (float) * base->w * base->h);
          }
        else if (i == 0)
          {
            const Img *s = &G (py, o - 1, N_LAYERS);
            Img d = img_new (s->w / 2, s->h / 2);
            for (int y = 0; y < d.h; y++)
              for (int x = 0; x < d.w; x++)
                AT (d, x, y) = AT (*s, 2 * x, 2 * y);
            G (py, o, 0) = d;
          }
        else
          {
            const Img *s = &G (py, o, i - 1);
            G (py, o, i) = img_new (s->w, s->h);
            gauss_blur (s, &G (py, o, i), sig[i], 0);
          }
      }
  for (int o = 0; o < py->n_oct; o++)
    for (int i = 0; i < N_LAYERS + 2; i++)
      {
        const Img *a = &G (py, o, i), *b = &G (py, o, i + 1);
        Img d = img_new (a->w, a->h);
        for (size_t p = 0; p < (size_t) a->w * a->h; p++)
          d.p[p] = b->p[p] - a->p[p];
        D (py, o, i) = d;
      }
}

static void
free_pyramid (Pyramid *py)
{
  for (int i = 0; i < py->n_oct * (N_LAYERS + 3); i++)
    free (py->gauss[i].p);
  for (int i = 0; i < py->n_oct * (N_LAYERS + 2); i++)
    free (py->dog[i].p);
  free (py->gauss);
  free (py->dog);
}

typedef struct
{
  FpmKeypoint *v;
  int          n, cap;
} KpList;

static FpmKeypoint *
kp_push (KpList *l)
{
  if (l->n == l->cap)
    {
      l->cap = l->cap ? l->cap * 2 : 256;
      l->v = realloc (l->v, sizeof (FpmKeypoint) * l->cap);
    }
  memset (&l->v[l->n], 0, sizeof (FpmKeypoint));
  return &l->v[l->n++];
}

/* Sub-pixel/sub-scale refinement and rejection, as OpenCV adjustLocalExtrema */
static int
refine (const Pyramid *py, int o, int *layer, int *r, int *c,
        float *xc_out, float *xr_out, float *xi_out, float *contr_out)
{
  const float img_scale = 1.0f / 255, deriv = img_scale * 0.5f;
  const float second = img_scale, cross = img_scale * 0.25f;
  float xi = 0, xr = 0, xc = 0;
  int i;

  for (i = 0; i < MAX_INTERP; i++)
    {
      const Img *img = &D (py, o, *layer), *prv = &D (py, o, *layer - 1), *nxt = &D (py, o, *layer + 1);
      int x = *c, y = *r;
      float dD[3] = {
        (AT (*img, x + 1, y) - AT (*img, x - 1, y)) * deriv,
        (AT (*img, x, y + 1) - AT (*img, x, y - 1)) * deriv,
        (AT (*nxt, x, y) - AT (*prv, x, y)) * deriv,
      };
      float v2 = AT (*img, x, y) * 2;
      float dxx = (AT (*img, x + 1, y) + AT (*img, x - 1, y) - v2) * second;
      float dyy = (AT (*img, x, y + 1) + AT (*img, x, y - 1) - v2) * second;
      float dss = (AT (*nxt, x, y) + AT (*prv, x, y) - v2) * second;
      float dxy = (AT (*img, x + 1, y + 1) - AT (*img, x - 1, y + 1) - AT (*img, x + 1, y - 1) + AT (*img, x - 1, y - 1)) * cross;
      float dxs = (AT (*nxt, x + 1, y) - AT (*nxt, x - 1, y) - AT (*prv, x + 1, y) + AT (*prv, x - 1, y)) * cross;
      float dys = (AT (*nxt, x, y + 1) - AT (*nxt, x, y - 1) - AT (*prv, x, y + 1) + AT (*prv, x, y - 1)) * cross;
      double H[3][3] = { { dxx, dxy, dxs }, { dxy, dyy, dys }, { dxs, dys, dss } };
      double det = H[0][0] * (H[1][1] * H[2][2] - H[1][2] * H[2][1]) -
                   H[0][1] * (H[1][0] * H[2][2] - H[1][2] * H[2][0]) +
                   H[0][2] * (H[1][0] * H[2][1] - H[1][1] * H[2][0]);
      if (fabs (det) < 1e-30)
        return 0;
      /* X = H^-1 dD (Cramer) */
      double X[3];
      for (int k = 0; k < 3; k++)
        {
          double M[3][3];
          memcpy (M, H, sizeof (M));
          for (int j = 0; j < 3; j++)
            M[j][k] = dD[j];
          X[k] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                  M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                  M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
        }
      xi = (float) -X[2];
      xr = (float) -X[1];
      xc = (float) -X[0];
      if (fabsf (xi) < 0.5f && fabsf (xr) < 0.5f && fabsf (xc) < 0.5f)
        break;
      if (fabsf (xi) > 1e6f || fabsf (xr) > 1e6f || fabsf (xc) > 1e6f)
        return 0;
      *c += (int) lrintf (xc);
      *r += (int) lrintf (xr);
      *layer += (int) lrintf (xi);
      if (*layer < 1 || *layer > N_LAYERS ||
          *c < IMG_BORDER || *c >= img->w - IMG_BORDER ||
          *r < IMG_BORDER || *r >= img->h - IMG_BORDER)
        return 0;
    }
  if (i >= MAX_INTERP)
    return 0;

  {
    const Img *img = &D (py, o, *layer), *prv = &D (py, o, *layer - 1), *nxt = &D (py, o, *layer + 1);
    int x = *c, y = *r;
    float dD[3] = {
      (AT (*img, x + 1, y) - AT (*img, x - 1, y)) * deriv,
      (AT (*img, x, y + 1) - AT (*img, x, y - 1)) * deriv,
      (AT (*nxt, x, y) - AT (*prv, x, y)) * deriv,
    };
    float t = dD[0] * xc + dD[1] * xr + dD[2] * xi;
    float contr = AT (*img, x, y) * img_scale + t * 0.5f;
    if (fabsf (contr) * N_LAYERS < CONTRAST_THR)
      return 0;

    float v2 = AT (*img, x, y) * 2;
    float dxx = (AT (*img, x + 1, y) + AT (*img, x - 1, y) - v2) * second;
    float dyy = (AT (*img, x, y + 1) + AT (*img, x, y - 1) - v2) * second;
    float dxy = (AT (*img, x + 1, y + 1) - AT (*img, x - 1, y + 1) - AT (*img, x + 1, y - 1) + AT (*img, x - 1, y - 1)) * cross;
    float tr = dxx + dyy, det = dxx * dyy - dxy * dxy;
    if (det <= 0 || tr * tr * EDGE_THR >= (EDGE_THR + 1) * (EDGE_THR + 1) * det)
      return 0;
    *contr_out = contr;
  }
  *xc_out = xc;
  *xr_out = xr;
  *xi_out = xi;
  return 1;
}

static float
orientation_hist (const Img *img, int px, int py_, int radius, float sigma, float *hist)
{
  float tmp[ORI_BINS + 4] = { 0 };
  float *t = tmp + 2, expf_scale = -1.0f / (2 * sigma * sigma), maxv = 0;

  for (int i = -radius; i <= radius; i++)
    {
      int y = py_ + i;
      if (y <= 0 || y >= img->h - 1)
        continue;
      for (int j = -radius; j <= radius; j++)
        {
          int x = px + j;
          if (x <= 0 || x >= img->w - 1)
            continue;
          float dx = AT (*img, x + 1, y) - AT (*img, x - 1, y);
          float dy = AT (*img, x, y - 1) - AT (*img, x, y + 1);
          float w = expf ((i * i + j * j) * expf_scale);
          float ori = atan2f (dy, dx) * 180.0f / (float) M_PI;
          if (ori < 0)
            ori += 360;
          int bin = (int) lrintf (ori * ORI_BINS / 360.0f);
          if (bin >= ORI_BINS)
            bin -= ORI_BINS;
          if (bin < 0)
            bin += ORI_BINS;
          t[bin] += w * sqrtf (dx * dx + dy * dy);
        }
    }
  t[-1] = t[ORI_BINS - 1];
  t[-2] = t[ORI_BINS - 2];
  t[ORI_BINS] = t[0];
  t[ORI_BINS + 1] = t[1];
  for (int i = 0; i < ORI_BINS; i++)
    {
      hist[i] = (t[i - 2] + t[i + 2]) * (1.0f / 16) + (t[i - 1] + t[i + 1]) * (4.0f / 16) + t[i] * (6.0f / 16);
      if (hist[i] > maxv)
        maxv = hist[i];
    }
  return maxv;
}

static void
descriptor (const Img *img, float ptx, float pty, float ori, float scl, float *dst)
{
  const int d = DESCR_WIDTH, n = DESCR_BINS;
  int px = (int) lrintf (ptx), py_ = (int) lrintf (pty);
  float cos_t = cosf (ori * (float) M_PI / 180), sin_t = sinf (ori * (float) M_PI / 180);
  float bins_per_rad = n / 360.0f, exp_scale = -1.0f / (d * d * 0.5f);
  float hist_width = DESCR_SCL_FCTR * scl;
  int radius = (int) lrintf (hist_width * 1.4142135623730951f * (d + 1) * 0.5f);
  float hist[(DESCR_WIDTH + 2) * (DESCR_WIDTH + 2) * (DESCR_BINS + 2)] = { 0 };

  {
    int lim = (int) sqrtf ((float) img->w * img->w + (float) img->h * img->h);
    if (radius > lim)
      radius = lim;
  }
  cos_t /= hist_width;
  sin_t /= hist_width;

  for (int i = -radius; i <= radius; i++)
    for (int j = -radius; j <= radius; j++)
      {
        float c_rot = j * cos_t - i * sin_t, r_rot = j * sin_t + i * cos_t;
        float rbin = r_rot + d / 2 - 0.5f, cbin = c_rot + d / 2 - 0.5f;
        int r = py_ + i, c = px + j;

        if (!(rbin > -1 && rbin < d && cbin > -1 && cbin < d && r > 0 && r < img->h - 1 && c > 0 && c < img->w - 1))
          continue;
        float dx = AT (*img, c + 1, r) - AT (*img, c - 1, r);
        float dy = AT (*img, c, r - 1) - AT (*img, c, r + 1);
        float o = atan2f (dy, dx) * 180.0f / (float) M_PI;
        if (o < 0)
          o += 360;
        float mag = sqrtf (dx * dx + dy * dy) * expf ((c_rot * c_rot + r_rot * r_rot) * exp_scale);
        float obin = (o - ori) * bins_per_rad;
        int r0 = (int) floorf (rbin), c0 = (int) floorf (cbin), o0 = (int) floorf (obin);
        rbin -= r0;
        cbin -= c0;
        obin -= o0;
        if (o0 < 0)
          o0 += n;
        if (o0 >= n)
          o0 -= n;

        float v_r1 = mag * rbin, v_r0 = mag - v_r1;
        float v_rc11 = v_r1 * cbin, v_rc10 = v_r1 - v_rc11;
        float v_rc01 = v_r0 * cbin, v_rc00 = v_r0 - v_rc01;
        float v_rco111 = v_rc11 * obin, v_rco110 = v_rc11 - v_rco111;
        float v_rco101 = v_rc10 * obin, v_rco100 = v_rc10 - v_rco101;
        float v_rco011 = v_rc01 * obin, v_rco010 = v_rc01 - v_rco011;
        float v_rco001 = v_rc00 * obin, v_rco000 = v_rc00 - v_rco001;
        int idx = ((r0 + 1) * (d + 2) + c0 + 1) * (n + 2) + o0;
        hist[idx] += v_rco000;
        hist[idx + 1] += v_rco001;
        hist[idx + (n + 2)] += v_rco010;
        hist[idx + (n + 3)] += v_rco011;
        hist[idx + (d + 2) * (n + 2)] += v_rco100;
        hist[idx + (d + 2) * (n + 2) + 1] += v_rco101;
        hist[idx + (d + 3) * (n + 2)] += v_rco110;
        hist[idx + (d + 3) * (n + 2) + 1] += v_rco111;
      }

  for (int i = 0; i < d; i++)
    for (int j = 0; j < d; j++)
      {
        int idx = ((i + 1) * (d + 2) + (j + 1)) * (n + 2);
        hist[idx] += hist[idx + n];
        hist[idx + 1] += hist[idx + n + 1];
        for (int k = 0; k < n; k++)
          dst[(i * d + j) * n + k] = hist[idx + k];
      }
  float nrm2 = 0;
  for (int k = 0; k < FPM_DESC_LEN; k++)
    nrm2 += dst[k] * dst[k];
  float thr = sqrtf (nrm2) * DESCR_MAG_THR;
  nrm2 = 0;
  for (int k = 0; k < FPM_DESC_LEN; k++)
    {
      if (dst[k] > thr)
        dst[k] = thr;
      nrm2 += dst[k] * dst[k];
    }
  nrm2 = DESCR_FCTR / fmaxf (sqrtf (nrm2), FLT_EPSILON);
  for (int k = 0; k < FPM_DESC_LEN; k++)
    dst[k] *= nrm2;
}

static int
cmp_response (const void *a, const void *b)
{
  float ra = ((const FpmKeypoint *) a)->response, rb = ((const FpmKeypoint *) b)->response;

  return ra < rb ? 1 : ra > rb ? -1 : 0;
}

int
fpm_extract (const uint8_t *img, int width, int height, int max_kp, FpmFeatures *out)
{
  int W = width * UPSCALE, H = height * UPSCALE;
  uint8_t *up = malloc ((size_t) W * H);
  Img pre = img_new (W, H), base = img_new (W * 2, H * 2);
  KpList kps = { 0 };
  Pyramid py;

  /* 2x bicubic, CLAHE, 3x3 Gaussian (sigma 0.8 as OpenCV picks for ksize 3) */
  upscale_cubic (img, width, height, UPSCALE, up);
  clahe (up, W, H);
  for (int i = 0; i < W * H; i++)
    pre.p[i] = up[i];
  gauss_blur (&pre, &pre, 0.8f, 3);
  for (int i = 0; i < W * H; i++)
    pre.p[i] = (float) clampi ((int) lrintf (pre.p[i]), 0, 255);
  free (up);

  /* SIFT base: upsample 2x (bilinear), blur to SIGMA */
  for (int y = 0; y < base.h; y++)
    for (int x = 0; x < base.w; x++)
      {
        float fx = (x + 0.5f) / 2 - 0.5f, fy = (y + 0.5f) / 2 - 0.5f;
        int x0 = (int) floorf (fx), y0 = (int) floorf (fy);
        float ax = fx - x0, ay = fy - y0;
        int x1 = clampi (x0 + 1, 0, W - 1), y1 = clampi (y0 + 1, 0, H - 1);
        x0 = clampi (x0, 0, W - 1);
        y0 = clampi (y0, 0, H - 1);
        AT (base, x, y) = (AT (pre, x0, y0) * (1 - ax) + AT (pre, x1, y0) * ax) * (1 - ay) +
                          (AT (pre, x0, y1) * (1 - ax) + AT (pre, x1, y1) * ax) * ay;
      }
  free (pre.p);
  {
    float sig_diff = sqrtf (fmaxf (SIGMA * SIGMA - INIT_SIGMA * INIT_SIGMA * 4, 0.01f));
    gauss_blur (&base, &base, sig_diff, 0);
  }
  build_pyramid (&base, &py);
  free (base.p);

  for (int o = 0; o < py.n_oct; o++)
    for (int l = 1; l <= N_LAYERS; l++)
      {
        const Img *dog = &D (&py, o, l), *prv = &D (&py, o, l - 1), *nxt = &D (&py, o, l + 1);
        for (int r = IMG_BORDER; r < dog->h - IMG_BORDER; r++)
          for (int c = IMG_BORDER; c < dog->w - IMG_BORDER; c++)
            {
              float v = AT (*dog, c, r);
              int ext = 1;

              if (v == 0)
                continue;
              for (int dl = -1; dl <= 1 && ext; dl++)
                {
                  const Img *L = dl < 0 ? prv : dl > 0 ? nxt : dog;
                  for (int dy = -1; dy <= 1 && ext; dy++)
                    for (int dx = -1; dx <= 1 && ext; dx++)
                      {
                        if (!dl && !dy && !dx)
                          continue;
                        float u = AT (*L, c + dx, r + dy);
                        if (v > 0 ? v < u : v > u)
                          ext = 0;
                      }
                }
              if (!ext)
                continue;

              int layer = l, rr = r, cc = c;
              float xc, xr, xi, contr;
              if (!refine (&py, o, &layer, &rr, &cc, &xc, &xr, &xi, &contr))
                continue;

              float scl_octv = SIGMA * powf (2.0f, (layer + xi) / N_LAYERS);
              float hist[ORI_BINS];
              float omax = orientation_hist (&G (&py, o, layer), cc, rr,
                                             (int) lrintf (ORI_RADIUS * scl_octv),
                                             ORI_SIG_FCTR * scl_octv, hist);
              float mag_thr = omax * ORI_PEAK_RATIO;
              for (int j = 0; j < ORI_BINS; j++)
                {
                  int lft = j > 0 ? j - 1 : ORI_BINS - 1, rgt = j < ORI_BINS - 1 ? j + 1 : 0;
                  if (!(hist[j] > hist[lft] && hist[j] > hist[rgt] && hist[j] >= mag_thr))
                    continue;
                  float bin = j + 0.5f * (hist[lft] - hist[rgt]) / (hist[lft] - 2 * hist[j] + hist[rgt]);
                  bin = bin < 0 ? ORI_BINS + bin : bin >= ORI_BINS ? bin - ORI_BINS : bin;
                  float angle = 360.0f - (360.0f / ORI_BINS) * bin;
                  if (fabsf (angle - 360.0f) < FLT_EPSILON)
                    angle = 0;
                  FpmKeypoint *k = kp_push (&kps);
                  /* pyramid octave o is 2x the upsampled base; /2 back to `pre` coords */
                  k->x = (cc + xc) * (float) (1 << o) * 0.5f;
                  k->y = (rr + xr) * (float) (1 << o) * 0.5f;
                  k->size = scl_octv * (float) (1 << o) * 2 * 0.5f;
                  k->angle = angle;
                  k->response = fabsf (contr);
                  k->octave = o * 256 + layer;
                  /* descriptor inputs, in octave coordinates; computed after
                   * the strongest keypoints are chosen */
                  k->work[0] = cc + xc;
                  k->work[1] = rr + xr;
                  k->work[2] = scl_octv;
                }
            }
      }
  qsort (kps.v, kps.n, sizeof (FpmKeypoint), cmp_response);
  if (max_kp > 0 && kps.n > max_kp)
    {
      /* keep ties at the cut, like OpenCV's retainBest */
      float cut = kps.v[max_kp - 1].response;
      int n = max_kp;
      while (n < kps.n && kps.v[n].response >= cut)
        n++;
      kps.n = n;
    }
  for (int i = 0; i < kps.n; i++)
    {
      FpmKeypoint *k = &kps.v[i];
      float a = 360.0f - k->angle, d[FPM_DESC_LEN];
      if (fabsf (a - 360.0f) < FLT_EPSILON)
        a = 0;
      descriptor (&G (&py, k->octave / 256, k->octave % 256), k->work[0], k->work[1], a, k->work[2], d);
      for (int m = 0; m < FPM_DESC_LEN; m++)
        k->desc[m] = (uint8_t) clampi ((int) lrintf (d[m]), 0, 255);
    }
  free_pyramid (&py);
  out->n = kps.n;
  out->kp = kps.v;
  return 0;
}

void
fpm_features_clear (FpmFeatures *f)
{
  free (f->kp);
  f->kp = NULL;
  f->n = 0;
}

/* ---------------- matching ---------------- */

typedef struct
{
  float sx, sy, dx, dy;
} Pair;

static uint32_t
rng_next (uint32_t *s)
{
  *s ^= *s << 13;
  *s ^= *s >> 17;
  *s ^= *s << 5;
  return *s;
}

static int
update_iters (double conf, double outlier_ratio, int iters)
{
  double num = log (1 - conf), denom = 1 - pow (1 - outlier_ratio, 2);

  if (denom < DBL_MIN)
    return 0;
  denom = log (denom);
  return denom >= 0 || -num >= iters * (-denom) ? iters : (int) lrint (num / denom);
}

int
fpm_score (const FpmFeatures *a, const FpmFeatures *b)
{
  Pair *pairs;
  int np = 0;

  if (a->n < 2 || b->n < 2)
    return 0;
  pairs = malloc (sizeof (Pair) * a->n);

  for (int i = 0; i < a->n; i++)
    {
      int d1 = INT32_MAX, d2 = INT32_MAX;
      int best = -1;
      const uint8_t *da = a->kp[i].desc;
      for (int j = 0; j < b->n; j++)
        {
          const uint8_t *db = b->kp[j].desc;
          int s = 0;
          for (int k = 0; k < FPM_DESC_LEN; k++)
            {
              int t = da[k] - db[k];
              s += t * t;
            }
          if (s < d1)
            {
              d2 = d1;
              d1 = s;
              best = j;
            }
          else if (s < d2)
            {
              d2 = s;
            }
        }
      if (best >= 0 && sqrtf ((float) d1) < RATIO * sqrtf ((float) d2))
        pairs[np++] = (Pair) { a->kp[i].x, a->kp[i].y, b->kp[best].x, b->kp[best].y };
    }
  if (np < 4)
    {
      free (pairs);
      return 0;
    }

  /* RANSAC over 2-point similarity transforms */
  uint32_t seed = 0x9e3779b9u;
  int best_inl = 0, iters = RANSAC_ITERS;
  float best_a = 1, best_b = 0;
  const float thr2 = RANSAC_PX * RANSAC_PX;

  for (int it = 0; it < iters; it++)
    {
      int i = rng_next (&seed) % np, j = rng_next (&seed) % np;
      if (i == j)
        continue;
      float sx = pairs[j].sx - pairs[i].sx, sy = pairs[j].sy - pairs[i].sy;
      float dx = pairs[j].dx - pairs[i].dx, dy = pairs[j].dy - pairs[i].dy;
      float n2 = sx * sx + sy * sy;
      if (n2 < 1e-3f)
        continue;
      /* [a -b; b a] maps (sx,sy) to (dx,dy) */
      float ca = (sx * dx + sy * dy) / n2, cb = (sx * dy - sy * dx) / n2;
      float tx = pairs[i].dx - (ca * pairs[i].sx - cb * pairs[i].sy);
      float ty = pairs[i].dy - (cb * pairs[i].sx + ca * pairs[i].sy);
      int inl = 0;
      for (int k = 0; k < np; k++)
        {
          float ex = ca * pairs[k].sx - cb * pairs[k].sy + tx - pairs[k].dx;
          float ey = cb * pairs[k].sx + ca * pairs[k].sy + ty - pairs[k].dy;
          if (ex * ex + ey * ey <= thr2)
            inl++;
        }
      if (inl > best_inl)
        {
          best_inl = inl;
          best_a = ca;
          best_b = cb;
          iters = update_iters (RANSAC_CONF, (double) (np - inl) / np, iters);
        }
    }
  free (pairs);
  if (best_inl < 2)
    return 0;
  float scale = sqrtf (best_a * best_a + best_b * best_b);
  if (scale <= SCALE_MIN || scale >= SCALE_MAX)
    return 0;
  return best_inl;
}
