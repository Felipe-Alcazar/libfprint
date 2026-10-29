/*
 * Match-on-host support for Validity sensors (06cb:00a2).
 *
 * These sensors run the same capture program as the match-on-chip ones,
 * but answer the image download command with the image itself (112x112,
 * 8 bit) instead of only the capture geometry. Matching happens here,
 * with validity_fpmatch (SIFT keypoints + RANSAC), against templates kept
 * in the FpPrint, which fprintd stores on disk.
 *
 * The print also keeps the sensor calibration from enrollment. A fresh
 * calibration on every open shifts the image enough that prints enrolled
 * in one session stop matching in the next, so verify captures with the
 * enrolled print's calibration.
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#define FP_COMPONENT "validity"

#include "drivers_api.h"
#include "fpi-print.h"
#include "validity.h"
#include "validity_fpmatch.h"
#include "validity_moh.h"
#include "validity_capture.h"
#include "vcsfw_protocol.h"

#include <math.h>

/* One stored view must reach this many consistent matches. Across 5436
 * impostor pairs from this sensor the maximum was 4. */
#define MOH_MATCH_THRESHOLD   20
/* An enroll capture this similar to an earlier one adds no coverage. */
#define MOH_DUPLICATE_SCORE   250
#define MOH_MAX_KEYPOINTS     500
/* Enrollment keeps only clear captures (see validity_moh_quality). */
#define MOH_ENROLL_MIN_SCORE  68
#define MOH_ENROLL_MIN_COVER  0.9
#define MOH_MIN_KEYPOINTS     50
/* Verify asks for another touch instead of matching a smeared or partial
 * image (finger slid): such images almost never match, and a failed match
 * would use up one of the user's attempts. */
#define MOH_VERIFY_MIN_SCORE  60
#define MOH_VERIFY_MIN_COVER  0.75
#define MOH_TEMPLATE_VERSION  1
#define MOH_MAX_IMAGE_BYTES   (256 * 256)

gboolean
validity_moh_is_moh (FpiDeviceValidity *self)
{
  return self->dev_type == VALIDITY_DEV_A2;
}

/* ---------------- image download ---------------- */

void
validity_moh_image_reset (FpiDeviceValidity *self)
{
  g_clear_pointer (&self->moh_img, g_byte_array_unref);
  self->moh_w = self->moh_h = 0;
}

static guint32
le32 (const guint8 *d)
{
  return d[0] | (d[1] << 8) | (d[2] << 16) | ((guint32) d[3] << 24);
}

gint
validity_moh_image_add (FpiDeviceValidity *self,
                        guint16            status,
                        const guint8      *data,
                        gsize              len)
{
  gsize off, size;

  if (status != VCSFW_STATUS_OK || data == NULL)
    return -1;

  if (self->moh_img == NULL)
    {
      /* First chunk: size(4) width(2) height(2) ...(8) pixels */
      if (len < 16)
        return -1;
      size = le32 (data);
      self->moh_w = data[4] | (data[5] << 8);
      self->moh_h = data[6] | (data[7] << 8);
      if (self->moh_w == 0 || self->moh_h == 0 ||
          (gsize) self->moh_w * self->moh_h > MOH_MAX_IMAGE_BYTES)
        return -1;
      self->moh_img = g_byte_array_sized_new (self->moh_w * self->moh_h);
      off = 16;
    }
  else
    {
      /* Next chunks: size(4) pixels; size 0 once the image was sent */
      if (len < 4)
        return -1;
      size = le32 (data);
      if (size == 0)
        return self->moh_img->len >= self->moh_w * self->moh_h ? 1 : -1;
      off = 4;
    }

  if (4 + size < off || len < off)
    return -1;
  size = MIN (len, 4 + size) - off;
  g_byte_array_append (self->moh_img, data + off, size);

  return self->moh_img->len >= self->moh_w * self->moh_h ? 1 : 0;
}

/* ---------------- lift check ---------------- */

guint8 *
validity_moh_frame_cmd (FpiDeviceValidity *self,
                        gsize             *cmd_len,
                        gsize             *frame_len)
{
  guint8 *cmd = validity_capture_build_cmd_02 (&self->capture, self->sensor.type_info,
                                               VALIDITY_CAPTURE_CALIBRATE, cmd_len);
  guint16 lines = self->capture.lines_per_frame + 1;

  if (cmd == NULL || *cmd_len < 5)
    return NULL;
  /* One frame instead of the calibration's several: bytes 3..4 = line count */
  cmd[3] = lines & 0xff;
  cmd[4] = lines >> 8;
  *frame_len = (gsize) lines * self->capture.bytes_per_line;
  return cmd;
}

gboolean
validity_moh_frame_has_finger (FpiDeviceValidity *self,
                               const guint8      *frame,
                               gsize              len)
{
  /* The calibrated empty sensor is flat (std ~3); ridges are not. Each line
   * starts with an 8-byte header. */
  const gsize bpl = self->capture.bytes_per_line;
  const gsize lines = self->capture.lines_per_frame;
  gdouble sum = 0, sum2 = 0;
  gsize n = 0;

  if (bpl <= 8 || len < lines * bpl)
    return TRUE;   /* short read: assume the finger is still there, check again */
  for (gsize l = 0; l < lines; l++)
    for (gsize x = 8; x < bpl; x++)
      {
        gdouble v = frame[l * bpl + x];
        sum += v;
        sum2 += v * v;
        n++;
      }
  return sqrt (sum2 / n - (sum / n) * (sum / n)) > 10;
}

/* ---------------- quality ---------------- */

void
validity_moh_quality (const guint8 *img,
                      guint         width,
                      guint         height,
                      gint         *score,
                      gdouble      *coverage)
{
  /* Orientation coherence of the ridge gradients in 14x14 blocks: high for
   * a clean ridge/valley pattern, low for smeared or blurred touches. */
  const guint block = 14;
  gdouble coh_sum = 0;
  guint fg = 0, n = 0;

  for (guint by = 0; by + block <= height; by += block)
    for (guint bx = 0; bx + block <= width; bx += block)
      {
        gdouble mean = 0, var = 0, gxx = 0, gyy = 0, gxy = 0;

        n++;
        for (guint y = by; y < by + block; y++)
          for (guint x = bx; x < bx + block; x++)
            mean += img[y * width + x];
        mean /= block * block;
        for (guint y = by; y < by + block; y++)
          for (guint x = bx; x < bx + block; x++)
            var += (img[y * width + x] - mean) * (img[y * width + x] - mean);
        if (sqrt (var / (block * block)) < 18)
          continue;   /* flat: no finger here */
        fg++;
        for (guint y = by; y < by + block; y++)
          for (guint x = bx; x < bx + block; x++)
            {
              guint xl = x > 0 ? x - 1 : x, xr = x + 1 < width ? x + 1 : x;
              guint yu = y > 0 ? y - 1 : y, yd = y + 1 < height ? y + 1 : y;
              gdouble gx = (gdouble) img[y * width + xr] - img[y * width + xl];
              gdouble gy = (gdouble) img[yd * width + x] - img[yu * width + x];
              gxx += gx * gx;
              gyy += gy * gy;
              gxy += gx * gy;
            }
        if (gxx + gyy > 0)
          coh_sum += sqrt ((gxx - gyy) * (gxx - gyy) + 4 * gxy * gxy) / (gxx + gyy);
      }

  *coverage = n ? (gdouble) fg / n : 0;
  *score = fg ? (gint) lround (100 * (coh_sum / fg) * MIN (1.0, *coverage / 0.8)) : 0;
}

/* ---------------- templates ---------------- */

#define KP_BYTES (2 * sizeof (gfloat) + FPM_DESC_LEN)

static GBytes *
features_to_bytes (const FpmFeatures *f)
{
  gsize len = 4 + (gsize) f->n * KP_BYTES;
  guint8 *buf = g_malloc (len), *p = buf;
  guint32 n = GUINT32_TO_LE (f->n);

  memcpy (p, &n, 4);
  p += 4;
  for (int i = 0; i < f->n; i++)
    {
      memcpy (p, &f->kp[i].x, sizeof (gfloat));
      memcpy (p + sizeof (gfloat), &f->kp[i].y, sizeof (gfloat));
      memcpy (p + 2 * sizeof (gfloat), f->kp[i].desc, FPM_DESC_LEN);
      p += KP_BYTES;
    }
  return g_bytes_new_take (buf, len);
}

static gboolean
features_from_bytes (const guint8 *buf, gsize len, FpmFeatures *f)
{
  guint32 n;

  if (len < 4)
    return FALSE;
  memcpy (&n, buf, 4);
  n = GUINT32_FROM_LE (n);
  if (n > 4096 || len != 4 + (gsize) n * KP_BYTES)
    return FALSE;
  f->n = n;
  f->kp = g_new0 (FpmKeypoint, n);
  for (guint32 i = 0; i < n; i++)
    {
      const guint8 *p = buf + 4 + i * KP_BYTES;
      memcpy (&f->kp[i].x, p, sizeof (gfloat));
      memcpy (&f->kp[i].y, p + sizeof (gfloat), sizeof (gfloat));
      memcpy (f->kp[i].desc, p + 2 * sizeof (gfloat), FPM_DESC_LEN);
    }
  return TRUE;
}

static void
features_free (gpointer p)
{
  FpmFeatures *f = p;

  fpm_features_clear (f);
  g_free (f);
}

/* (version, calibration, [view, ...]) */
static GVariant *
template_to_variant (const guint8 *calib, gsize calib_len, GPtrArray *views)
{
  GVariantBuilder vb;

  g_variant_builder_init (&vb, G_VARIANT_TYPE ("aay"));
  for (guint i = 0; i < views->len; i++)
    {
      g_autoptr(GBytes) b = features_to_bytes (g_ptr_array_index (views, i));
      g_variant_builder_add_value (&vb, g_variant_new_from_bytes (G_VARIANT_TYPE_BYTESTRING, b, TRUE));
    }
  return g_variant_new ("(u@ay@aay)", MOH_TEMPLATE_VERSION,
                        g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, calib, calib_len, 1),
                        g_variant_builder_end (&vb));
}

static GVariant *
print_template (FpPrint *print)
{
  g_autoptr(GVariant) data = NULL;
  guint32 version;

  if (print == NULL)
    return NULL;
  g_object_get (print, "fpi-data", &data, NULL);
  if (data == NULL || !g_variant_is_of_type (data, G_VARIANT_TYPE ("(uayaay)")))
    return NULL;
  g_variant_get_child (data, 0, "u", &version);
  if (version != MOH_TEMPLATE_VERSION)
    return NULL;
  return g_steal_pointer (&data);
}

void
validity_moh_apply_calibration (FpiDeviceValidity *self,
                                FpPrint           *print)
{
  g_autoptr(GVariant) tpl = print_template (print);
  g_autoptr(GVariant) calib = NULL;
  const guint8 *data;
  gsize len;

  if (tpl == NULL)
    return;
  calib = g_variant_get_child_value (tpl, 1);
  data = g_variant_get_fixed_array (calib, &len, 1);
  if (len == 0)
    return;
  g_free (self->capture.calib_data);
  self->capture.calib_data = g_memdup2 (data, len);
  self->capture.calib_data_len = len;
}

/* Best single-view score of the probe against the print, stopping at the
 * first view that reaches the threshold. */
static gint
match_print (FpPrint *print, const FpmFeatures *probe)
{
  g_autoptr(GVariant) tpl = print_template (print);
  g_autoptr(GVariant) views = NULL;
  gint best = 0;

  if (tpl == NULL)
    return 0;
  views = g_variant_get_child_value (tpl, 2);
  for (gsize i = 0; i < g_variant_n_children (views) && best < MOH_MATCH_THRESHOLD; i++)
    {
      g_autoptr(GVariant) v = g_variant_get_child_value (views, i);
      gsize len;
      const guint8 *buf = g_variant_get_fixed_array (v, &len, 1);
      FpmFeatures f = { 0 };

      if (!features_from_bytes (buf, len, &f))
        continue;
      best = MAX (best, fpm_score (probe, &f));
      fpm_features_clear (&f);
    }
  return best;
}

/* ---------------- enroll / verify / identify ---------------- */

void
validity_moh_enroll (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);

  g_clear_pointer (&self->moh_views, g_ptr_array_unref);
  self->moh_views = g_ptr_array_new_with_free_func (features_free);
  self->moh_enroll = TRUE;
  self->identify_mode = FALSE;
  validity_verify_run_capture (device);
}

static void
enroll_finish (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpPrint *print = NULL;
  GVariant *data;

  fpi_device_get_enroll_data (device, &print);
  data = template_to_variant (self->capture.calib_data, self->capture.calib_data_len,
                              self->moh_views);
  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, FALSE);
  g_object_set (print, "fpi-data", data, NULL);

  self->moh_enroll = FALSE;
  g_clear_pointer (&self->moh_views, g_ptr_array_unref);
  fpi_device_enroll_complete (device, g_object_ref (print), NULL);
}

static void
enroll_capture_done (FpDevice *device, FpmFeatures *probe)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  gint quality;
  gdouble coverage;
  GError *retry = NULL;

  validity_moh_quality (self->moh_img->data, self->moh_w, self->moh_h, &quality, &coverage);
  fp_dbg ("Enroll capture: quality %d, coverage %.2f, %d keypoints", quality, coverage, probe->n);

  if (quality < MOH_ENROLL_MIN_SCORE || coverage < MOH_ENROLL_MIN_COVER ||
      probe->n < MOH_MIN_KEYPOINTS)
    {
      retry = fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL);
    }
  else
    {
      for (guint i = 0; i < self->moh_views->len && !retry; i++)
        if (fpm_score (probe, g_ptr_array_index (self->moh_views, i)) > MOH_DUPLICATE_SCORE)
          retry = fpi_device_retry_new (FP_DEVICE_RETRY_REMOVE_FINGER);
    }

  if (retry)
    {
      fpm_features_clear (probe);
      g_free (probe);
      fpi_device_enroll_progress (device, self->moh_views->len, NULL, retry);
    }
  else
    {
      g_ptr_array_add (self->moh_views, probe);
      if (self->moh_views->len >= VALIDITY_MOH_ENROLL_STAGES)
        {
          enroll_finish (device);
          return;
        }
      fpi_device_enroll_progress (device, self->moh_views->len, NULL, NULL);
    }
  validity_verify_run_capture (device);
}

void
validity_moh_capture_done (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpmFeatures *probe = g_new0 (FpmFeatures, 1);

  if (self->moh_img == NULL || self->moh_img->len < self->moh_w * self->moh_h)
    {
      /* The state machine only ends without an image if it was cut short. */
      g_free (probe);
      validity_moh_capture_failed (device, fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }
  fpm_extract (self->moh_img->data, self->moh_w, self->moh_h, MOH_MAX_KEYPOINTS, probe);
  self->moh_need_lift = TRUE;

  if (self->moh_enroll)
    {
      enroll_capture_done (device, probe);
      validity_moh_image_reset (self);
      return;
    }

  {
    gint quality;
    gdouble coverage;

    validity_moh_quality (self->moh_img->data, self->moh_w, self->moh_h, &quality, &coverage);
    if (quality < MOH_VERIFY_MIN_SCORE || coverage < MOH_VERIFY_MIN_COVER)
      {
        GError *retry = fpi_device_retry_new (coverage < MOH_VERIFY_MIN_COVER ?
                                              FP_DEVICE_RETRY_CENTER_FINGER :
                                              FP_DEVICE_RETRY_GENERAL);

        fp_info ("Verify capture unusable (quality %d, coverage %.2f): asking for another touch",
                 quality, coverage);
        if (self->identify_mode)
          {
            fpi_device_identify_report (device, NULL, NULL, retry);
            fpi_device_identify_complete (device, NULL);
          }
        else
          {
            fpi_device_verify_report (device, FPI_MATCH_ERROR, NULL, retry);
            fpi_device_verify_complete (device, NULL);
          }
        fpm_features_clear (probe);
        g_free (probe);
        validity_moh_image_reset (self);
        return;
      }
  }

  if (self->identify_mode)
    {
      GPtrArray *gallery = NULL;
      FpPrint *found = NULL;

      fpi_device_get_identify_data (device, &gallery);
      for (guint i = 0; gallery && i < gallery->len && !found; i++)
        if (match_print (g_ptr_array_index (gallery, i), probe) >= MOH_MATCH_THRESHOLD)
          found = g_ptr_array_index (gallery, i);
      fp_info ("Identify (host match): %s", found ? "match" : "no match");
      fpi_device_identify_report (device, found, NULL, NULL);
      fpi_device_identify_complete (device, NULL);
    }
  else
    {
      FpPrint *print = NULL;
      gint score;

      fpi_device_get_verify_data (device, &print);
      score = match_print (print, probe);
      fp_info ("Verify (host match): score %d/%d", score, MOH_MATCH_THRESHOLD);
      fpi_device_verify_report (device,
                                score >= MOH_MATCH_THRESHOLD ? FPI_MATCH_SUCCESS : FPI_MATCH_FAIL,
                                NULL, NULL);
      fpi_device_verify_complete (device, NULL);
    }
  fpm_features_clear (probe);
  g_free (probe);
  validity_moh_image_reset (self);
}

void
validity_moh_capture_failed (FpDevice *device,
                             GError   *error)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);

  validity_moh_image_reset (self);
  if (self->moh_enroll)
    {
      self->moh_enroll = FALSE;
      g_clear_pointer (&self->moh_views, g_ptr_array_unref);
      fpi_device_enroll_complete (device, NULL, error);
    }
  else if (self->identify_mode)
    {
      fpi_device_identify_complete (device, error);
    }
  else
    {
      fpi_device_verify_complete (device, error);
    }
}
