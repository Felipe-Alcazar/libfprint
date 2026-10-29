/*
 * Match-on-host support for Validity sensors that send the image to the
 * host instead of matching on the chip (06cb:00a2).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */
#pragma once

#include "validity.h"

#define VALIDITY_MOH_ENROLL_STAGES 20

gboolean validity_moh_is_moh (FpiDeviceValidity *self);

/* Drop any partial image and start a new capture. */
void     validity_moh_image_reset (FpiDeviceValidity *self);

/* Feed one response of the image download command (0x51 00 20 00 00).
 * Returns 1 when the image is complete, 0 when more chunks are needed and
 * -1 when the capture holds no image (no finger, scan error). */
gint     validity_moh_image_add (FpiDeviceValidity *self,
                                 guint16            status,
                                 const guint8      *data,
                                 gsize              len);

/* Before a verify/identify capture: use the calibration stored with the
 * enrolled print, so the image matches the one enrolled. */
void     validity_moh_apply_calibration (FpiDeviceValidity *self,
                                         FpPrint           *print);

/* Start an enrollment: captures run through the verify state machine. */
void     validity_moh_enroll (FpDevice *device);

/* Called when a capture state machine run finished without error. */
void     validity_moh_capture_done (FpDevice *device);

/* Lift check: the calibration-mode command that scans one raw frame at
 * once, and whether a finger is on that frame. */
guint8  *validity_moh_frame_cmd (FpiDeviceValidity *self,
                                 gsize             *cmd_len,
                                 gsize             *frame_len);
gboolean validity_moh_frame_has_finger (FpiDeviceValidity *self,
                                        const guint8      *frame,
                                        gsize              len);

/* Called when a capture state machine run failed. */
void     validity_moh_capture_failed (FpDevice *device,
                                      GError   *error);

/* Image quality, 0..100, and the share of the sensor the finger covers. */
void     validity_moh_quality (const guint8 *img,
                               guint         width,
                               guint         height,
                               gint         *score,
                               gdouble      *coverage);
