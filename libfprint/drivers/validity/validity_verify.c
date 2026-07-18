/*
 * Verify and Identify state machines for Validity/Synaptics VCSFW sensors
 *
 * Implements FpDevice::verify and FpDevice::identify virtual methods.
 *
 * Both operations share the same state machine since the sensor-side
 * flow is nearly identical:
 *   1. LED on
 *   2. Send capture command (IDENTIFY mode)
 *   3. Wait for finger-down interrupt (EP 0x83)
 *   4. Wait for scan-complete interrupt
 *   5. Start match (cmd 0x5E)
 *   6. Wait for match interrupt
 *   7. Get match result (cmd 0x60)
 *   8. Match cleanup (cmd 0x62)
 *   9. LED off
 *   10. Report result
 *
 * The difference between verify and identify is only in how the
 * result is reported to libfprint.
 *
 * Also implements list and delete operations.
 *
 * Reference: python-validity sensor.py Sensor.identify(), Sensor.match_finger()
 *
 * Copyright (C) 2024 libfprint contributors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "validity"

#include "drivers_api.h"
#include "validity_pack.h"
#include "fpi-print.h"
#include "validity.h"
#include "vcsfw_protocol.h"

/* ================================================================
 * Interrupt helpers (shared with enrollment)
 * ================================================================ */

static void
verify_interrupt_cb (FpiUsbTransfer *transfer,
                     FpDevice       *device,
                     gpointer        user_data,
                     GError         *error)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpiSsm *ssm = user_data;
  guint8 int_type;

  if (error)
    {
      if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        {
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_REMOVED));
          g_error_free (error);
          return;
        }
      fpi_ssm_mark_failed (ssm, error);
      return;
    }

  if (transfer->actual_length < 1)
    {
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }

  int_type = transfer->buffer[0];

  if (transfer->actual_length >= 5)
    {
      fp_dbg ("Verify interrupt: type=0x%02x bytes=[%02x %02x %02x %02x %02x] (len=%" G_GSSIZE_FORMAT ")",
              int_type, transfer->buffer[0], transfer->buffer[1],
              transfer->buffer[2], transfer->buffer[3], transfer->buffer[4],
              transfer->actual_length);
    }
  else
    {
      fp_dbg ("Verify interrupt: type=0x%02x (len=%" G_GSSIZE_FORMAT ")",
              int_type, transfer->actual_length);
    }

  /* Match result: type 3 = match found, anything else = no match.
   * On no-match, skip cmd 0x60 (get_match_result) and go to cleanup;
   * the sensor only sends one interrupt per match attempt. */
  if (fpi_ssm_get_cur_state (ssm) == VERIFY_WAIT_MATCH_INT)
    {
      if (int_type == 3)
        {
          fpi_ssm_next_state (ssm);
          return;
        }

      /* No match — clear any stale result and skip to cleanup */
      fp_info ("Match interrupt type=%d: no match", int_type);
      g_clear_pointer (&self->bulk_data, g_free);
      self->bulk_data_len = 0;
      fpi_ssm_jump_to_state (ssm, VERIFY_CLEANUP);
      return;
    }

  /* During finger wait: type 2 = finger down */
  if (fpi_ssm_get_cur_state (ssm) == VERIFY_WAIT_FINGER)
    {
      if (int_type == VALIDITY_INT_FINGER_DOWN)
        {
          fpi_ssm_next_state (ssm);
          return;
        }
      /* 0x969/0xd51 silicon omits the type-2 finger-down and signals the
       * finger with a type-3 capture event. Accept it as finger-down; if it
       * already carries the scan-complete bit, skip VERIFY_WAIT_SCAN_COMPLETE
       * (a fresh read there would hang waiting for a type-3 that already
       * arrived). */
      if (int_type == 3 && self->sensor.device_info &&
          validity_sensor_skips_finger_down (self->sensor.real_sensor_type))
        {
          if (transfer->actual_length >= 3 &&
              (transfer->buffer[2] & VALIDITY_INT_SCAN_COMPLETE))
            fpi_ssm_jump_to_state (ssm, VERIFY_GET_PRG_STATUS);
          else
            fpi_ssm_next_state (ssm);
          return;
        }
      /* type 0 = capture started, expected */
      if (int_type == 0)
        goto read_again;
    }

  /* During scan wait: type 3 with scan_complete bit */
  if (fpi_ssm_get_cur_state (ssm) == VERIFY_WAIT_SCAN_COMPLETE)
    {
      if (int_type == 3 && transfer->actual_length >= 3 &&
          (transfer->buffer[2] & VALIDITY_INT_SCAN_COMPLETE))
        {
          fpi_ssm_next_state (ssm);
          return;
        }
      if (int_type == 3 || int_type == 0)
        goto read_again;
    }

  /* Unexpected, but keep listening */
  fp_dbg ("Ignoring verify interrupt type 0x%02x in state %d",
          int_type, fpi_ssm_get_cur_state (ssm));

read_again:
  {
    FpiUsbTransfer *new_transfer = fpi_usb_transfer_new (device);
    fpi_usb_transfer_fill_interrupt (new_transfer, VALIDITY_EP_INT_IN,
                                     VALIDITY_USB_INT_DATA_SIZE);
    fpi_usb_transfer_submit (new_transfer, 0,
                             fpi_device_get_cancellable (FP_DEVICE (self)),
                             verify_interrupt_cb, ssm);
  }
}

static void
verify_start_interrupt_wait (FpiDeviceValidity *self,
                             FpiSsm            *ssm)
{
  FpiUsbTransfer *transfer;

  transfer = fpi_usb_transfer_new (FP_DEVICE (self));
  fpi_usb_transfer_fill_interrupt (transfer, VALIDITY_EP_INT_IN,
                                   VALIDITY_USB_INT_DATA_SIZE);
  /* Use no timeout (0) for finger-wait and scan-complete states,
   * since these wait for physical user interaction. The action's
   * GCancellable (fpi_device_get_cancellable) aborts the wait on cancel. */
  fpi_usb_transfer_submit (transfer, 0,
                           fpi_device_get_cancellable (FP_DEVICE (self)),
                           verify_interrupt_cb, ssm);
}

/* ================================================================
 * Match result parsing
 *
 * cmd 0x60 response: len(2LE) | dict_data[len]
 * dict_data is a TLV dictionary (python-validity parse_dict()):
 *   while data: tag(2LE) | len(2LE) | value[len]
 * In the match response:
 *   tag 1 → user_dbid(4LE)
 *   tag 3 → subtype(2LE)
 *   tag 4 → hash (variable length)
 * ================================================================ */

/* MatchResult is now ValidityMatchResult in validity_db.h */
typedef ValidityMatchResult MatchResult;

static void
match_result_clear (MatchResult *r)
{
  validity_match_result_clear (r);
}

/**
 * parse_match_result:
 * @data: response payload from cmd 0x60 (after status word)
 * @data_len: length of @data
 * @result: (out): parsed match result
 *
 * Parses the TLV dictionary from a get_match_result (cmd 0x60) response.
 *
 * python-validity reference (sensor.py match_finger()):
 *   rsp = self.parse_dict(rsp)
 *   usrid, subtype, hsh = rsp[1], rsp[3], rsp[4]
 *
 * where parse_dict() is:
 *   while len(x) > 0:
 *       (t, l), x = unpack('<HH', x[:4]), x[4:]
 *       rc[t], x = x[:l], x[l:]
 *
 * Returns: %TRUE if parsing succeeded (result may still be !matched
 *          if the dict was empty), %FALSE on malformed data.
 */
gboolean
validity_parse_match_result (const guint8        *data,
                             gsize                data_len,
                             ValidityMatchResult *result)
{
  FpiByteReader reader;
  guint16 total_len;

  memset (result, 0, sizeof (*result));

  if (data_len < 2)
    return FALSE;

  fpi_byte_reader_init (&reader, data, data_len);

  if (!fpi_byte_reader_get_uint16_le (&reader, &total_len))
    return FALSE;

  gsize remaining = MIN (total_len, data_len - 2);
  const guint8 *dict_data;

  if (!fpi_byte_reader_get_data (&reader, remaining, &dict_data))
    return FALSE;

  /* Parse TLV entries: tag(2LE) | len(2LE) | value[len] */
  FpiByteReader dict_reader;
  fpi_byte_reader_init (&dict_reader, dict_data, remaining);

  while (fpi_byte_reader_get_remaining (&dict_reader) >= 4)
    {
      guint16 tag, entry_len;
      const guint8 *entry_data;

      if (!fpi_byte_reader_get_uint16_le (&dict_reader, &tag))
        break;
      if (!fpi_byte_reader_get_uint16_le (&dict_reader, &entry_len))
        break;
      if (!fpi_byte_reader_get_data (&dict_reader, entry_len, &entry_data))
        break;

      switch (tag)
        {
        case 1: /* user_dbid (4 bytes LE) */
          {
            FpiByteReader entry_reader;
            fpi_byte_reader_init (&entry_reader, entry_data, entry_len);
            if (fpi_byte_reader_get_uint32_le (&entry_reader, &result->user_dbid))
              result->matched = TRUE;
          }
          break;

        case 3: /* subtype (2 bytes LE) */
          {
            FpiByteReader entry_reader;
            fpi_byte_reader_init (&entry_reader, entry_data, entry_len);
            fpi_byte_reader_get_uint16_le (&entry_reader, &result->subtype);
          }
          break;

        case 4: /* hash (variable) */
          if (entry_len > 0)
            {
              result->hash = g_memdup2 (entry_data, entry_len);
              result->hash_len = entry_len;
            }
          break;

        default:
          fp_dbg ("parse_match_result: ignoring unknown tag %u (len=%u)",
                  tag, entry_len);
          break;
        }
    }

  return TRUE;
}

/**
 * validity_find_gallery_match:
 * @gallery: (element-type FpPrint): array of gallery prints
 * @subtype: sensor finger subtype from match result
 *
 * Find the gallery print whose finger matches the given sensor subtype.
 * Falls back to the first gallery entry if no subtype match is found
 * (the sensor confirmed a match; we just can't correlate the subtype).
 *
 * Returns: (nullable): the matching FpPrint, or %NULL if gallery is empty
 */
FpPrint *
validity_find_gallery_match (GPtrArray *gallery,
                             guint16    subtype)
{
  if (!gallery || gallery->len == 0)
    return NULL;

  gint matched_finger = validity_subtype_to_finger (subtype);

  for (guint i = 0; i < gallery->len; i++)
    {
      FpPrint *candidate = g_ptr_array_index (gallery, i);
      if (fp_print_get_finger (candidate) == (FpFinger) matched_finger)
        return candidate;
    }

  /* Fallback: sensor confirmed a match but we can't correlate the subtype */
  return g_ptr_array_index (gallery, 0);
}

/* ================================================================
 * Verify/Identify SSM
 * ================================================================ */

static void
verify_build_capture (FpiSsm            *ssm,
                      FpiDeviceValidity *self)
{
  gsize cmd_len;
  guint8 *cmd = validity_capture_build_cmd_02 (&self->capture,
                                               self->sensor.type_info,
                                               VALIDITY_CAPTURE_IDENTIFY,
                                               &cmd_len);

  if (!cmd)
    {
      fp_warn ("Failed to build identify capture command");
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      return;
    }

  vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
  g_free (cmd);
}

static void
verify_capture_recv (FpiSsm            *ssm,
                     FpiDeviceValidity *self)
{
  if (self->cmd_response_status != VCSFW_STATUS_OK)
    {
      fp_warn ("Capture (identify) failed: status=0x%04x",
               self->cmd_response_status);
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }
  fpi_ssm_next_state (ssm);
}

static void
verify_match_start_recv (FpiSsm            *ssm,
                         FpiDeviceValidity *self)
{
  if (self->cmd_response_status != VCSFW_STATUS_OK)
    {
      fp_warn ("match_finger failed: status=0x%04x",
               self->cmd_response_status);
      /* No match — continue to cleanup */
      fpi_ssm_jump_to_state (ssm, VERIFY_CLEANUP);
      return;
    }
  fpi_ssm_next_state (ssm);
}

static void
verify_get_result_recv (FpiSsm            *ssm,
                        FpiDeviceValidity *self)
{
  if (self->cmd_response_status != VCSFW_STATUS_OK)
    {
      fp_info ("No match found (status=0x%04x)",
               self->cmd_response_status);
      /* Store no-match indicator */
      g_clear_pointer (&self->bulk_data, g_free);
      self->bulk_data_len = 0;
    }
  else if (self->cmd_response_data && self->cmd_response_len > 0)
    {
      /* Store match result for later reporting */
      g_clear_pointer (&self->bulk_data, g_free);
      self->bulk_data = g_memdup2 (self->cmd_response_data,
                                   self->cmd_response_len);
      self->bulk_data_len = self->cmd_response_len;
    }

  fpi_ssm_next_state (ssm);
}

static void
verify_run_state (FpiSsm   *ssm,
                  FpDevice *dev)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case VERIFY_LED_ON:
      self->verify_capture_retries = 0;
      self->verify_recapture = FALSE;
      {
        gsize cmd_len;
        const guint8 *cmd = validity_capture_glow_start_cmd (&cmd_len);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
      }
      break;

    case VERIFY_LED_ON_RECV:
      fpi_ssm_next_state (ssm);
      break;

    case VERIFY_BUILD_CAPTURE:
      verify_build_capture (ssm, self);
      break;

    case VERIFY_CAPTURE_SEND:
      fpi_ssm_next_state (ssm);
      break;

    case VERIFY_CAPTURE_RECV:
      verify_capture_recv (ssm, self);
      break;

    case VERIFY_WAIT_FINGER:
      verify_start_interrupt_wait (self, ssm);
      break;

    case VERIFY_WAIT_SCAN_COMPLETE:
      verify_start_interrupt_wait (self, ssm);
      break;

    case VERIFY_GET_PRG_STATUS:
      {
        const guint8 cmd[] = { 0x51, 0x00, 0x20, 0x00, 0x00 };
        vcsfw_tls_cmd_send (self, ssm, cmd, sizeof (cmd), NULL);
      }
      break;

    case VERIFY_GET_PRG_STATUS_RECV:
      /* Capture status: len(4) | x(2) | y(2) | w1(2) | w2(2) | error(4).
       * A real finger reports non-zero geometry and error==0. An empty
       * "phantom" capture (no finger on the sensor) reports x=0/y=0 and a
       * non-zero error. python-validity raises on this and its identify loop
       * re-captures; mirror that so verify waits for a real finger instead of
       * running a match that can only fail. */
      {
        gboolean empty = TRUE;

        if (self->cmd_response_data && self->cmd_response_len >= 16)
          {
            const guint8 *d = self->cmd_response_data;
            guint16 x   = d[4]  | (d[5] << 8);
            guint16 y   = d[6]  | (d[7] << 8);
            guint16 w1  = d[8]  | (d[9] << 8);
            guint16 w2  = d[10] | (d[11] << 8);
            guint32 err = d[12] | (d[13] << 8) | (d[14] << 16) | ((guint32) d[15] << 24);
            fp_dbg ("Verify capture status: x=%u y=%u w1=%u w2=%u error=0x%08x",
                    x, y, w1, w2, err);
            empty = (err != 0 || x == 0 || y == 0);
          }

        if (empty)
          {
            /* No finger on the sensor yet. Re-arm and wait. Like every other
             * finger-wait in this driver, this is unbounded and terminated by
             * the operation's GCancellable (fprintd's timeout or a user
             * cancel) rather than a fixed driver-side timeout. */
            self->verify_recapture = TRUE;
            fp_dbg ("Verify: empty capture (no finger yet) -- re-arming (attempt %u)",
                    ++self->verify_capture_retries);
          }
        else
          {
            self->verify_capture_retries = 0;
          }
      }
      fpi_ssm_next_state (ssm);
      break;

    case VERIFY_CAPTURE_STOP:
      {
        const guint8 cmd[] = { 0x04 };
        vcsfw_tls_cmd_send (self, ssm, cmd, sizeof (cmd), NULL);
      }
      break;

    case VERIFY_CAPTURE_STOP_RECV:
      /* Cleanup status doesn't matter. If the last capture was empty (no
       * finger), loop back to re-arm; a short delay avoids busy-looping the
       * sensor while waiting for the user to place a finger. */
      if (self->verify_recapture)
        {
          self->verify_recapture = FALSE;
          fpi_ssm_jump_to_state_delayed (ssm, VERIFY_BUILD_CAPTURE, 300);
        }
      else
        {
          fpi_ssm_next_state (ssm);
        }
      break;

    case VERIFY_MATCH_START:
      {
        gsize cmd_len;
        guint8 *cmd = validity_pack_new (&cmd_len, "bbbhhhhh",
                                         VCSFW_CMD_MATCH_FINGER,
                                         0x02, 0xFF, 0, 0, 1, 0, 0);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
        g_free (cmd);
      }
      break;

    case VERIFY_MATCH_START_RECV:
      verify_match_start_recv (ssm, self);
      break;

    case VERIFY_WAIT_MATCH_INT:
      /* Wait for interrupt type 3 indicating match result ready */
      verify_start_interrupt_wait (self, ssm);
      break;

    case VERIFY_GET_RESULT:
      {
        gsize cmd_len;
        guint8 *cmd = validity_pack_new (&cmd_len, "bw",
                                         VCSFW_CMD_GET_MATCH_RESULT,
                                         (guint32) 0);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
        g_free (cmd);
      }
      break;

    case VERIFY_GET_RESULT_RECV:
      verify_get_result_recv (ssm, self);
      break;

    case VERIFY_CLEANUP:
      {
        gsize cmd_len;
        guint8 *cmd = validity_pack_new (&cmd_len, "bw",
                                         VCSFW_CMD_MATCH_CLEANUP,
                                         (guint32) 0);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
        g_free (cmd);
      }
      break;

    case VERIFY_CLEANUP_RECV:
      /* Cleanup status doesn't matter */
      fpi_ssm_next_state (ssm);
      break;

    case VERIFY_LED_OFF:
      {
        gsize cmd_len;
        const guint8 *cmd = validity_capture_glow_end_cmd (&cmd_len);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
      }
      break;

    case VERIFY_LED_OFF_RECV:
      fpi_ssm_next_state (ssm);
      break;

    case VERIFY_DONE:
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
verify_ssm_done (FpiSsm   *ssm,
                 FpDevice *dev,
                 GError   *error)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  if (error)
    {
      error = validity_map_removed_error (error);
      if (self->identify_mode)
        fpi_device_identify_complete (dev, error);
      else
        fpi_device_verify_complete (dev, error);

      g_clear_pointer (&self->bulk_data, g_free);
      self->bulk_data_len = 0;
      return;
    }

  /* Parse stored match result */
  MatchResult match = { 0 };
  gboolean have_match = FALSE;

  if (self->bulk_data && self->bulk_data_len > 0)
    if (validity_parse_match_result (self->bulk_data, self->bulk_data_len, &match))
      have_match = match.matched;

  if (self->identify_mode)
    {
      if (have_match)
        {
          fp_info ("Identify matched: user_dbid=%u subtype=%u",
                   match.user_dbid, match.subtype);

          /* Match the sensor result against the gallery by comparing
           * the finger subtype. The sensor does the actual 1:N match
           * internally; we just need to find which gallery FpPrint
           * corresponds to the matched subtype. */
          GPtrArray *gallery = NULL;

          fpi_device_get_identify_data (dev, &gallery);

          FpPrint *gallery_match = validity_find_gallery_match (
            gallery, match.subtype);

          fpi_device_identify_report (dev, gallery_match, NULL, NULL);
        }
      else
        {
          fpi_device_identify_report (dev, NULL, NULL, NULL);
        }

      fpi_device_identify_complete (dev, NULL);
    }
  else
    {
      /* Verify mode */
      if (have_match)
        {
          fp_info ("Verify matched: user_dbid=%u", match.user_dbid);
          fpi_device_verify_report (dev, FPI_MATCH_SUCCESS, NULL, NULL);
        }
      else
        {
          fp_info ("Verify: no match");
          fpi_device_verify_report (dev, FPI_MATCH_FAIL, NULL, NULL);
        }

      fpi_device_verify_complete (dev, NULL);
    }

  match_result_clear (&match);
  g_clear_pointer (&self->bulk_data, g_free);
  self->bulk_data_len = 0;
}

/* ================================================================
 * Emulation mode: virtual verify / identify / list / delete / clear
 * ================================================================ */

static void
emulation_verify (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);

  if (self->emulation_prints->len > 0)
    fpi_device_verify_report (device, FPI_MATCH_SUCCESS, NULL, NULL);
  else
    fpi_device_verify_report (device, FPI_MATCH_FAIL, NULL, NULL);

  fpi_device_verify_complete (device, NULL);
}

static void
emulation_identify (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  GPtrArray *gallery = NULL;

  fpi_device_get_identify_data (device, &gallery);

  if (self->emulation_prints->len > 0 && gallery && gallery->len > 0)
    {
      fpi_device_identify_report (device,
                                  g_ptr_array_index (gallery, 0),
                                  g_ptr_array_index (self->emulation_prints, 0),
                                  NULL);
    }
  else
    {
      fpi_device_identify_report (device, NULL, NULL, NULL);
    }

  fpi_device_identify_complete (device, NULL);
}

static void
emulation_list (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  GPtrArray *prints = g_ptr_array_new_with_free_func (g_object_unref);

  for (guint i = 0; i < self->emulation_prints->len; i++)
    g_ptr_array_add (prints,
                     g_object_ref (g_ptr_array_index (self->emulation_prints, i)));

  fpi_device_list_complete (device, prints, NULL);
}

static void
emulation_delete (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpPrint *print = NULL;

  fpi_device_get_delete_data (device, &print);

  FpFinger target_finger = fp_print_get_finger (print);

  for (guint i = 0; i < self->emulation_prints->len; i++)
    {
      FpPrint *stored = g_ptr_array_index (self->emulation_prints, i);

      if (fp_print_get_finger (stored) == target_finger)
        {
          g_ptr_array_remove_index (self->emulation_prints, i);
          break;
        }
    }

  fpi_device_delete_complete (device, NULL);
}

static void
emulation_clear_storage (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);

  g_ptr_array_set_size (self->emulation_prints, 0);
  fpi_device_clear_storage_complete (device, NULL);
}

void
validity_verify (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpiSsm *ssm;

  G_DEBUG_HERE ();

  if (self->emulation_prints)
    {
      emulation_verify (device);
      return;
    }

  self->identify_mode = FALSE;

  ssm = fpi_ssm_new (device, verify_run_state, VERIFY_NUM_STATES);
  fpi_ssm_start (ssm, verify_ssm_done);
}

void
validity_identify (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpiSsm *ssm;

  G_DEBUG_HERE ();

  if (self->emulation_prints)
    {
      emulation_identify (device);
      return;
    }

  self->identify_mode = TRUE;

  ssm = fpi_ssm_new (device, verify_run_state, VERIFY_NUM_STATES);
  fpi_ssm_start (ssm, verify_ssm_done);
}

/* ================================================================
 * List prints — enumerate enrolled fingerprints from sensor DB
 * ================================================================ */

static void
list_get_storage (FpiSsm            *ssm,
                  FpiDeviceValidity *self)
{
  gsize cmd_len;
  guint8 *cmd = validity_db_build_cmd_get_user_storage (
    VALIDITY_STORAGE_NAME, &cmd_len);

  self->list_user_idx = 0;
  memset (&self->list_storage, 0, sizeof (self->list_storage));
  vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
  g_free (cmd);
}

static void
list_get_storage_recv (FpiSsm            *ssm,
                       FpiDeviceValidity *self)
{
  if (self->cmd_response_status != VCSFW_STATUS_OK)
    {
      fp_info ("No user storage found (status=0x%04x)",
               self->cmd_response_status);
      fpi_ssm_jump_to_state (ssm, LIST_DONE);
      return;
    }

  if (!self->cmd_response_data ||
      !validity_db_parse_user_storage (self->cmd_response_data,
                                       self->cmd_response_len,
                                       &self->list_storage))
    {
      fp_info ("Failed to parse user storage — no enrolled prints");
      fpi_ssm_jump_to_state (ssm, LIST_DONE);
      return;
    }

  fp_info ("Storage '%s': %u users",
           self->list_storage.name ? self->list_storage.name : "",
           self->list_storage.user_count);

  if (self->list_storage.user_count == 0)
    {
      fpi_ssm_jump_to_state (ssm, LIST_DONE);
      return;
    }

  self->list_user_idx = 0;
  fpi_ssm_next_state (ssm);
}

static void
list_get_user (FpiSsm            *ssm,
               FpiDeviceValidity *self)
{
  if (self->list_user_idx >= self->list_storage.user_count)
    {
      fpi_ssm_jump_to_state (ssm, LIST_DONE);
      return;
    }

  guint16 user_dbid = self->list_storage.user_dbids[self->list_user_idx];

  gsize cmd_len;
  guint8 *cmd = validity_db_build_cmd_get_user (user_dbid, &cmd_len);
  vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
  g_free (cmd);
}

static void
list_get_user_recv (FpiSsm            *ssm,
                    FpiDeviceValidity *self)
{
  FpDevice *dev = FP_DEVICE (self);
  GPtrArray *prints_array = fpi_ssm_get_data (ssm);

  if (self->cmd_response_status == VCSFW_STATUS_OK &&
      self->cmd_response_data)
    {
      ValidityUser user = { 0 };

      if (validity_db_parse_user (self->cmd_response_data,
                                  self->cmd_response_len,
                                  &user))
        {
          for (guint16 i = 0; i < user.finger_count; i++)
            {
              FpPrint *print = fp_print_new (dev);
              gint finger = validity_subtype_to_finger (
                user.fingers[i].subtype);

              fpi_print_set_type (print, FPI_PRINT_RAW);
              fpi_print_set_device_stored (print, TRUE);
              if (finger >= 0)
                fp_print_set_finger (print, (FpFinger) finger);

              /* Set fpi-data to the identity id string, matching what
               * enroll stores (g_variant_new_string(user_id)), so the print
               * serialises and fp_print_equal() can match it against a saved
               * print. Identity blob: type(4LE) | len(4LE) | id_bytes[len]. */
              if (user.identity && user.identity_len >= 8)
                {
                  FpiByteReader idr;
                  guint32 id_type = 0, id_len = 0;
                  const guint8 *id_bytes = NULL;

                  fpi_byte_reader_init (&idr, user.identity, user.identity_len);
                  if (fpi_byte_reader_get_uint32_le (&idr, &id_type) &&
                      fpi_byte_reader_get_uint32_le (&idr, &id_len) &&
                      id_len > 0 &&
                      fpi_byte_reader_get_data (&idr, id_len, &id_bytes))
                    {
                      g_autofree gchar *id =
                        g_strndup ((const gchar *) id_bytes, id_len);
                      /* The id comes straight off the device. g_variant_new_string
                       * aborts (g_critical) on non-UTF-8 input, so validate first;
                       * a print with unset fpi-data simply won't match, which is the
                       * right outcome for a garbage record. */
                      if (g_utf8_validate (id, -1, NULL))
                        g_object_set (print, "fpi-data",
                                      g_variant_new_string (id), NULL);
                      else
                        fp_warn ("Skipping non-UTF-8 identity in stored print");
                    }
                }

              g_ptr_array_add (prints_array, print);
            }

          validity_user_clear (&user);
        }
    }

  self->list_user_idx++;

  if (self->list_user_idx < self->list_storage.user_count)
    fpi_ssm_jump_to_state (ssm, LIST_GET_USER);
  else
    fpi_ssm_next_state (ssm);
}

static void
list_run_state (FpiSsm   *ssm,
                FpDevice *dev)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case LIST_GET_STORAGE:
      list_get_storage (ssm, self);
      break;

    case LIST_GET_STORAGE_RECV:
      list_get_storage_recv (ssm, self);
      break;

    case LIST_GET_USER:
      list_get_user (ssm, self);
      break;

    case LIST_GET_USER_RECV:
      list_get_user_recv (ssm, self);
      break;

    case LIST_DONE:
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
list_ssm_done (FpiSsm   *ssm,
               FpDevice *dev,
               GError   *error)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);
  GPtrArray *prints_array = fpi_ssm_get_data (ssm);

  validity_user_storage_clear (&self->list_storage);

  if (error)
    {
      fpi_device_list_complete (dev, NULL, validity_map_removed_error (error));
      return;
    }

  /* The ssm still owns a reference via its g_ptr_array_unref destroy notify
   * (set with fpi_ssm_set_data), so hand list_complete its own reference --
   * otherwise the array is unreffed twice and freed while the caller still
   * iterates it (use-after-free). */
  fpi_device_list_complete (dev, g_ptr_array_ref (prints_array), NULL);
}

void
validity_list (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpiSsm *ssm;
  GPtrArray *prints_array;

  G_DEBUG_HERE ();

  if (self->emulation_prints)
    {
      emulation_list (device);
      return;
    }

  prints_array = g_ptr_array_new_with_free_func (g_object_unref);

  ssm = fpi_ssm_new (device, list_run_state, LIST_NUM_STATES);
  fpi_ssm_set_data (ssm, prints_array, (GDestroyNotify) g_ptr_array_unref);
  fpi_ssm_start (ssm, list_ssm_done);
}

/* ================================================================
 * Delete print — remove a fingerprint record from the sensor DB
 * ================================================================ */


static void
delete_get_storage (FpiSsm            *ssm,
                    FpiDeviceValidity *self)
{
  gsize cmd_len;
  guint8 *cmd = validity_db_build_cmd_get_user_storage (
    VALIDITY_STORAGE_NAME, &cmd_len);

  vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
  g_free (cmd);
}

static void
delete_get_storage_recv (FpiSsm            *ssm,
                         FpiDeviceValidity *self)
{
  FpDevice *dev = FP_DEVICE (self);

  if (self->cmd_response_status != VCSFW_STATUS_OK)
    {
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
      return;
    }

  /* Parse into list_storage (shared with list SSM, not concurrent) */
  validity_user_storage_clear (&self->list_storage);
  if (!self->cmd_response_data ||
      !validity_db_parse_user_storage (self->cmd_response_data,
                                       self->cmd_response_len,
                                       &self->list_storage))
    {
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
      return;
    }

  self->delete_storage_dbid = self->list_storage.dbid;

  /* Extract finger subtype from the print to delete */
  {
    FpPrint *print = NULL;
    fpi_device_get_delete_data (dev, &print);

    FpFinger finger = fp_print_get_finger (print);
    self->delete_finger_subtype = validity_finger_to_subtype (finger);
  }

  self->list_user_idx = 0;
  fpi_ssm_next_state (ssm);
}

static void
delete_lookup_user (FpiSsm            *ssm,
                    FpiDeviceValidity *self)
{
  /* Look up the user matching the print to delete.
   * Iterate users to find one with a matching finger subtype.
   * python-validity: db.lookup_user(identity) */
  if (self->list_user_idx >= self->list_storage.user_count)
    {
      /* No matching finger found across all users */
      fp_info ("Delete: no matching finger (subtype=%u) found in DB",
               self->delete_finger_subtype);
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
      return;
    }

  {
    guint16 user_dbid = self->list_storage.user_dbids[self->list_user_idx];
    gsize cmd_len;
    guint8 *cmd = validity_db_build_cmd_get_user (user_dbid, &cmd_len);
    vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
    g_free (cmd);
  }
}

static void
delete_lookup_user_recv (FpiSsm            *ssm,
                         FpiDeviceValidity *self)
{
  /* Parse user and look for the finger to delete */
  if (self->cmd_response_status == VCSFW_STATUS_OK &&
      self->cmd_response_data)
    {
      ValidityUser user = { 0 };
      if (validity_db_parse_user (self->cmd_response_data,
                                  self->cmd_response_len,
                                  &user))
        {
          for (guint16 i = 0; i < user.finger_count; i++)
            {
              if (user.fingers[i].subtype == self->delete_finger_subtype)
                {
                  /* Found matching finger — store dbid for deletion */
                  self->delete_finger_dbid = user.fingers[i].dbid;
                  validity_user_clear (&user);
                  fpi_ssm_next_state (ssm);
                  return;
                }
            }
          validity_user_clear (&user);
        }
    }

  /* Try next user — jump back to DELETE_LOOKUP_USER */
  self->list_user_idx++;
  fpi_ssm_jump_to_state (ssm, DELETE_LOOKUP_USER);
}

static void
delete_del_record (FpiSsm            *ssm,
                   FpiDeviceValidity *self)
{
  /* Delete the finger record via cmd 0x48
   * python-validity: db.del_record(dbid) */
  gsize cmd_len;
  guint8 *cmd = validity_db_build_cmd_del_record (
    self->delete_finger_dbid, &cmd_len);

  vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
  g_free (cmd);
}

static void
delete_del_record_recv (FpiSsm            *ssm,
                        FpiDeviceValidity *self)
{
  if (self->cmd_response_status != VCSFW_STATUS_OK)
    {
      fp_warn ("del_record failed: status=0x%04x",
               self->cmd_response_status);
      fpi_ssm_mark_failed (ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }

  fp_info ("Deleted finger record: dbid=%u", self->delete_finger_dbid);
  fpi_ssm_next_state (ssm);
}

static void
delete_run_state (FpiSsm   *ssm,
                  FpDevice *dev)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case DELETE_GET_STORAGE:
      delete_get_storage (ssm, self);
      break;

    case DELETE_GET_STORAGE_RECV:
      delete_get_storage_recv (ssm, self);
      break;

    case DELETE_LOOKUP_USER:
      delete_lookup_user (ssm, self);
      break;

    case DELETE_LOOKUP_USER_RECV:
      delete_lookup_user_recv (ssm, self);
      break;

    case DELETE_DEL_RECORD:
      delete_del_record (ssm, self);
      break;

    case DELETE_DEL_RECORD_RECV:
      delete_del_record_recv (ssm, self);
      break;

    case DELETE_DONE:
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
delete_ssm_done (FpiSsm   *ssm,
                 FpDevice *dev,
                 GError   *error)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  validity_user_storage_clear (&self->list_storage);
  fpi_device_delete_complete (dev, validity_map_removed_error (error));
}

void
validity_delete (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpiSsm *ssm;

  G_DEBUG_HERE ();

  if (self->emulation_prints)
    {
      emulation_delete (device);
      return;
    }

  ssm = fpi_ssm_new (device, delete_run_state, DELETE_NUM_STATES);
  fpi_ssm_start (ssm, delete_ssm_done);
}

/* ================================================================
 * Clear storage — delete all fingerprint records from the sensor DB
 * python-validity: for user in db.get_user_storage(): db.del_record(user.dbid)
 * ================================================================ */

static void
clear_run_state (FpiSsm   *ssm,
                 FpDevice *dev)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CLEAR_DB_WRITE_ENABLE:
      {
        gsize blob_len;
        const guint8 *blob = validity_db_get_write_enable_blob (self, &blob_len);
        vcsfw_tls_cmd_send (self, ssm, blob, blob_len, NULL);
      }
      break;

    case CLEAR_DB_WRITE_ENABLE_RECV:
      if (self->cmd_response_status != VCSFW_STATUS_OK)
        {
          fp_warn ("clear_storage: db_write_enable failed: 0x%04x",
                   self->cmd_response_status);
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
          break;
        }
      fpi_ssm_next_state (ssm);
      break;

    case CLEAR_ERASE:
      {
        gsize cmd_len;
        guint8 *cmd = validity_db_build_cmd_erase_flash (VALIDITY_DB_PARTITION,
                                                         &cmd_len);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
        g_free (cmd);
      }
      break;

    case CLEAR_ERASE_RECV:
      if (self->cmd_response_status != VCSFW_STATUS_OK)
        {
          fp_warn ("clear_storage: erase_flash(%d) failed: 0x%04x",
                   VALIDITY_DB_PARTITION, self->cmd_response_status);
          fpi_ssm_mark_failed (ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
          break;
        }
      fp_info ("clear_storage: erased DB partition %d (reclaimed flash)",
               VALIDITY_DB_PARTITION);
      fpi_ssm_next_state (ssm);
      break;

    case CLEAR_CLEANUPS:
      {
        gsize cmd_len;
        guint8 *cmd = validity_db_build_cmd_call_cleanups (&cmd_len);
        vcsfw_tls_cmd_send (self, ssm, cmd, cmd_len, NULL);
        g_free (cmd);
      }
      break;

    case CLEAR_CLEANUPS_RECV:
      fpi_ssm_next_state (ssm);
      break;

    case CLEAR_DONE:
      fpi_ssm_mark_completed (ssm);
      break;
    }
}

static void
clear_ssm_done (FpiSsm   *ssm,
                FpDevice *dev,
                GError   *error)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (dev);

  validity_user_storage_clear (&self->list_storage);
  fpi_device_clear_storage_complete (dev, validity_map_removed_error (error));
}

void
validity_clear_storage (FpDevice *device)
{
  FpiDeviceValidity *self = FPI_DEVICE_VALIDITY (device);
  FpiSsm *ssm;

  G_DEBUG_HERE ();

  if (self->emulation_prints)
    {
      emulation_clear_storage (device);
      return;
    }

  ssm = fpi_ssm_new (device, clear_run_state, CLEAR_NUM_STATES);
  fpi_ssm_start (ssm, clear_ssm_done);
}
