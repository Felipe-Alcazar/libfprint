/*
 * Validity/Synaptics VCSFW fingerprint sensor driver
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

#pragma once

#include "fpi-device.h"
#include "fpi-ssm.h"
#include "validity_capture.h"
#include "validity_data.h"
#include "validity_db.h"
#include "validity_pair.h"
#include "validity_sensor.h"
#include "validity_tls.h"

/* USB Endpoint addresses */
#define VALIDITY_EP_CMD_OUT 0x01
#define VALIDITY_EP_CMD_IN 0x81
#define VALIDITY_EP_DATA_IN 0x82
#define VALIDITY_EP_INT_IN 0x83

/* USB transfer parameters */
#define VALIDITY_USB_TIMEOUT 15000
#define VALIDITY_USB_INT_TIMEOUT 100
#define VALIDITY_MAX_TRANSFER_LEN (100 * 1024)
#define VALIDITY_USB_INT_DATA_SIZE 1024
#define VALIDITY_USB_SEND_HEADER_LEN 1

/* Number of enroll stages */
#define VALIDITY_ENROLL_STAGES 8

/* Interrupt response bits */
#define VALIDITY_INT_FINGER_DOWN 0x02
#define VALIDITY_INT_SCAN_COMPLETE 0x04

typedef enum {
  VALIDITY_DEV_90 = 0,  /* 138a:0090 */
  VALIDITY_DEV_97,      /* 138a:0097 */
  VALIDITY_DEV_9A,      /* 06cb:009a */
  VALIDITY_DEV_9D,      /* 138a:009d */
  VALIDITY_DEV_AB,      /* 138a:00ab (HP ZBook Studio x360 G5) */
  VALIDITY_DEV_CB,      /* 06cb:00cb (HP Pavilion x360 14-dh) */
  VALIDITY_DEV_B7,      /* 06cb:00b7 (HP G6 series) — UNTESTED, from python-validity */
} ValidityDeviceType;

/* Firmware version info from GET_VERSION (cmd 0x01) */
typedef struct
{
  guint32 build_time;
  guint32 build_num;
  guint8  version_major;
  guint8  version_minor;
  guint8  target;
  guint8  product;
  guint8  silicon_rev;
  guint8  formal_release;
  guint8  platform;
  guint8  patch;
  guint8  serial_number[6];
  guint16 security;
  guint8  iface;
  guint8  device_type;
} ValidityVersionInfo;

/* Open SSM states */
typedef enum {
  VALIDITY_OPEN_CLAIM_INTERFACE = 0,
  VALIDITY_OPEN_SEND_INIT,
  VALIDITY_OPEN_NUM_STATES,
} ValidityOpenState;

/* Close SSM states */
typedef enum {
  VALIDITY_CLOSE_RELEASE_INTERFACE = 0,
  VALIDITY_CLOSE_NUM_STATES,
} ValidityCloseState;

/* Calibration SSM states (runs during open, after capture_setup) */
typedef enum {
  CALIB_BUILD_CMD = 0,
  CALIB_SEND_CMD,
  CALIB_SEND_CMD_RECV,
  CALIB_READ_DATA,
  CALIB_AVERAGE_FRAMES,
  CALIB_PROCESS,
  CALIB_LOOP_CHECK,
  CALIB_BUILD_CLEAN_SLATE_CMD,
  CALIB_CLEAN_SLATE_SEND,
  CALIB_CLEAN_SLATE_RECV,
  CALIB_CLEAN_SLATE_READ,
  CALIB_SAVE_CLEAN_SLATE,
  CALIB_DONE,
  CALIB_NUM_STATES,
} ValidityCalibState;

/* Enrollment SSM states — matches python-validity sensor.py Sensor.enroll() */
typedef enum {
  ENROLL_CLEANUP_STALE = 0,    /* Close any stale enrollment session */
  ENROLL_CLEANUP_STALE_RECV,
  /* Pre-enrollment: delete existing user records to avoid 0x0526 */
  ENROLL_PRE_GET_STORAGE,
  ENROLL_PRE_GET_STORAGE_RECV,
  ENROLL_PRE_DEL_USER,
  ENROLL_PRE_DEL_USER_RECV,
  ENROLL_START,                /* create_enrollment (cmd 0x69 flag=1) */
  ENROLL_START_RECV,
  /* --- Per-iteration loop --- */
  ENROLL_LED_ON,
  ENROLL_LED_ON_RECV,
  ENROLL_WAIT_FINGER_DELAY,
  ENROLL_BUILD_CAPTURE,
  ENROLL_CAPTURE_SEND,
  ENROLL_CAPTURE_RECV,
  ENROLL_WAIT_FINGER,
  ENROLL_WAIT_SCAN_COMPLETE,
  ENROLL_GET_PRG_STATUS,
  ENROLL_GET_PRG_STATUS_RECV,
  ENROLL_CAPTURE_STOP,
  ENROLL_CAPTURE_STOP_RECV,
  ENROLL_UPDATE_START,
  ENROLL_UPDATE_START_RECV,
  ENROLL_WAIT_UPDATE_START_INT,  /* PY: usb.wait_int() inside enrollment_update_start */
  ENROLL_DB_WRITE_ENABLE,        /* PY: write_enable() before 1st enrollment_update */
  ENROLL_DB_WRITE_ENABLE_RECV,
  ENROLL_APPEND_IMAGE,           /* 1st enrollment_update (trigger) */
  ENROLL_APPEND_IMAGE_RECV,
  ENROLL_CLEANUPS,               /* PY: call_cleanups() after 1st enrollment_update */
  ENROLL_CLEANUPS_RECV,
  ENROLL_WAIT_UPDATE_INT,        /* PY: usb.wait_int() between the two calls */
  ENROLL_DB_WRITE_ENABLE_READ,   /* PY: write_enable() before 2nd enrollment_update */
  ENROLL_DB_WRITE_ENABLE_READ_RECV,
  ENROLL_APPEND_IMAGE_READ,      /* 2nd enrollment_update (read result) */
  ENROLL_APPEND_IMAGE_READ_RECV,
  ENROLL_CLEANUPS_READ,          /* PY: call_cleanups() after 2nd enrollment_update */
  ENROLL_CLEANUPS_READ_RECV,
  ENROLL_UPDATE_END,             /* PY: enrollment_update_end = cmd 0x69 flag=0 (finally) */
  ENROLL_UPDATE_END_RECV,
  ENROLL_LOOP_CHECK,
  /* --- Post-loop: DB commit --- */
  ENROLL_UPDATE_END2,            /* PY: 2nd enrollment_update_end after loop */
  ENROLL_UPDATE_END2_RECV,
  ENROLL_GET_STORAGE,
  ENROLL_GET_STORAGE_RECV,
  /* If storage doesn't exist (0x04b3), create it: */
  ENROLL_INIT_STORAGE_WE,       /* db_write_enable for storage creation */
  ENROLL_INIT_STORAGE_WE_RECV,
  ENROLL_INIT_STORAGE_CREATE,   /* new_record(1, 4, 3, "StgWindsor\0") */
  ENROLL_INIT_STORAGE_CREATE_RECV,
  ENROLL_INIT_STORAGE_CLEAN,    /* call_cleanups */
  ENROLL_INIT_STORAGE_CLEAN_RECV,
  ENROLL_DB_WRITE_ENABLE2,
  ENROLL_DB_WRITE_ENABLE2_RECV,
  ENROLL_CREATE_USER,
  ENROLL_CREATE_USER_RECV,
  ENROLL_CREATE_USER_CLEANUPS,
  ENROLL_CREATE_USER_CLEANUPS_RECV,
  ENROLL_DB_WRITE_ENABLE3,
  ENROLL_DB_WRITE_ENABLE3_RECV,
  ENROLL_CREATE_FINGER,
  ENROLL_CREATE_FINGER_RECV,
  ENROLL_FINAL_CLEANUPS,
  ENROLL_FINAL_CLEANUPS_RECV,
  ENROLL_WAIT_FINGER_INT,
  ENROLL_LED_OFF,                /* PY: glow_end_scan() — LAST step per PY */
  ENROLL_LED_OFF_RECV,
  ENROLL_DONE,
  ENROLL_NUM_STATES,
} ValidityEnrollState;

/* Verify/Identify SSM states */
typedef enum {
  VERIFY_LED_ON = 0,
  VERIFY_LED_ON_RECV,
  VERIFY_BUILD_CAPTURE,
  VERIFY_CAPTURE_SEND,
  VERIFY_CAPTURE_RECV,
  VERIFY_WAIT_FINGER,
  VERIFY_WAIT_SCAN_COMPLETE,
  VERIFY_GET_PRG_STATUS,
  VERIFY_GET_PRG_STATUS_RECV,
  VERIFY_CAPTURE_STOP,
  VERIFY_CAPTURE_STOP_RECV,
  VERIFY_MATCH_START,
  VERIFY_MATCH_START_RECV,
  VERIFY_WAIT_MATCH_INT,
  VERIFY_GET_RESULT,
  VERIFY_GET_RESULT_RECV,
  VERIFY_CLEANUP,
  VERIFY_CLEANUP_RECV,
  VERIFY_LED_OFF,
  VERIFY_LED_OFF_RECV,
  VERIFY_DONE,
  VERIFY_NUM_STATES,
} ValidityVerifyState;

/* List prints SSM states */
typedef enum {
  LIST_GET_STORAGE = 0,
  LIST_GET_STORAGE_RECV,
  LIST_GET_USER,
  LIST_GET_USER_RECV,
  LIST_DONE,
  LIST_NUM_STATES,
} ValidityListState;

/* Delete print SSM states */
typedef enum {
  DELETE_GET_STORAGE = 0,
  DELETE_GET_STORAGE_RECV,
  DELETE_LOOKUP_USER,
  DELETE_LOOKUP_USER_RECV,
  DELETE_DEL_RECORD,
  DELETE_DEL_RECORD_RECV,
  DELETE_DONE,
  DELETE_NUM_STATES,
} ValidityDeleteState;

/* Clear storage SSM states */
typedef enum {
  CLEAR_GET_STORAGE = 0,
  CLEAR_GET_STORAGE_RECV,
  CLEAR_DEL_USER,
  CLEAR_DEL_USER_RECV,
  CLEAR_DONE,
  CLEAR_NUM_STATES,
} ValidityClearState;

#define FPI_TYPE_DEVICE_VALIDITY (fpi_device_validity_get_type ())
G_DECLARE_FINAL_TYPE (FpiDeviceValidity, fpi_device_validity,
                      FPI, DEVICE_VALIDITY, FpDevice)

struct _FpiDeviceValidity
{
  FpDevice            parent;

  ValidityDeviceType  dev_type;
  ValidityVersionInfo version_info;
  GCancellable       *interrupt_cancellable;

  /* Runtime data files loaded from libfprint-validity-data package */
  ValidityDataStore device_data;
  ValidityDataStore common_data;

  /* TLS session state */
  ValidityTlsState tls;

  /* Sensor identification and HAL state (post-TLS) */
  ValiditySensorState sensor;

  /* Capture program infrastructure and calibration state */
  ValidityCaptureState capture;

  /* Firmware extension status */
  gboolean fwext_loaded;

  /* Pairing state (for uninitialized devices) */
  ValidityPairState pair_state;

  /* Calibration state */
  gboolean calibrated;
  guint    calib_iteration;

  /* Enrollment state */
  guint32 enroll_key;
  guint8 *enroll_template;
  gsize   enroll_template_len;
  guint   enroll_stage;
  guint16 enroll_user_dbid;
  guint16 enroll_storage_dbid;
  guint   scan_incomplete_count;
  gboolean enroll_frame_rejected;   /* sensor rejected the last frame (0x0526) — retry stage */

  /* Verify: retry state for empty/phantom captures (no finger down) */
  guint    verify_capture_retries;
  gboolean verify_recapture;

  /* Verify/identify mode flag: TRUE = identify, FALSE = verify */
  gboolean identify_mode;

  /* List prints state */
  ValidityUserStorage list_storage;
  guint               list_user_idx;

  /* Delete state */
  guint16 delete_storage_dbid;
  guint16 delete_finger_subtype;
  guint16 delete_finger_dbid;

  /* Command SSM: manages the send-cmd/recv-response cycle */
  FpiSsm *cmd_ssm;

  /* Parent SSM: back-pointer for non-subsm child SSMs */
  FpiSsm *open_ssm;

  /* Pending response data stashed for higher-level SSM consumption */
  guint16 cmd_response_status;
  guint8 *cmd_response_data;
  gsize   cmd_response_len;

  /* Bulk data buffer (EP 0x82 reads during capture/calibration) */
  guint8 *bulk_data;
  gsize   bulk_data_len;

  /* Emulation mode: in-memory print storage (element-type FpPrint) */
  GPtrArray *emulation_prints;
};

/* Enrollment SSM (validity_enroll.c) */
void validity_enroll (FpDevice *device);

/* Enrollment response parsing — exposed for unit testing */
#define ENROLLMENT_MAGIC_LEN 0x38

typedef struct
{
  guint8 *header;
  gsize   header_len;
  guint8 *template_data;
  gsize   template_len;
  guint8 *tid;
  gsize   tid_len;
} EnrollmentUpdateResult;

void     enrollment_update_result_clear (EnrollmentUpdateResult *r);
gboolean parse_enrollment_update_response (const guint8           *data,
                                           gsize                   data_len,
                                           EnrollmentUpdateResult *result);

/* Verify/Identify SSMs (validity_verify.c) */
/* Map a USB "no device" error (intermittent re-enumeration) to
 * FP_DEVICE_ERROR_REMOVED. Takes ownership of @error, returns the result. */
GError *validity_map_removed_error (GError *error);

void validity_verify (FpDevice *device);
void validity_identify (FpDevice *device);
void validity_list (FpDevice *device);
void validity_delete (FpDevice *device);
void validity_clear_storage (FpDevice *device);

/* Gallery matching helper (validity_verify.c) */
FpPrint *validity_find_gallery_match (GPtrArray *gallery,
                                      guint16    subtype);
