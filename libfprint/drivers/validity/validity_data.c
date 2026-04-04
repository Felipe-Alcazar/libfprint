/*
 * Runtime data file loader for Validity/Synaptics VCSFW fingerprint sensors
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

#include "validity_data.h"
#include "fpi-log.h"
#include "fp-device.h"

#include <glib/gstdio.h>
#include <string.h>

/* Search paths for data files */
const gchar *validity_data_search_paths[] = {
  "/usr/share/libfprint/validity",
  "/usr/local/share/libfprint/validity",
  NULL,
};

/* HMAC-SHA256 key for integrity verification.
 * This is NOT secret — it is also compiled into the data package generator.
 * Purpose: detect file corruption and casual tampering, not authentication.
 * Plain text: "libfprint-validity-data-integrit" (32 bytes) */
static const guint8 hmac_key[32] = {
  0x6c, 0x69, 0x62, 0x66, 0x70, 0x72, 0x69, 0x6e,  /* "libfprin" */
  0x74, 0x2d, 0x76, 0x61, 0x6c, 0x69, 0x64, 0x69,  /* "t-validi" */
  0x74, 0x79, 0x2d, 0x64, 0x61, 0x74, 0x61, 0x2d,  /* "ty-data-" */
  0x69, 0x6e, 0x74, 0x65, 0x67, 0x72, 0x69, 0x74,  /* "integrit" */
};

/* Per-device blob filename mapping (indexed by tag) */
static const gchar *device_blob_filenames[] = {
  [VALIDITY_DATA_INIT]             = "init.bin",
  [VALIDITY_DATA_INIT_CLEAN_SLATE] = "init_clean_slate.bin",
  [VALIDITY_DATA_RESET]            = "reset.bin",
  [VALIDITY_DATA_DB_WRITE_ENABLE]  = "db_write_enable.bin",
};

/* Common data filename mapping (indexed by tag) */
static const struct
{
  ValidityDataTag tag;
  const gchar    *filename;
} common_file_map[] = {
  { VALIDITY_DATA_PARTITION_SIG_STANDARD, "partition_sig_standard.bin" },
  { VALIDITY_DATA_PARTITION_SIG_0090,     "partition_sig_0090.bin" },
  { VALIDITY_DATA_CA_PUBKEY,              "ca_pubkey.bin" },
  { VALIDITY_DATA_TLS_PASSWORD,           "tls_password.bin" },
  { VALIDITY_DATA_GWK_SIGN,              "gwk_sign.bin" },
  { VALIDITY_DATA_FW_PUBKEY_X,           "fw_pubkey_x.bin" },
  { VALIDITY_DATA_FW_PUBKEY_Y,           "fw_pubkey_y.bin" },
};

/* ================================================================
 * HMAC-SHA256 verification
 * ================================================================ */

static gboolean
verify_hmac (const guint8 *data,
             gsize         data_len,
             const guint8 *expected_hmac)
{
  guint8 computed[32];

  GHmac *hmac = g_hmac_new (G_CHECKSUM_SHA256, hmac_key, sizeof (hmac_key));

  g_hmac_update (hmac, data, data_len);

  gsize digest_len = sizeof (computed);
  g_hmac_get_digest (hmac, computed, &digest_len);
  g_hmac_unref (hmac);

  /* Constant-time comparison */
  guint diff = 0;
  for (gsize i = 0; i < 32; i++)
    diff |= computed[i] ^ expected_hmac[i];

  return diff == 0;
}

/* ================================================================
 * Public API
 * ================================================================ */

void
validity_data_store_init (ValidityDataStore *store)
{
  memset (store, 0, sizeof (*store));
}

void
validity_data_store_free (ValidityDataStore *store)
{
  for (gsize i = 0; i < VALIDITY_DATA_NUM_TAGS; i++)
    g_clear_pointer (&store->entries[i], g_bytes_unref);
}

gboolean
validity_data_load_file (ValidityDataStore *store,
                         ValidityDataTag    tag,
                         const gchar       *filepath,
                         GError           **error)
{
  g_autofree guint8 *contents = NULL;
  gsize length = 0;

  g_return_val_if_fail (store != NULL, FALSE);
  g_return_val_if_fail (tag < VALIDITY_DATA_NUM_TAGS, FALSE);
  g_return_val_if_fail (filepath != NULL, FALSE);

  if (!g_file_get_contents (filepath, (gchar **) &contents, &length, error))
    return FALSE;

  if (length < VALIDITY_DATA_MIN_FILE_SIZE)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "Data file '%s' too short (%zu bytes, minimum %d)",
                   filepath, length, VALIDITY_DATA_MIN_FILE_SIZE);
      return FALSE;
    }

  gsize data_len = length - VALIDITY_DATA_HMAC_SIZE;
  const guint8 *hmac_trailer = contents + data_len;

  if (!verify_hmac (contents, data_len, hmac_trailer))
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                   "Data file '%s' failed HMAC verification — "
                   "file may be corrupt or from an incompatible version",
                   filepath);
      return FALSE;
    }

  /* Store only the data portion (without HMAC trailer) */
  g_clear_pointer (&store->entries[tag], g_bytes_unref);
  store->entries[tag] = g_bytes_new (contents, data_len);

  fp_dbg ("Loaded data file: %s (%zu bytes)", filepath, data_len);
  return TRUE;
}

gboolean
validity_data_load_device (ValidityDataStore *store,
                           guint16            vid,
                           guint16            pid,
                           GError           **error)
{
  g_autofree gchar *subdir = g_strdup_printf ("%04x_%04x", vid, pid);
  gboolean found_any = FALSE;

  g_return_val_if_fail (store != NULL, FALSE);

  for (const gchar **base = validity_data_search_paths; *base != NULL; base++)
    {
      g_autofree gchar *dir_path = g_build_filename (*base, subdir, NULL);

      if (!g_file_test (dir_path, G_FILE_TEST_IS_DIR))
        continue;

      found_any = TRUE;
      fp_info ("Loading device data from %s", dir_path);

      for (gsize i = 0; i < G_N_ELEMENTS (device_blob_filenames); i++)
        {
          if (device_blob_filenames[i] == NULL)
            continue;

          g_autofree gchar *fpath =
            g_build_filename (dir_path, device_blob_filenames[i], NULL);

          if (!g_file_test (fpath, G_FILE_TEST_IS_REGULAR))
            {
              fp_dbg ("Optional device blob not found: %s", fpath);
              continue;
            }

          if (!validity_data_load_file (store, (ValidityDataTag) i,
                                        fpath, error))
            return FALSE;
        }

      break;  /* Found directory — don't search further */
    }

  if (!found_any)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_NOT_FOUND,
                   "Device data files not found for %04x:%04x. "
                   "Install the libfprint-validity-data package. "
                   "Search paths: /usr/share/libfprint/validity/, "
                   "/usr/local/share/libfprint/validity/",
                   vid, pid);
      return FALSE;
    }

  /* Verify that init.bin was loaded (it is mandatory) */
  if (!store->entries[VALIDITY_DATA_INIT])
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_NOT_FOUND,
                   "Required file init.bin not found for %04x:%04x",
                   vid, pid);
      return FALSE;
    }

  return TRUE;
}

gboolean
validity_data_load_common (ValidityDataStore *store,
                           GError           **error)
{
  g_return_val_if_fail (store != NULL, FALSE);

  for (gsize i = 0; i < G_N_ELEMENTS (common_file_map); i++)
    {
      ValidityDataTag tag = common_file_map[i].tag;
      const gchar *filename = common_file_map[i].filename;
      gboolean found = FALSE;

      for (const gchar **base = validity_data_search_paths;
           *base != NULL; base++)
        {
          g_autofree gchar *fpath =
            g_build_filename (*base, filename, NULL);

          if (!g_file_test (fpath, G_FILE_TEST_IS_REGULAR))
            continue;

          if (!validity_data_load_file (store, tag, fpath, error))
            return FALSE;

          found = TRUE;
          break;
        }

      if (!found)
        {
          g_set_error (error, FP_DEVICE_ERROR,
                       FP_DEVICE_ERROR_DATA_NOT_FOUND,
                       "Common data file '%s' not found. "
                       "Install the libfprint-validity-data package. "
                       "Search paths: /usr/share/libfprint/validity/, "
                       "/usr/local/share/libfprint/validity/",
                       filename);
          return FALSE;
        }
    }

  return TRUE;
}

GBytes *
validity_data_get (const ValidityDataStore *store,
                   ValidityDataTag          tag)
{
  g_return_val_if_fail (store != NULL, NULL);
  g_return_val_if_fail (tag < VALIDITY_DATA_NUM_TAGS, NULL);

  return store->entries[tag];
}

const guint8 *
validity_data_get_bytes (const ValidityDataStore *store,
                         ValidityDataTag          tag,
                         gsize                   *len)
{
  g_return_val_if_fail (len != NULL, NULL);

  GBytes *bytes = validity_data_get (store, tag);
  if (!bytes)
    {
      *len = 0;
      return NULL;
    }

  return g_bytes_get_data (bytes, len);
}
