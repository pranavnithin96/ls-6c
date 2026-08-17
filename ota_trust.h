#pragma once

// LineSights firmware release-signing public key (ECDSA P-256).
// The matching private key is server-only and is never stored in this repo or
// in a firmware image. OTA metadata is untrusted until the downloaded image's
// SHA-256 digest verifies against this key.
static const char OTA_SIGNING_PUBLIC_KEY[] =
    "-----BEGIN PUBLIC KEY-----\n"
    "MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAE8T6Csi/8cYgmswZYw6IAcnG20fhC\n"
    "feUUPcStruOv7wQbPRHFWMDdD9GulE7x8lbGG+yS5wEITjF2sqgQ+TlLfQ==\n"
    "-----END PUBLIC KEY-----\n";
