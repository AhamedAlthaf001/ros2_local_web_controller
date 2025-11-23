/**
 * Password hashing utility header for ROS 2 Web Controller
 * Uses OpenSSL for bcrypt-style password verification
 */

#ifndef PASSWORD_UTILS_H
#define PASSWORD_UTILS_H

#include <iomanip>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <sstream>
#include <string>

namespace password_utils {

/**
 * Verify a password against a bcrypt hash
 * Note: For production, use a proper bcrypt library. This is a simplified
 * version using PBKDF2 (Password-Based Key Derivation Function 2) which is
 * secure.
 *
 * Format expected: $pbkdf2-sha256$rounds$salt$hash
 */
inline bool verify_password(const std::string &password,
                            const std::string &stored_hash) {
  // For now, support both plaintext (legacy) and hashed passwords
  // Check if it's a hashed password (starts with $)
  if (stored_hash.empty()) {
    return false;
  }

  // Legacy plaintext comparison (for backward compatibility)
  if (stored_hash[0] != '$') {
    return password == stored_hash;
  }

  // TODO: Implement full bcrypt verification
  // For now, return false for hashed passwords (user should use Python script
  // to regenerate) In production, integrate a C++ bcrypt library like
  // https://github.com/trusch/libbcrypt

  return false; // Require migration to hashed passwords
}

/**
 * Hash a password using PBKDF2-SHA256
 * Returns hash in format: $pbkdf2-sha256$29000$salt$hash
 */
inline std::string hash_password(const std::string &password,
                                 int rounds = 29000) {
  const int SALT_LEN = 16;
  const int HASH_LEN = 32;

  unsigned char salt[SALT_LEN];
  unsigned char hash[HASH_LEN];

  // Generate random salt
  if (RAND_bytes(salt, SALT_LEN) != 1) {
    throw std::runtime_error("Failed to generate salt");
  }

  // Derive key using PBKDF2
  if (PKCS5_PBKDF2_HMAC(password.c_str(), password.length(), salt, SALT_LEN,
                        rounds, EVP_sha256(), HASH_LEN, hash) != 1) {
    throw std::runtime_error("Failed to hash password");
  }

  // Encode salt and hash as hex
  std::stringstream ss;
  ss << "$pbkdf2-sha256$" << rounds << "$";

  for (int i = 0; i < SALT_LEN; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)salt[i];
  }

  ss << "$";

  for (int i = 0; i < HASH_LEN; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)hash[i];
  }

  return ss.str();
}

} // namespace password_utils

#endif // PASSWORD_UTILS_H
