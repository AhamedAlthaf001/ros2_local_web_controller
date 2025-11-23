#!/usr/bin/env python3
"""
Password hashing utility for ROS 2 Web Controller
Uses bcrypt to hash passwords for storage in .env file
"""

import sys
import getpass
import bcrypt


def hash_password(password: str) -> str:
    """Hash a password using bcrypt"""
    salt = bcrypt.gensalt(rounds=12)
    hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
    return hashed.decode('utf-8')


def verify_password(password: str, hashed: str) -> bool:
    """Verify a password against a bcrypt hash"""
    return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))


def main():
    if len(sys.argv) > 1:
        # Password provided as argument (for scripting)
        password = sys.argv[1]
    else:
        # Interactive mode
        password = getpass.getpass("Enter password to hash: ")
        confirm = getpass.getpass("Confirm password: ")
        
        if password != confirm:
            print("Error: Passwords do not match!", file=sys.stderr)
            sys.exit(1)
    
    hashed = hash_password(password)
    print(f"\nBcrypt hash (copy this to your .env file):")
    print(hashed)
    print(f"\nUpdate your .env file:")
    print(f"USERNAME=admin")
    print(f"PASSWORD={hashed}")
    print(f"\nTo verify: python3 scripts/hash_password.py --verify")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--verify":
        print("Password verification mode")
        password = getpass.getpass("Enter password: ")
        hashed = getpass.getpass("Enter hash: ")
        if verify_password(password, hashed):
            print("✓ Password matches!")
        else:
            print("✗ Password does not match")
    else:
        main()
