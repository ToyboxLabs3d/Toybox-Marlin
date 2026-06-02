Import("env")


import hashlib
import os

APP_BIN = "$BUILD_DIR/${PROGNAME}.bin"
HASH_FILE = "$BUILD_DIR/hash.txt"
BOARD_CONFIG = env.BoardConfig()

def sha256sum(source, target, env):
  try:
    print("Generating hash for ", source[0])
    with open(str(source[0]), 'rb', buffering=0) as bin_file:
        hashValue = hashlib.file_digest(bin_file, 'sha256').hexdigest()
        print("Generated hash")
    with open(str(target[0]), 'w') as hash_file:
          hash_file.write(hashValue)
    print(f"Wrote hash to {target[0]}")
  except Exception as e:
    print("Couldn't generate hash", e)

hash = env.Command(
    HASH_FILE,
    APP_BIN,
    sha256sum,
)
env.Default(hash)
