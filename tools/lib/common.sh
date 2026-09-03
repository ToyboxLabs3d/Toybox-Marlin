# Shared helpers for tools/*.sh
# Source this file from scripts in tools/. Do not execute directly.

source "${PIO_ACTIVATE_PATH:-${HOME}/.platformio/penv/bin/activate}"

# ---- Colors / logging ----
if [ -t 1 ]; then
    C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'
    C_RED=$'\033[31m'; C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'
    C_BLUE=$'\033[34m'; C_CYAN=$'\033[36m'
else
    C_RESET=""; C_BOLD=""; C_RED=""; C_GREEN=""; C_YELLOW=""; C_BLUE=""; C_CYAN=""
fi

info()    { echo "${C_CYAN}[INFO] $*${C_RESET}"; }
step()    { echo "${C_BOLD}${C_BLUE}==> $*${C_RESET}"; }
success() { echo "${C_GREEN}[OK] $*${C_RESET}"; }
warn()    { echo 1>&2 "${C_BOLD}${C_YELLOW}[WARN] $*${C_RESET}"; }
err()     { echo 1>&2 "${C_BOLD}${C_RED}[ERROR] $*${C_RESET}"; }


