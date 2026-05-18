# hc32fpu.py
Import("env")

fpuFlags = [
    "-mfloat-abi=hard",
    "-mfpu=fpv4-sp-d16"
]

env.Append(CCFLAGS=fpuFlags, LINKFLAGS=fpuFlags)