# Measure decoded rover dimensions
import subprocess

cmd = r".\build\ProjectARES.exe --smoke-test"
res = subprocess.run(cmd, capture_output=True, text=True, shell=True)
print("STDOUT:")
print(res.stdout)
print("STDERR:")
print(res.stderr)
