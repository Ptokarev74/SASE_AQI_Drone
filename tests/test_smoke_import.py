import subprocess
import sys


def test_bridge_import_smoke():
    """Verify that 'python -m bridge' can execute without ModuleNotFoundError."""
    try:
        # Run bridge module as a subprocess for 1 second.
        # If the imports are totally broken (e.g., ModuleNotFoundError), it will exit immediately with code 1.
        # If it succeeds it will hang running the server, so we timeout and capture it.
        result = subprocess.run(
            [sys.executable, "-m", "bridge"],
            capture_output=True,
            text=True,
            timeout=1
        )
    except subprocess.TimeoutExpired as e:
        # A timeout means the server started and blocked successfully.
        # e.stdout / e.stderr are None when no input was provided, so guard with `or ""`.
        stdout = e.stdout or ""
        stderr = e.stderr or ""
        assert "ModuleNotFoundError" not in stdout and "ModuleNotFoundError" not in stderr, \
            "Found ModuleNotFoundError despite timeout"
        return
        
    # If it didn't timeout, it crashed immediately. Check why.
    assert result.returncode == 0, f"Bridge failed to start. Return code {result.returncode}\\nSTDOUT: {result.stdout}\\nSTDERR: {result.stderr}"

if __name__ == "__main__":
    test_bridge_import_smoke()
    print("Smoke test passed: module 'bridge' is importable and executes cleanly.")
