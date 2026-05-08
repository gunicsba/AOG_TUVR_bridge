@echo off
pip show pyserial >nul 2>&1
if errorlevel 1 (
    echo Installing pyserial...
    pip uninstall serial -y >nul 2>&1
    pip install pyserial
)
python "%~dp0AOG_TUVR_bridge.py"
pause
