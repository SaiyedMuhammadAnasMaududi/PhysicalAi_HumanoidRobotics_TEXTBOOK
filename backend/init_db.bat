@echo off
REM Database Initialization Script for Windows
REM Run from backend directory

echo =====================================
echo Initializing RAG Chatbot Database
echo =====================================
echo.

python init_db.py

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Database initialized successfully!
) else (
    echo.
    echo Database initialization failed!
    echo Check the error messages above.
)

pause
