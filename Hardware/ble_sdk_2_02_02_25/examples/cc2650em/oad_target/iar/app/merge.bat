@echo off
:: ============================
@echo.
@echo Create combined HEX files
@echo.
:: ============================
:: User's guide for hexmerge.py: http://pythonhosted.org/IntelHex/part3-4.html

pushd ..

:: Path names
@set DEST_PATH=app
@set APP_PATH=app\FlashROM\Exe
@set STACK_PATH=stack\FlashROM\Exe
@set BIM_PATH=..\..\..\util\bim\cc2640\iar\FlashOnly\Exe

:: Set application name
@set APP_NAME=oad_target_cc2650em

:: Calculate file names
@set APP_IMG=%APP_NAME%_app.hex
@set STACK_IMG=%APP_IMG:app=stack%
@set FULL_IMG=%APP_IMG:app=unified%

:: Application image start at address 0x100 to include RTOS in ROM configuration table.
@set BIM_START=0
@set APP_START=100

:: Stack image end (page 31 is for BIM, OAD Target and CCFG)
@set STACK_END=1EFFF
@set BIM_END=1FFFF

:: 1) full image (BIM + App + Stack). For download using flash programmer
"C:\Python27\python" "C:\Python27\Scripts\hexmerge.py" -o "%DEST_PATH%\%FULL_IMG%" -r "%BIM_START%:%BIM_END%" "--overlap=error"  "%APP_PATH%\%APP_IMG%":%APP_START%: "%STACK_PATH%\%STACK_IMG%"::%STACK_END% "%BIM_PATH%\bim.hex":%BIM_START%:%BIM_END%
echo Created %DEST_PATH%\%FULL_IMG%

pause
