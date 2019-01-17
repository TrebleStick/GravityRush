@echo off
:: ============================
@echo.
@echo Create combined HEX files
@echo.
:: ============================
:: User's guide for hexmerge.py: http://pythonhosted.org/IntelHex/part3-4.html

pushd ..

:: Path names
@set DEST_PATH=app\FlashOnly_OAD
@set APP_PATH=app\FlashOnly_OAD
@set STACK_PATH=stack\FlashROM
@set BIM_PATH=..\..\..\util\bim_extflash\cc2640\ccs\FlashOnly_ST

:: Set application name
@set APP_NAME=sensortag_cc2650stk

:: Calculate file names
@set APP_IMG=%APP_NAME%_app.hex
@set STACK_IMG=%APP_IMG:app=stack%
@set APST_IMG=%APP_IMG:app=app_stack_oad%
@set ALL_IMG=%APP_IMG:app=all%

:: Application image start at address 0x1000 (page 0 reserved for BIM vector table)
@set APP_IMG_START=1000

:: Stack image end (page 31 reserved for BIM and CCFG)
@set OAD_STACK_END=1EFFF

:: 1) OAD image (application + stack, placed from 0x1000 to 0x1EFFF). For download using OAD or flash programmer
"C:\Python27\python" "C:\Python27\Scripts\hexmerge.py" -o "%DEST_PATH%\%APST_IMG%" -r "%APP_IMG_START%:%OAD_STACK_END%" "--overlap=error"  "%APP_PATH%\%APP_IMG%":%APP_IMG_START%: "%STACK_PATH%\%STACK_IMG%"::%OAD_STACK_END%
echo Created %DEST_PATH%\%APST_IMG%

:: 2) Full executable image (application + stack + BIM). For use with flash programmer
"C:\Python27\python" "C:\\Python27\\Scripts\hexmerge.py" -o "%DEST_PATH%\%ALL_IMG%" "--overlap=error"  "%DEST_PATH%\%APST_IMG%" %BIM_PATH%\bim_extflash.hex
echo Created %DEST_PATH%\%ALL_IMG%

pause
