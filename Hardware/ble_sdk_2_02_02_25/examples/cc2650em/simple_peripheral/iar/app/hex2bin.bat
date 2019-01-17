@echo off
:: ============================
@echo.
@echo convert hex to bin
@echo.
:: ============================
:: User's guide for hex2bin.py: http://pythonhosted.org/IntelHex/part3-4.html

pushd ..

:: Path names
@set DEST_PATH=app\FlashOnly_OAD_ImgB\Exe
@set APP_PATH=app\FlashOnly_OAD_ImgB\Exe

:: Set application name
@set APP_NAME=simple_peripheral_cc2650em

:: Calculate file names
@set APP_IMG=%APP_NAME%_app.hex
@set BIN_IMG=%APP_NAME%_app_oad.bin

:: Application image range
@set APP_IMG_START=9000
@set APP_IMG_END=12FFF

:: 1) OAD Bin.  For programming with CC254x OAD Manager.
"C:\Python27\python" "C:\Python27\Scripts\hex2bin.py" -r "%APP_IMG_START%:%APP_IMG_END%" "%APP_PATH%\%APP_IMG%" "%DEST_PATH%\%BIN_IMG%"
echo Created %DEST_PATH%\%BIN_IMG%

pause
