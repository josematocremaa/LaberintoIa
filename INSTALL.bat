@echo off
REM ###############################################################################
REM INSTALADOR AUTOMÁTICO - Controlador Khepera3 en Webots (Windows)
REM 
REM Uso: INSTALL.bat [ruta_webots_project]
REM 
REM Ejemplo:
REM   INSTALL.bat "C:\Users\Usuario\webots_project"
REM   INSTALL.bat "D:\Proyectos\webots"
REM ###############################################################################

setlocal enabledelayedexpansion
cls

echo.
echo ========================================
echo    INSTALADOR CONTROLADOR KHEPERA3
echo ========================================
echo.

REM Validar parametros
if "%1"=="" (
    echo WARNING: Uso: INSTALL.bat [ruta_al_proyecto_webots]
    echo.
    echo Ejemplos:
    echo   INSTALL.bat "C:\Users\Usuario\webots_project"
    echo   INSTALL.bat "D:\Proyectos\webots"
    echo.
    echo Si no tienes proyecto, primero crea la carpeta:
    echo   mkdir "C:\Users\Usuario\webots_project"
    echo   INSTALL.bat "C:\Users\Usuario\webots_project"
    echo.
    pause
    exit /b 1
)

set "WEBOTS_PROJECT=%1"

REM Validar que la ruta existe
if not exist "%WEBOTS_PROJECT%" (
    echo ERROR: La carpeta '%WEBOTS_PROJECT%' no existe
    echo.
    echo Crea la carpeta primero:
    echo   mkdir "%WEBOTS_PROJECT%"
    echo.
    pause
    exit /b 1
)

echo [*] Ruta del proyecto Webots: %WEBOTS_PROJECT%
echo.

REM Crear estructura de carpetas
echo [*] Creando estructura de carpetas...
if not exist "%WEBOTS_PROJECT%\controllers\Grupo_N" mkdir "%WEBOTS_PROJECT%\controllers\Grupo_N"
if not exist "%WEBOTS_PROJECT%\worlds" mkdir "%WEBOTS_PROJECT%\worlds"
echo [OK] Carpetas creadas
echo.

REM Copiar archivos
echo [*] Copiando archivos...

if exist "%cd%\Grupo_N.py" (
    copy /Y "%cd%\Grupo_N.py" "%WEBOTS_PROJECT%\controllers\Grupo_N\Grupo_N.py" >nul
    echo [OK] Grupo_N.py
) else (
    echo [ERROR] No encontrado: Grupo_N.py
)

if exist "%cd%\Grupo_N_advanced.py" (
    copy /Y "%cd%\Grupo_N_advanced.py" "%WEBOTS_PROJECT%\controllers\Grupo_N\Grupo_N_advanced.py" >nul
    echo [OK] Grupo_N_advanced.py
) else (
    echo [WARN] No encontrado: Grupo_N_advanced.py
)

if exist "%cd%\Grupo_N_configurable.py" (
    copy /Y "%cd%\Grupo_N_configurable.py" "%WEBOTS_PROJECT%\controllers\Grupo_N\Grupo_N_configurable.py" >nul
    echo [OK] Grupo_N_configurable.py
) else (
    echo [WARN] No encontrado: Grupo_N_configurable.py
)

if exist "%cd%\config.py" (
    copy /Y "%cd%\config.py" "%WEBOTS_PROJECT%\controllers\Grupo_N\config.py" >nul
    echo [OK] config.py
) else (
    echo [WARN] No encontrado: config.py
)

if exist "%cd%\maze_world.wbt" (
    copy /Y "%cd%\maze_world.wbt" "%WEBOTS_PROJECT%\worlds\maze_world.wbt" >nul
    echo [OK] maze_world.wbt
) else (
    echo [ERROR] No encontrado: maze_world.wbt
)

echo.
echo ========================================
echo    INSTALACION COMPLETADA
echo ========================================
echo.

echo [*] Estructura instalada:
echo.
echo %WEBOTS_PROJECT%\
echo  - controllers\Grupo_N\
echo    + Grupo_N.py
echo  - worlds\
echo    + maze_world.wbt
echo.

echo [*] PROXIMOS PASOS:
echo.
echo 1. Abre Webots
echo 2. File -^> Open World
echo 3. Navega a: %WEBOTS_PROJECT%\worlds\
echo 4. Abre: maze_world.wbt
echo 5. Click derecho en 'Robot' -^> Properties
echo 6. En 'controller' escribe: Grupo_N
echo 7. Click PLAY para ejecutar
echo.

echo [*] Documentacion:
echo  - QUICK_START.md: Guia rapida
echo  - README.md: Documentacion tecnica
echo  - SUMMARY.md: Resumen completo
echo.

echo [SUCCESS] El controlador esta instalado!
echo.
pause
