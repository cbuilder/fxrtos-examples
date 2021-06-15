@echo off
echo Cleaning up...

if exist *.o del /F /Q *.o
if exist *.elf del /F /Q *.elf

if "%1"=="clean" (
	echo OK
	exit /b 0
) 

echo Compiling...

call set OBJS=

for %%f in (*.c) do (
	echo %%f
	%GCC_PREFIX%gcc -g -DUSE_STDPERIPH_DRIVER -DGD32VF103V_EVAL -march=rv32im -mabi=ilp32 -O2 -ffunction-sections -I. -c -o %%~nf.o %%f
	call set OBJS=%%OBJS%% %%~nf.o
)

for %%f in (*.S) do (
	echo %%f
	%GCC_PREFIX%gcc -g -DGD32VF103V_EVAL -march=rv32im -mabi=ilp32 -O2 -ffunction-sections -I. -c -o %%~nf.o %%f
	call set OBJS=%%OBJS%% %%~nf.o
)


echo Linking...
%GCC_PREFIX%ld -gc-sections -nostartfiles -T GD32VF103xB.lds -o fxrtos_demo.elf %OBJS% libfxrtos.a
%GCC_PREFIX%objcopy --target ihex fxrtos_demo.elf fxrtos_demo.hex
