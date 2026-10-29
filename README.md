# SonoGuard

SonoGuard is an experimental acoustic-hazard detector for the
[MAX32655FTHR](https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/max32655fthr.html)
development board. The firmware samples audio through the MAX9867 codec,
processes the samples on the Cortex-M4F, and indicates a detected tone with
the board LED.

The project is currently a hardware and signal-processing prototype. It does
not yet provide a phone application, BLE notifications, alert playback, or
event logging.

## Current implementation

The main prototype is [`firmware/CMSIS_DSP`](firmware/CMSIS_DSP):

1. Configures the MAX9867 over I2C at 100 kHz.
2. Captures mono, 16-bit audio through the CMSIS-DSP audio peripheral.
3. Samples at 48 kHz using ping-pong DMA buffers.
4. Computes a 512-point real FFT with CMSIS-DSP.
5. Compares the energy in FFT bins 11-16 (approximately 1.03-1.50 kHz)
   with total spectral energy.
6. Turns on the red LED and prints a warning when the target-band ratio is
   greater than 40 percent and the signal is above the current energy floor.

The detector is a frequency-band heuristic, not a validated classifier. Its
thresholds are currently compile-time constants in
[`firmware/CMSIS_DSP/main.c`](firmware/CMSIS_DSP/main.c).

## Repository layout

| Path | Purpose |
| --- | --- |
| [`firmware/CMSIS_DSP`](firmware/CMSIS_DSP) | Live FFT-based hazard detector and MAX9867 configuration |
| [`firmware/I2S`](firmware/I2S) | Captures five-second, 48 kHz mono WAV files to a microSD card |
| [`firmware/I2C_SCAN`](firmware/I2C_SCAN) | Scans the board I2C bus for connected devices |
| [`firmware/codec_probe`](firmware/codec_probe) | Initializes the MAX9867 and periodically reports that the firmware is alive |
| [`firmware/Hello_World`](firmware/Hello_World) | Basic UART and LED smoke test |
| [`firmware/SDHC_FTHR`](firmware/SDHC_FTHR) | SDHC/FatFs command-line example |
| [`LICENSE`](LICENSE) | MIT license for this repository |

Each firmware directory is an independent MSDK/CodeFusion Studio project.
The project directories also contain their own example-specific README files.

## Hardware

### Required hardware

- Analog Devices/MAXIM MAX32655FTHR board
- USB connection to the board's J4 connector
- MAX9867 codec/audio path supported by the board
- Headphones for audio experiments
- A microSD card for the [`I2S`](firmware/I2S) capture project

### Interfaces used by the firmware

| Function | MAX32655FTHR connection |
| --- | --- |
| Console UART | P0.0 RX, P0.1 TX |
| Codec I2C | I2C1: P0.16 SCL, P0.17 SDA |
| User buttons | SW2/P0.2 is used to start capture/detection |
| Status LEDs | Board RGB LED; the prototype uses LED 0 for detection and LED 1 for ready |
| Audio data | Codec digital audio interface configured by the MSDK peripheral driver |
| SD card | Board SDHC interface used by the capture and filesystem projects |

The I2C scanner has confirmed the MAX9867 at address `0x18` on the FTHR I2C
bus. Check the board schematic before connecting external devices.

## Toolchain setup

Install:

- VS Code with the CodeFusion Studio extension, or the Analog Devices MSDK
  command-line build environment
- The MAX32655 device and board support packages
- A terminal program for the board's console UART
- `make` if building from the command line

The firmware Makefiles expect the `MAXIM_PATH` environment variable to point
to the MSDK root when the projects are outside the SDK tree. The checked-in
`project.mk` files select:

```text
TARGET = MAX32655
BOARD  = FTHR_Apps_P1
```

## Build and flash

### CodeFusion Studio / VS Code

1. Open the repository in VS Code.
2. Open the firmware project you want to run, for example
   `firmware/CMSIS_DSP`.
3. Select the `FTHR_Apps_P1` board target if prompted.
4. Build the project with the project task.
5. Connect to the board's console UART at `115200 8-N-1`.
6. Flash and start the program with the CodeFusion Studio debug/flash task.

### MSDK command line

From a firmware project directory, set `MAXIM_PATH` and run:

```powershell
cd firmware/CMSIS_DSP
$env:MAXIM_PATH = "C:\path\to\MaximSDK"
make -j
```

The exact flash command depends on the installed debugger and CodeFusion
Studio/MSDK setup. Use the project's generated flash/debug task rather than
copying build artifacts manually.

## Running the prototypes

### Live detector

Flash [`firmware/CMSIS_DSP`](firmware/CMSIS_DSP), open the UART, and press
SW2 after the green ready LED is shown. The console prints:

```text
******************* SonoGuard Live DSP *******************
Press SW2 to start the Live Hazard Detector.
Live Detection Running. Play a 1250Hz tone!
```

The detector runs continuously after starting. A target-band detection prints
`[HAZARD DETECTED]` and turns on LED 0. The current implementation has no
automatic stop or calibration flow; reset the board to stop it.

### Audio capture

Flash [`firmware/I2S`](firmware/I2S), insert a FAT-formatted microSD card, and
press SW2 when the green ready LED is shown. The firmware records successive
files named `cap0.wav`, `cap1.wav`, and so on. Each capture is intended to be
five seconds of mono, 16-bit PCM at 48 kHz.

### I2C and codec checks

- [`firmware/I2C_SCAN`](firmware/I2C_SCAN) scans I2C1 and reports responding
  addresses.
- [`firmware/codec_probe`](firmware/codec_probe) probes and configures the
  MAX9867, then prints `alive` every two seconds.
- [`firmware/Hello_World`](firmware/Hello_World) is the simplest board,
  UART, and LED sanity check.

## Development status

- [x] MAX32655FTHR project setup and UART/LED smoke test
- [x] I2C bus scan for board peripherals
- [x] MAX9867 I2C configuration and codec probe
- [x] 48 kHz I2S audio capture to microSD
- [x] On-device 512-point FFT prototype
- [ ] Calibrate detector thresholds against recorded hazard and background audio
- [ ] Add temporal signature checks for sirens and reversing beepers
- [ ] Play an audible or haptic alert on a confirmed detection
- [ ] Add BLE notification support
- [ ] Add persistent event logging
- [ ] Measure detection accuracy, false alarms, latency, CPU load, and battery life
- [ ] Evaluate a TinyML classifier against the frequency-band heuristic

## Limitations

- The current detector recognizes energy in one fixed frequency range; it does
  not identify a complete siren or beeper signature.
- Thresholds have not been validated across environments, microphone
  placement, or hearing-protection configurations.
- The firmware waits on DMA and has no user-facing error recovery or
  calibration procedure.
- The repository does not currently contain host-side Python analysis tools,
  datasets, or a mobile application.
- This prototype must not be relied on as a safety-critical warning system.

## License and acknowledgements

The repository-specific work is licensed under the [MIT License](LICENSE).
Some firmware projects are based on Analog Devices/Maxim MSDK examples and
retain their upstream source notices and license headers.

Developed by Christy Roy using the MAX32655FTHR and MAX9867 ecosystem.
