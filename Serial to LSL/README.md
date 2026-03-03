# ESP32 ADS1299 Bridge GUI

## Install

```bash
pip install pyserial numpy matplotlib pylsl
```

## Run

```bash
python "UDP bridge.py"
```

## What it does

- Select COM port and baud in the GUI
- Starts your ESP32 stream (`s` then `b` command)
- Shows 8 live EEG channels
- Shows averaged-signal FFT (0-100 Hz)
- Optionally outputs:
  - LSL stream (`OpenBCIEEG`)
  - UDP JSON stream (default `127.0.0.1:12345`)

## Important about OpenBCI GUI external streaming

OpenBCI GUI `STREAMING (from external)` with `Board=Cyton` expects **BrainFlow StreamingBoard protocol** (not arbitrary JSON UDP).

So your current UDP JSON output is good for custom apps, but OpenBCI GUI will fail to initialize with it.

## OpenBCI GUI logs

Check logs here on Windows:

- `C:/Users/<you>/Documents/OpenBCI_GUI/Console_Data/Console_*.txt`
- `C:/Users/<you>/Documents/OpenBCI_GUI/Console_Data/Brainflow_*.txt`

In the GUI, open the Console Log window from the console icon in the top navigation bar.
