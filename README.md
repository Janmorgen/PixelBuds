# PixelBuds
Pure red, green and blue pixels wander a 32 x 8 WS2812B LED matrix, share colours with the neighbours they touch, then die and respawn. A day cycle follows the real sun: bright mixed colours by day, warm golden hour and evening, dim embers at night.

Everything is controlled over Wi-Fi through a small plain-text HTTP API.

## Hardware
- ESP32 dev board (classic ESP32, CP2102 USB).
- 32 x 8 serpentine WS2812B matrix, data on GPIO26.
- 5 V supply; FastLED limits the matrix to 500 mA.

## Setup
Copy `secrets.h.example` to `secrets.h` (it is gitignored) and fill in your Wi-Fi network, an OTA password, your POSIX time zone and your latitude and longitude (for sunrise and sunset).

Required libraries: FastLED and ArxContainer, with the esp32 Arduino core.

The first flash must be over USB, because it sets up the flash partitions that wireless updates need (`sketch.yaml` pins them). After that, flash over Wi-Fi to the board's IP address:

```
arduino-cli compile --output-dir build .
arduino-cli upload -p <board IP> --protocol network --upload-field password=<OTA password> --input-dir build .
```

## Day cycle
Looks change at keyframes anchored to local sunrise and sunset: embers, wake (sunrise ramp), day, golden hour (either side of sunset), evening, wind-down (22:00) and embers (23:00). Transitions ramp across each stretch. A late summer sunset pushes wind-down later so the evening always gets at least an hour.

## HTTP API
Every route answers GET at `http://<board IP>/` and checks all its options before changing anything. Routes reply with JSON carrying the full updated status; `/`, `/status` and `/help` are plain text. `/help` lists them all. For writing a client such as a remote control, see [docs/API.md](docs/API.md): the JSON status fields, exact option ranges, error codes and client conventions.

### Status
| Route | Does |
|---|---|
| `/` or `/status` | Clock, phase, sun times, current look, schedule, settings and overrides |
| `/status.json` | The same as JSON |
| `/help` | Every route and option |

### Overrides
Temporary changes. They end when the phase changes (the status shows when), or at `/reset`. A reboot clears them.

| Route | Does |
|---|---|
| `/power/off?fade=SEC&until=morning` | Turns the light off, fading over 0–600 s (default 1). `until=morning` keeps it off until the wake phase |
| `/power/on?fade=SEC` | Turns it back on; also cancels a sleep timer |
| `/brightness?scale=0-100` | Dims the current look |
| `/sleep?in=MIN` | Dims to off over 1–240 minutes, then stays off until wake |
| `/adjust?speed=&pixels=&trail=&mix=` | Percent of the current look: 25–400, pixels 25–200 |
| `/colors?red=&green=&blue=&amber=` | How often each colour spawns, 0–100, or `auto` for the schedule's |
| `/phase/next` | Shows the next look now; the schedule catches up |
| `/pause/on`, `/pause/off` | Freezes or resumes the motion |
| `/shuffle` | Rescatters the pixels |
| `/reset` | Clears every override |

### Settings
Kept until changed, across reboots. Each has a `/reset` route back to the defaults (`/schedule/reset`, `/rules/reset`, `/islands/reset`).

| Route | Does |
|---|---|
| `/schedule?winddown=HH:MM&embers=HH:MM&wake=MIN&golden=MIN&sunrise=MIN&sunset=MIN` | Fixed times, transition lengths, and shifts to the sun times (negative is earlier) |
| `/rules?mix=pool\|pair&mixrate=0-100&life=MIN-MAX&expire=snap\|fade&round=on\|off` | How pixels share colour, how long they live (frames spent touching another pixel) and how they die |
| `/islands?mode=off\|split\|edge\|orphan&boost=0-5&hold=auto\|0-100` | The island-hold experiment for longer-lived daytime clumps |

### Testing
| Route | Does |
|---|---|
| `/test` | Plays a whole day in 24 minutes from 05:00 |
| `/test?hold=day\|golden\|evening\|embers\|HH:MM&for=MIN` | Holds one look or time, optionally for 1–1440 minutes |
| `/test/stop` | Ends a test day or hold |

The API has no authentication: anything on the local network can change the light.
