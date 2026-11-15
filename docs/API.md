# PixelBuds HTTP API

Reference for writing a remote control (phone app, web page, home automation) against the PixelBuds firmware on the `web-control` branch. The firmware's own `/help` route gives a short version of the same list.

## Contents
- [Transport](#transport)
- [Conventions every client must follow](#conventions-every-client-must-follow)
- [Response format](#response-format)
- [State model](#state-model)
- [The status object](#the-status-object)
- [Endpoint reference](#endpoint-reference)
  - [Status](#status)
  - [Overrides (temporary)](#overrides-temporary)
  - [Settings (saved)](#settings-saved)
  - [Testing](#testing)
- [Errors](#errors)
- [Recommended client pattern](#recommended-client-pattern)
- [Known gaps for a remote control](#known-gaps-for-a-remote-control)

## Transport

| Property | Value |
|---|---|
| Protocol | HTTP/1.1, port 80, no TLS |
| Address | The board's IP on the LAN. It uses DHCP, so reserve an address on the router; `pixelbuds.local` is not reliable |
| Methods | Every route answers any method. Use `GET` |
| Request body | Ignored. All options go in the query string |
| Responses | `application/json` for every route except `/`, `/status` and `/help`, which are plain text for people |
| Connection | `Connection: close` on every response; one request per connection |
| Authentication | None. Anything on the LAN can control the light |
| CORS | No `Access-Control-Allow-Origin` header is sent (see [Known gaps](#known-gaps-for-a-remote-control)) |
| Throughput | Requests are served one at a time, once per animation frame (about 85 ms), and only while Wi-Fi is up |

## Conventions every client must follow

- **Render from the `status` object in each reply.** Every successful command returns the complete, already-updated state, so no follow-up request is needed. Never parse `message` or `error`; their wording is for people and may change.
- **Send only documented options.** Any unknown query option, including a cache buster such as `?_=1700000000`, returns `400`.
- **Every option needs a value.** The web server drops options without `=`, so on/off actions are separate paths (`/power/off`), never `?off`.
- **Validation is all-or-nothing.** If any option in a request is invalid, the request returns `400` and nothing changes.
- **Plus signs must be encoded.** In a query string `+` decodes to a space. Send positive numbers without a sign, or encode it as `%2B`.
- **Times are local 24-hour `H:MM` or `HH:MM`** (`7:30` and `07:30` are accepted; `7:5` is not). Minutes and seconds are integers unless stated.
- **Every request takes effect immediately** and affects everyone; there is no session or undo.

## Response format

Every JSON route except `/status.json` wraps its reply in an envelope.

Success (`200`):

```json
{ "ok": true, "message": "Brightness set.", "status": { ...the status object... } }
```

Failure (`400`, `404`, `409`):

```json
{ "ok": false, "error": "?scale= takes a percentage, 0 to 100." }
```

| Field | Type | Meaning |
|---|---|---|
| `ok` | bool | `true` for success, `false` for any error |
| `message` | string | Short human-readable confirmation |
| `status` | object | The full state after the change; identical in shape to `/status.json` |
| `error` | string | Human-readable reason, naming the valid values where relevant |

`/status.json` returns the bare status object, with no envelope. Routes that take options but are called without any (`/brightness`, `/adjust`, `/colors`, `/schedule`, `/rules`, `/islands`) change nothing and simply return the envelope, so they double as reads.

## State model

The light has three layers. A remote control should present them separately.

| Layer | Set by | Lifetime |
|---|---|---|
| **Schedule** | The sun and `/schedule` | Always running. Picks the current phase and look |
| **Overrides** | `/power`, `/brightness`, `/sleep`, `/adjust`, `/colors`, `/phase/next`, `/pause` | Until the **phase name changes**, `/reset`, or a reboot |
| **Settings** | `/schedule`, `/rules`, `/islands` | Saved to flash; kept across reboots until changed |

**Phases**, in daily order: `embers` → `wake` (sunrise ramp) → `day` → `golden hour` (two segments, either side of sunset) → `evening` → `wind-down` → `embers`. An override ends when the phase *name* changes, so the two golden-hour segments and embers across midnight don't end one. `overrides.until` in the status says when the current overrides will end.

**Exceptions to "until the phase changes":**
- `/power/off?until=morning` and `/sleep` last until the `wake` (or `day`) phase begins.
- During a hold (`/test?hold=`) the phase never changes, so overrides last until the hold ends.
- Changing `/schedule` can move the current time into a different phase, which clears overrides.

**Brightness shown** = `look.brightness` × `power.dimmer`. The dimmer combines power, the brightness override and the sleep ramp, and glides to new values: about 1 s for brightness changes, 3 s when overrides clear, and the requested fade for power.

## The status object

Returned bare by `/status.json` and as `status` in every success envelope. Example (night, no overrides):

```json
{
  "clock": "2026-11-15T00:51:21-0700",
  "mode": "schedule",
  "hold": null,
  "testDay": null,
  "skip": null,
  "lightTime": "00:51",
  "phase": "embers",
  "progress": 0.12,
  "sun": { "rise": "07:13", "set": "17:10", "fromClock": true },
  "look": {
    "brightness": 10.0, "pixels": 6.0, "pixelsNow": 6,
    "moveEvery": 6.0, "mixEvery": 3.0, "fade": 16.0,
    "weights": { "red": 1.0, "green": 0.0, "blue": 0.0, "amber": 0.4 }
  },
  "schedule": [
    { "at": "00:00", "phase": "embers" }, { "at": "07:13", "phase": "wake" },
    { "at": "08:13", "phase": "day" }, { "at": "16:10", "phase": "golden hour" },
    { "at": "17:10", "phase": "golden hour" }, { "at": "18:10", "phase": "evening" },
    { "at": "22:00", "phase": "wind-down" }, { "at": "23:00", "phase": "embers" }
  ],
  "scheduleSettings": { "winddown": "22:00", "embers": "23:00", "wake": 60, "golden": 60, "sunrise": 0, "sunset": 0 },
  "power": { "on": true, "untilMorning": false, "sleepSecondsLeft": null, "dimmer": 1.0 },
  "overrides": {
    "brightness": 100, "speed": 100, "pixels": 100, "trail": 100, "mix": 100,
    "colors": { "red": null, "green": null, "blue": null, "amber": null },
    "paused": false,
    "until": { "phase": "wake", "at": "07:13" }
  },
  "rules": { "mix": "pool", "mixrate": 100, "life": [30, 100], "expire": "snap", "round": false },
  "islands": { "mode": "split", "hold": 0.0, "holdSetting": "auto", "boost": 2.0 },
  "uptimeSeconds": 408,
  "build": "Nov 15 2026 00:41:47"
}
```

### Fields

| Field | Type | Meaning |
|---|---|---|
| `clock` | string \| null | Local wall clock, ISO 8601 with offset. `null` until the board gets the time from NTP |
| `mode` | string | `schedule`, `test` (24-minute test day), `hold`, `skipped` (after `/phase/next`), or `fallback` (no clock yet) |
| `hold` | object \| null | While `mode` is `hold`: `{look, time, secondsLeft}`. Exactly one of `look` (`day`, `golden`, `evening`, `embers`) and `time` (`"HH:MM"`) is set, the other `null`. `secondsLeft` is `null` when the hold has no `for=` limit |
| `testDay` | object \| null | While `mode` is `test`: `{secondsLeft}` until the test day ends by itself |
| `skip` | object \| null | While `mode` is `skipped`: `{look, catchUpAt, secondsLeft}`: the look jumped to, the `"HH:MM"` when the schedule reaches it, and seconds until then |
| `lightTime` | `"HH:MM"` \| null | The time the light is showing. Equals the clock in `schedule` mode, `catchUpAt` while skipped; `null` while holding a named look |
| `phase` | string | Current phase name (see [State model](#state-model)). `day (waiting for clock)` before NTP |
| `progress` | number 0–1 | Progress through the current schedule segment |
| `sun.rise`, `sun.set` | `"HH:MM"` | Today's real sunrise and sunset, before any `/schedule` shift |
| `sun.fromClock` | bool | `false` means default 06:00/18:00 times (no clock yet) |
| `look.brightness` | number 0–255 | The look's brightness before the dimmer |
| `look.pixels` | number | Target pixel count, after the `pixels` override |
| `look.pixelsNow` | int | Pixels currently alive, including ones fading in or out |
| `look.moveEvery`, `look.mixEvery` | number ≥ 1 | Frames between moves and between colour mixes, after overrides |
| `look.fade` | number 0–255 | Trail fade per frame, after the `trail` override |
| `look.weights.<colour>` | number 0–1 | Spawn weight of each colour, after `/colors`. Colours: `red`, `green`, `blue`, `amber` |
| `schedule[]` | array | Today's keyframes in order; `at` is when the named phase starts |
| `scheduleSettings` | object | Saved `/schedule` values. `wake`, `golden`, `sunrise` and `sunset` are minutes |
| `power.on` | bool | `false` while powered off (including after a sleep timer finishes) |
| `power.untilMorning` | bool | The power-off lasts until wake rather than the next phase |
| `power.sleepSecondsLeft` | int \| null | Seconds until a running sleep timer reaches off |
| `power.dimmer` | number 0–1 | The share of the look's brightness being shown right now. It glides, so a command's reply shows where the glide starts; `overrides.brightness` and `power.on` show the target |
| `overrides.brightness` | int 0–100 | Brightness override, percent |
| `overrides.speed`, `.pixels`, `.trail`, `.mix` | int | `/adjust` percentages; 100 means no change |
| `overrides.colors.<colour>` | int 0–100 \| null | Colour override; `null` means the schedule decides |
| `overrides.paused` | bool | Motion is frozen |
| `overrides.until` | object \| null | `{phase, at}` of the next phase change, when overrides end. `null` during a hold |
| `rules` | object | Saved `/rules` values; `life` is `[min, max]` |
| `islands.mode` | string | `off`, `split`, `edge` or `orphan` |
| `islands.hold` | number 0–1 | Island-hold strength in force now |
| `islands.holdSetting` | `"auto"` \| int 0–100 | The saved `hold` option |
| `islands.boost` | number 0–5 | The saved `boost` option |
| `uptimeSeconds` | int | Seconds since boot. A drop means the board rebooted, which cleared all overrides |
| `build` | string | Firmware compile time. Changes after a wireless update |

## Endpoint reference

Each command returns `200` with the success envelope unless noted. Defaults are the values used when an option is left out.

### Status

| Route | Options | Returns |
|---|---|---|
| `/status.json` | none (extras are ignored here) | The bare status object |
| `/` or `/status` | none (extras are ignored) | The same state as plain text, for people |
| `/help` | none | A plain-text route list |

### Overrides (temporary)

#### `/power/off`
Turns the light off. The animation keeps running unseen.

| Option | Values | Default |
|---|---|---|
| `fade` | seconds, 0–600 | 1 |
| `until` | `phase` or `morning` | `phase` |

Cancels a running sleep timer.

#### `/power/on`
Turns the light on and cancels power-off (including `until=morning`) and any sleep timer.

| Option | Values | Default |
|---|---|---|
| `fade` | seconds, 0–600 | 1 |

#### `/brightness`
| Option | Values | Default |
|---|---|---|
| `scale` | percent, 0–100 | none: with no option, returns the current override status |

Percent of the current look's brightness; it cannot exceed the schedule.

#### `/sleep`
| Option | Values | Default |
|---|---|---|
| `in` | minutes, 1–240 | required |

Dims smoothly to off over `in` minutes, then stays off until wake. A phase change while sleeping does not cancel it or reset the brightness override. Cancel with `/power/on` or `/reset`.

#### `/adjust`
Percent changes to the current look; 100 means no change. With no options, returns the current status.

| Option | Values | Effect |
|---|---|---|
| `speed` | 25–400 | How often pixels move |
| `pixels` | 25–200 | How many pixels there are; extras fade in or out |
| `trail` | 25–400 | How long trails last |
| `mix` | 25–400 | How often colours mix |

`speed` and `mix` cannot go faster than once per frame, so values above 100 have no effect in the `day` look, which already moves and mixes every frame.

#### `/colors`
| Option | Values |
|---|---|
| `red`, `green`, `blue`, `amber` | 0–100, or `auto` to hand back to the schedule |

Sets how often each colour spawns. 0 makes pixels of that colour fade out; a non-zero value brings in a colour the current look doesn't use. Returns `400` if the result would leave no colour on.

#### `/phase/next`
No options. Shows the next look immediately (day → golden → evening → embers → day). The schedule catches up when the wall clock reaches that look's start. Because the phase name changes, other overrides clear. `mode` reads `skipped` until the clock catches up. Returns `409` during a test day or hold, or before the clock is set.

#### `/pause/on`, `/pause/off`
No options. Freezes or resumes movement, mixing and ageing. The picture stays lit.

#### `/shuffle`
No options. Rescatters every pixel with fresh colours and lifetimes. Visible as a jump.

#### `/reset`
No options. Clears every override: power-off, sleep, brightness, adjust, colours, pause and `/phase/next`. It does not end a test day or hold, and it does not change settings.

### Settings (saved)

Saved to flash only when a value changes. With no options, each route returns its current values. Each has a reset route that takes no options: `/schedule/reset`, `/rules/reset`, `/islands/reset`.

#### `/schedule`
| Option | Values | Default |
|---|---|---|
| `winddown` | `HH:MM`, 12:00–23:59 | 22:00 |
| `embers` | `HH:MM`, at or after `winddown` (checked against the new or current value) | 23:00 |
| `wake` | minutes from sunrise to the full day look, 0–240 | 60 |
| `golden` | minutes of golden hour on each side of sunset, 0–180 | 60 |
| `sunrise` | minutes added to sunrise, −180 to 180 | 0 |
| `sunset` | minutes added to sunset, −180 to 180 | 0 |

Rebuilds today's schedule immediately. A late sunset still pushes wind-down and embers later than these times, so the evening always gets an hour. Read the effective times from `schedule[]`.

#### `/rules`
| Option | Values | Default |
|---|---|---|
| `mix` | `pool` (pool and split colours) or `pair` (average with each neighbour in turn) | `pool` |
| `mixrate` | 0–100, how far each mix moves toward the shared colour; 0 stops mixing | 100 |
| `life` | `MIN-MAX`, each 1–2000, MIN ≤ MAX: lifetime in frames spent touching another pixel | `30-100` |
| `expire` | `snap` (goes black at end of life) or `fade` (fades out, then respawns) | `snap` |
| `round` | `on` or `off`: rounding keeps colours brighter but paler | `off` |

`life` applies to newly spawned pixels, so changes show gradually.

#### `/islands`
The island-hold experiment for longer-lived daytime clumps.

| Option | Values | Default |
|---|---|---|
| `mode` | `off`, `split` (left half `edge`, right half `orphan`), `edge`, `orphan` | `split` |
| `boost` | 0–5, decimals allowed, no sign | 2 |
| `hold` | `auto` (follows the look: on by day, off otherwise) or 0–100 to fix the strength in every phase | `auto` |
| `round` | `on` or `off`: same setting as `/rules?round=` | `off` |

Changing `mode` rescatters every pixel on the next frame (a visible jump).

### Testing

#### `/test`
| Option | Values | Default |
|---|---|---|
| `hold` | `day`, `golden`, `evening`, `embers`, or `HH:MM` | none: plays a whole day from 05:00 in 24 minutes |
| `for` | minutes, 1–1440; only with `hold` | none: holds until `/test/stop` |

A hold pins the light to one look or time. `mode` becomes `hold` (or `test` for the test day).

#### `/test/stop`
No options. Ends a test day or hold and returns to the wall clock.

## Errors

| Code | When | Body |
|---|---|---|
| `200` | Success | Success envelope with the full status |
| `400` | Unknown option, bad value, missing required option, or an impossible combination | Error envelope naming the problem and the valid values |
| `404` | Unknown path | Error envelope: `No such endpoint: /path. See /help.` |
| `409` | `/phase/next` during a test day or hold, or before the clock is set | Error envelope |

Branch on the HTTP status code (or `ok`), not on the error text.

## Recommended client pattern

1. On launch, `GET /status.json`. If it fails, show the board as offline.
2. Poll `/status.json` every 2–5 s while the remote is on screen. Every second at most: requests compete with the animation.
3. After any command, check for `200` and render from the reply's `status`. On an error, show `error` and keep the previous state.
4. Watch `uptimeSeconds`: if it drops, the board rebooted and every override is gone.
5. Show when overrides end, from `overrides.until` (or "until wake" when `power.untilMorning` is true).
6. Disable `/phase/next` when `mode` is `test` or `hold`, or `clock` is `null`.
7. Show countdowns from `hold.secondsLeft`, `testDay.secondsLeft`, `skip.secondsLeft` and `power.sleepSecondsLeft`, counting down locally between polls.

```sh
B=http://192.168.1.218
curl -s "$B/status.json"
curl -s "$B/brightness?scale=40" | jq '.ok, .status.power.dimmer'
curl -s "$B/power/off?fade=5&until=morning"
curl -s "$B/schedule?sunrise=-20&winddown=22:30"
```

## Known gaps for a remote control

These limit what a client can do today.

- **No CORS headers.** A web page served from another origin can't read responses (native apps and `curl` are unaffected). Adding `Access-Control-Allow-Origin: *` to every response would fix it.
- **No discovery.** The board has no stable name on the network; reserve its IP.
- **State-changing GETs.** Link previews or browser prefetching of a command URL will run it.
