# PixelBuds HTTP API

The developer reference for every HTTP route the PixelBuds firmware serves, for writing a remote control such as a phone app, web page or home automation. This file is the source of truth for the API: [Route index](#route-index) lists every route with its options, and [Endpoint reference](#endpoint-reference) describes each one. The firmware's own `/help` route gives a short version of the same list.

## Contents
- [Route index](#route-index)
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

## Route index

Every route the firmware registers. Options go in the query string; a route listed with no options rejects any. Values are summarised here; each route's section has the details and defaults.

| Route | Options | Does |
|---|---|---|
| [`/`, `/status`](#status) | none | Status as plain text, for people |
| [`/status.json`](#status) | none | The status object as JSON |
| [`/help`](#status) | none | A plain-text route list |
| [`/power/off`](#poweroff) | `fade` 0–600 s · `until` `phase`\|`morning` | Turns the light off |
| [`/power/on`](#poweron) | `fade` 0–600 s | Turns the light on; cancels a sleep timer |
| [`/brightness`](#brightness) | `scale` 0–400 | Percent of the look's brightness |
| [`/sleep`](#sleep) | `in` 1–240 min, required | Dims to off over `in` minutes, then off until wake |
| [`/adjust`](#adjust) | `speed`, `trail`, `mix` 25–400 · `pixels` 25–200 | Percent changes to the current look |
| [`/colors`](#colors) | `red`, `green`, `blue`, `amber` 0–100 or `auto` | How often each colour spawns |
| [`/phase/next`](#phasenext) | none | Shows the next look now |
| [`/pause/on`, `/pause/off`](#pauseon-pauseoff) | none | Freezes or resumes the motion |
| [`/shuffle`](#shuffle) | none | Rescatters every pixel |
| [`/reset`](#reset) | none | Clears every override |
| [`/schedule`](#schedule) | `winddown`, `embers` HH:MM · `wake` 0–240 · `golden` 0–180 · `sunrise`, `sunset` −180–180 | Schedule times (saved) |
| [`/schedule/reset`](#reset-routes) | none | Schedule back to defaults |
| [`/rules`](#rules) | `mix` `pool`\|`pair` · `mixrate` 0–100 · `life` MIN-MAX · `expire` `snap`\|`fade` · `round` `on`\|`off` | Pixel mixing, lifetime and death (saved) |
| [`/rules/reset`](#reset-routes) | none | Rules back to defaults |
| [`/effects`](#effects) | `ageing` `normal`\|`edge` · `death` list · `island` list · `strength` `auto`\|0–100 · `boost` 0–5 | Effects on ageing, pixel death and island breakup (saved) |
| [`/effects/reset`](#reset-routes) | none | Effects back to defaults |
| [`/islands`](#islands-old-name) | `mode` · `hold` · `boost` · `round` | Old name for `/effects`, kept for existing clients |
| [`/islands/reset`](#reset-routes) | none | Same as `/effects/reset` |
| [`/test`](#test) | `hold` look\|HH:MM · `for` 1–1440 min | Plays a 24-minute test day, or holds one look or time |
| [`/test/stop`](#teststop) | none | Ends a test day or hold |

Any other path returns `404`.

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
| Throughput | Requests are served one at a time, once per LED frame (18.75 ms, about 19 ms between requests), and only while Wi-Fi is up |

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

`/status.json` returns the bare status object, with no envelope. Routes that take options but are called without any (`/brightness`, `/adjust`, `/colors`, `/schedule`, `/rules`, `/effects`, `/islands`) change nothing and simply return the envelope, so they double as reads.

## State model

The light has three layers. A remote control should present them separately.

| Layer | Set by | Lifetime |
|---|---|---|
| **Schedule** | The sun and `/schedule` | Always running. Picks the current phase and look |
| **Overrides** | `/power`, `/brightness`, `/sleep`, `/adjust`, `/colors`, `/phase/next`, `/pause` | Until the **phase name changes**, `/reset`, or a reboot |
| **Settings** | `/schedule`, `/rules`, `/effects` | Saved to flash; kept across reboots until changed |

**Phases**, in daily order: `embers` → `wake` (sunrise ramp) → `day` → `golden hour` (two segments, either side of sunset) → `evening` → `wind-down` → `embers`. An override ends when the phase *name* changes, so the two golden-hour segments and embers across midnight don't end one. `overrides.until` in the status says when the current overrides will end.

**Exceptions to "until the phase changes":**
- `/power/off?until=morning` and `/sleep` last until the `wake` (or `day`) phase begins.
- During a hold (`/test?hold=`) the phase never changes, so overrides last until the hold ends.
- Changing `/schedule` can move the current time into a different phase, which clears overrides.

**Brightness shown** = `look.brightness` × `power.dimmer`, at most 255. The dimmer combines power, the brightness override and the sleep ramp, and glides to new values: about 1 s for brightness changes, 3 s when overrides clear, and the requested fade for power. It never goes above the point where the look reaches 255, so a fade, sleep or glide from a high setting starts dimming at once instead of holding at 255 first; only the power cap (see [`/brightness`](#brightness)) can still hold a bright frame level for the start of a fade.

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
  "effects": {
    "ageing": "normal", "death": [], "island": [],
    "strength": 0.0, "strengthSetting": "auto", "boost": 2.0, "islandsNow": 2
  },
  "frames": { "fps": 53.3, "targetFps": 53.3, "workAvgMs": 1.23, "workMaxMs": 2.34, "late": 0 },
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
| `look.moveEvery`, `look.mixEvery` | number ≥ 0.25 | Steps (75 ms) between moves and between colour mixes, after overrides. Below 1 means several per step (0.25 is the floor: one per LED frame) |
| `look.fade` | number 0–255 | Trail fade per step (75 ms), spread over the step's four LED frames, after the `trail` override |
| `look.weights.<colour>` | number 0–1 | Spawn weight of each colour, after `/colors`. Colours: `red`, `green`, `blue`, `amber` |
| `schedule[]` | array | Today's keyframes in order; `at` is when the named phase starts |
| `scheduleSettings` | object | Saved `/schedule` values. `wake`, `golden`, `sunrise` and `sunset` are minutes |
| `power.on` | bool | `false` while powered off (including after a sleep timer finishes) |
| `power.untilMorning` | bool | The power-off lasts until wake rather than the next phase |
| `power.sleepSecondsLeft` | int \| null | Seconds until a running sleep timer reaches off |
| `power.dimmer` | number 0–4, at most 255 ÷ `look.brightness` | The multiplier on the look's brightness right now; above 1 is brighter than the look. It stops at the headroom where the look reaches 255, so on `day` (brightness 100) it is at most 2.55 even at 400 %. It glides, so a command's reply shows where the glide starts; `overrides.brightness` shows the requested percent (such as 400) and `power.on` the target |
| `overrides.brightness` | int 0–400 | Brightness override, percent; 100 means no change |
| `overrides.speed`, `.pixels`, `.trail`, `.mix` | int | `/adjust` percentages; 100 means no change |
| `overrides.colors.<colour>` | int 0–100 \| null | Colour override; `null` means the schedule decides |
| `overrides.paused` | bool | Motion is frozen |
| `overrides.until` | object \| null | `{phase, at}` of the next phase change, when overrides end. `null` during a hold |
| `rules` | object | Saved `/rules` values; `life` is `[min, max]` |
| `effects.ageing` | string | `normal` or `edge` |
| `effects.death` | string[] | Death effects in force, from `orphan`, `contagion`, `scatter`; empty for none |
| `effects.island` | string[] | Island effects in force, from `seed`, `ripple`; empty for none |
| `effects.strength` | number 0–1 | Effect strength in force now; 0 means every effect is idle |
| `effects.strengthSetting` | `"auto"` \| int 0–100 | The saved `strength` option |
| `effects.boost` | number 0–5 | The saved `boost` option |
| `effects.islandsNow` | int | Islands (two or more touching lit pixels) on the matrix right now |
| `frames.fps` | number | LED frames per second over the last full second. Reads 0 until the first full second after boot has passed |
| `frames.targetFps` | number | The frame rate aimed for: 4 frames per 75 ms step, about 53.3 |
| `frames.workAvgMs`, `frames.workMaxMs` | number | Average and worst time per frame, from the start of the frame's work to its end, over the last full second. It includes network handling, the wait in `frameShow()` for 9 ms to pass since the previous `show()` started, and `show()`. A frame has 18.75 ms. Both read 0 until the first full second after boot has passed |
| `frames.late` | int | Frames since boot whose work ended past their 18.75 ms slot while the frame clock was on schedule. A frame that ends past its slot after the frame before it also did is not counted again, so an isolated overrun of any length counts once. When a frame ends more than 18.75 ms past its slot the clock restarts its schedule from then, and the next frame to end past its slot counts again |
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
| `scale` | percent, 0–400 | none: with no option, returns the current override status |

Percent of the current look's brightness. Above 100 makes the light brighter than the look, up to the matrix's maximum of 255. The firmware also caps the matrix at 5 V, 500 mA, and FastLED dims the whole frame to stay under it. At high settings the day and golden looks show less than asked, and so can evening when effects that add light (such as ripple) are on; embers stays under the cap at 400 %.

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

The simulation advances in 75 ms steps and the LEDs refresh four times per step (18.75 ms frames). Moves and mixes run once per frame at most, so up to four per step. Every `speed` and `mix` value from 25 to 400 changes the rate in every look, `day` included (100 % there is one move and one mix per step).

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

Saved to flash only when a value changes. With no options, each route returns its current values.

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
| `life` | `MIN-MAX`, each 1–2000, MIN ≤ MAX: lifetime in steps (75 ms each) spent touching another pixel | `30-100` |
| `expire` | `snap` (goes black at end of life) or `fade` (fades out, then respawns) | `snap` |
| `round` | `on` or `off`: rounding keeps colours brighter but paler | `off` |

`life` applies to newly spawned pixels, so changes show gradually.

#### `/effects`
What happens as pixels age and die, and when an island (two or more touching lit pixels) breaks up. Every selected effect applies to every pixel across the whole matrix. Effects act in proportion to `strength`, so with `strength=auto` they run by day and fade out across golden hour. The defaults turn every effect off, which is how the light behaved before the island work.

| Option | Values | Default |
|---|---|---|
| `ageing` | `normal`, or `edge`: contact ages a pixel more slowly, and more slowly still the more neighbours it has, so islands wear from the outside in | `normal` |
| `death` | `none`, or a comma-separated list of effects run on a dying pixel's neighbours: `orphan` (a neighbour left alone gets a longer life), `contagion` (neighbours lose part of their remaining life), `scatter` (neighbours are pushed a few cells away) | `none` |
| `island` | `none`, or a comma-separated list of effects run when an island breaks up without merging into another: `seed` (the next respawns appear where it was, in its colour), `ripple` (a ring in its colour spreads from where it was; only islands that reached 3 pixels and lasted about 2 s) | `none` |
| `strength` | `auto` (follows the look: rises across the wake hour, full by day, falls to 0 across golden hour, 0 in evening and embers) or 0–100 to fix it in every phase | `auto` |
| `boost` | 0–5, decimals allowed, no sign: how strongly each effect acts at full strength (edge and orphan lives, the share of life contagion takes, scatter distance, seed count, ripple size) | 2 |

A list replaces the current one: `?death=orphan,scatter` turns on those two and turns `contagion` off. Changes apply from the next step without rescattering.

#### `/islands` (old name)
Kept for clients written against the island-hold experiment. It changes the `/effects` and `/rules` settings and replies like any settings route.

| Option | Values | Effect |
|---|---|---|
| `mode` | `off`, `edge`, `orphan` | Sets `ageing` and the `orphan` death effect: `off` neither, `edge` edge only, `orphan` orphan only. Other effects are kept. The old `split` mode was removed and returns `400` |
| `hold` | `auto` or 0–100 | Same as `/effects?strength=` |
| `boost` | 0–5 | Same as `/effects?boost=` |
| `round` | `on` or `off` | Same as `/rules?round=` |

#### Reset routes
No options. Each puts one group of settings back to its defaults and saves it.

| Route | Defaults |
|---|---|
| `/schedule/reset` | `winddown=22:00`, `embers=23:00`, `wake=60`, `golden=60`, `sunrise=0`, `sunset=0` |
| `/rules/reset` | `mix=pool`, `mixrate=100`, `life=30-100`, `expire=snap`, `round=off` |
| `/effects/reset` | `ageing=normal`, `death=none`, `island=none`, `strength=auto`, `boost=2`: every effect off |
| `/islands/reset` | Same as `/effects/reset` |

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
2. Poll `/status.json` every 2–5 s while the remote is on screen, and at most once a second. Each request is handled inside an LED frame.
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
- **A slow request freezes the light.** Requests are read inside the animation loop, and the server waits up to 5 s for a request that arrives slowly or in pieces. Send each request in one go.
