ESP32 Lightstrip
=========

Neopixel lightstrip control with an IR receiver and a MOSFET based power switch.

Depending on which ESP32 is used, a different target might have to be select in
`.cargo/config.toml`.

## Setting up Development Environment

- [esp-rs book](https://docs.esp-rs.org/book/)

## Elegoo Remote Keybindings

- 0: One Color
- 1: Pulse
- 2: Rainbow
- 3: Moving Rainbow
- 4: Disco Random
- 5: Disco

- VOL+ : Increase brightness
- VOL- : Decrease brightness
- UP: Increase speed
- DOWN: Decrease speed
- Left: Previous color 
- Right: Next color 
- Play/Pause: Freeze current LED state

## Build and Flash for Bedroom

1. `cargo run --features bedroom --release`

## Build and Flash for Tree

1. `cargo run --features tree --no-default-features --release`
