# stm32f7-blink

Minimal template for stm32f7 (microcontroller) development.
Lets an on-board LED blink.

## Building & running

Besides `cargo`, you'll need a few more things to build this project.

- Run `rustup target add thumbv7em-none-eabihf`.
- Install `openocd` (distro repos or manually from https://sourceforge.net/projects/openocd).
- Install `gdb-multiarch`; this may take a while.

To run the project, plug in the board and have `openocd` running in the background:

```
sudo openocd –f board/stm32f7discovery.cfg
```

In another terminal, run `cargo run` and enter `continue` once the program has been flashed.
