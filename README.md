# rust-on-m5stack

```bash
# generate project from template
cargo generate esp-rs/esp-idf-template cargo
# build
cargo build
# build and flash
cargo run
```

runner = "espflash flash --baud=921600 --monitor /dev/ttyUSB0"

https://zenn.dev/teruyamato0731/scraps/eaf1afddd92124

環境変数
ESPFLASH_PORT
ESPFLASH_BAUD
