TARGET = sysguage

.PHONY: all release check clean install

all:
	cargo build

release:
	cargo build --release

check:
	cargo fmt --check
	cargo clippy --release -- -D warnings
	cargo test --release

install: release
	install -Dm755 target/release/$(TARGET) ~/.local/bin/$(TARGET)

clean:
	cargo clean
