# shared_memory_extended
[![Build Status](https://github.com/phil-opp/shared_memory/workflows/build/badge.svg)](https://github.com/phil-opp/shared_memory/actions?query=workflow%3Abuild)
[![crates.io](https://img.shields.io/crates/v/shared_memory.svg)](https://crates.io/crates/shared_memory)
[![mio](https://docs.rs/shared_memory/badge.svg)](https://docs.rs/shared_memory/)
[![Lines of Code](https://tokei.rs/b1/github/phil-opp/shared_memory?category=code)](https://tokei.rs/b1/github/phil-opp/shared_memory?category=code)

A crate that allows you to share memory between __processes__. Fork of [elast0ny/shared_memory](https://github.com/elast0ny/shared_memory).

This crate provides lightweight wrappers around shared memory APIs in an OS agnostic way. It is intended to be used with it's sister crate [raw_sync](https://github.com/elast0ny/raw_sync-rs) which provide simple primitves to synchronize access to the shared memory (Mutex, RwLock, Events, etc...).

| raw_sync                                                                                                                                                                    |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [![crates.io](https://img.shields.io/crates/v/raw_sync.svg)](https://crates.io/crates/raw_sync) [![docs.rs](https://docs.rs/raw_sync/badge.svg)](https://docs.rs/raw_sync/) |

## Usage

For usage examples, see code located in [examples/](examples/) :

  | Examples                   | Description                                           |
  | -------------------------- | ----------------------------------------------------- |
  | [event](examples/event.rs) | Shows the use of shared events through shared memory  |
  | [mutex](examples/mutex.rs) | Shows the use of a shared mutex through shared memory |

## License

 * [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
 * [MIT license](http://opensource.org/licenses/MIT)

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
