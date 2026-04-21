# raw_sync

[![Build Status](https://github.com/haixuantao/raw_sync-rs/workflows/build/badge.svg)](https://github.com/haixuantao/raw_sync-rs/actions?query=workflow%3Abuild)
[![crates.io](https://img.shields.io/crates/v/raw_sync.svg)](https://crates.io/crates/raw_sync)
[![mio](https://docs.rs/raw_sync/badge.svg)](https://docs.rs/raw_sync/)
![Lines of Code](https://tokei.rs/b1/github/haixuantao/raw_sync-rs)

Pushed new version of raw_sync based on https://github.com/elast0ny/raw_sync-rs/pull/29 fix.

This crate provides a lightweight wrapper around OS synchronization primitives.

It was mainly developed to be used with the [shared_memory](https://github.com/elast0ny/shared_memory-rs) crate for cross-process synchronization through shared memory.


## Features
### Locks
| Feature| Description | Linux | Windows| Mac|
|--------|-------------|:-----:|:------:|:------:|
|Mutex|Mutually exclusive lock|✔|✔|✔|
|RwLock|Exclusive write/shared read|✔|X|✔|


### Events

| Feature| Description | Linux | Windows| Mac|
|--------|-------------|:-----:|:------:|:------:|
|Event| Generic event : [pthread_cond](https://linux.die.net/man/3/pthread_cond_init) on Unix and [Event Objects](https://msdn.microsoft.com/en-us/library/windows/desktop/ms682655.aspx) on windows. |✔|✔|✔|
|BusyEvent|Busy event implemented by polling a byte in a loop|✔|✔|✔|
|EventFd|[Linux specific event type](http://man7.org/linux/man-pages/man2/eventfd.2.html)|TODO|N/A|N/A|


## License

 * [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0)
 * [MIT license](http://opensource.org/licenses/MIT)

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
