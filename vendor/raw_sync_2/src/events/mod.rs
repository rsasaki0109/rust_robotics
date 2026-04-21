cfg_if::cfg_if! {
    if #[cfg(target_os = "windows")] {
        mod windows;
        use windows as os;
    } else if #[cfg(target_family = "unix")] {
        mod unix;
        use unix as os;
    } else {
        unimplemented!("This crate does not support your OS yet !");
    }
}
use crate::{Result, Timeout};
pub use os::*;

pub enum EventState {
    /// Clear's the event state so the next wait() call will block
    Clear,
    /// Sets the event to the signaled state unblocking any waiters
    Signaled,
}

pub trait EventInit {
    /// Size required for the event's internal representation
    fn size_of(addr: Option<*mut u8>) -> usize;

    /// Initializes a new instance of the event in the provided buffer and returns the number of used bytes
    /// # Safety
    /// This function is unsafe because it cannot guarantee that the provided memory is valid.
    #[allow(clippy::new_ret_no_self)]
    unsafe fn new(mem: *mut u8, auto_reset: bool) -> Result<(Box<dyn EventImpl>, usize)>;

    /// Re-uses an event from an already initialized location and returns the number of used bytes
    /// # Safety
    /// This function is unsafe because it cannot guarantee that the provided memory is valid.
    #[allow(clippy::new_ret_no_self)]
    unsafe fn from_existing(mem: *mut u8) -> Result<(Box<dyn EventImpl>, usize)>;
}

pub trait EventImpl {
    /// Wait for the event to be signaled
    fn wait(&self, timeout: Timeout) -> Result<()>;
    /// Set the current state of the event
    fn set(&self, state: EventState) -> Result<()>;
}

use std::mem::size_of;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time;

struct InnerBusy {
    signal: AtomicU8,
    auto_reset: u8,
}
pub struct BusyEvent {
    inner: *mut InnerBusy,
}
impl EventInit for BusyEvent {
    fn size_of(_addr: Option<*mut u8>) -> usize {
        size_of::<InnerBusy>()
    }
    #[allow(clippy::new_ret_no_self)]
    unsafe fn new(mem: *mut u8, auto_reset: bool) -> Result<(Box<dyn EventImpl>, usize)> {
        let ptr = mem as *mut InnerBusy;
        let obj = Self { inner: ptr };
        let inner = &mut *obj.inner;

        inner.auto_reset = if auto_reset { 1 } else { 0 };
        obj.set(EventState::Clear)?;

        Ok((Box::new(obj), Self::size_of(None)))
    }

    unsafe fn from_existing(mem: *mut u8) -> Result<(Box<dyn EventImpl>, usize)> {
        let ptr = mem as *mut InnerBusy;
        let obj = Self { inner: ptr };
        let inner = &mut *obj.inner;

        if inner.auto_reset > 1 || inner.signal.load(Ordering::Relaxed) > 1 {
            return Err(From::from("Existing BusyEvent is corrupted"));
        }

        Ok((Box::new(obj), Self::size_of(None)))
    }
}
fn busy_wait_auto(signal: &mut AtomicU8, timeout: Timeout) -> Result<()> {
    let mut prev_val = match signal.compare_exchange(1, 0, Ordering::Relaxed, Ordering::Relaxed) {
        Ok(v) => v,
        Err(v) => v,
    };

    if prev_val == 1 {
        return Ok(());
    }
    match timeout {
        Timeout::Infinite => {
            // Busy loop until signaled
            while prev_val == 0 {
                prev_val = match signal.compare_exchange(1, 0, Ordering::Relaxed, Ordering::Relaxed)
                {
                    Ok(v) => v,
                    Err(v) => v,
                };
            }
        }
        Timeout::Val(d) => {
            let start = time::Instant::now();
            while prev_val == 0 && start.elapsed() < d {
                prev_val = match signal.compare_exchange(1, 0, Ordering::Relaxed, Ordering::Relaxed)
                {
                    Ok(v) => v,
                    Err(v) => v,
                };
            }
        }
    };

    if prev_val == 1 {
        Ok(())
    } else {
        Err(From::from("Waiting for BusyEvent timed out !".to_string()))
    }
}
fn busy_wait_manual(signal: &mut AtomicU8, timeout: Timeout) -> Result<()> {
    let mut prev_val = signal.load(Ordering::Relaxed);
    if prev_val == 1 {
        return Ok(());
    }

    match timeout {
        Timeout::Infinite => {
            // Busy loop until signaled
            while prev_val == 0 {
                prev_val = signal.load(Ordering::Relaxed);
            }
        }
        Timeout::Val(d) => {
            let start = time::Instant::now();
            while prev_val == 0 && start.elapsed() < d {
                prev_val = signal.load(Ordering::Relaxed);
            }
        }
    };

    if prev_val == 1 {
        Ok(())
    } else {
        Err(From::from("Waiting for BusyEvent timed out !".to_string()))
    }
}
impl EventImpl for BusyEvent {
    fn wait(&self, timeout: Timeout) -> Result<()> {
        let inner = unsafe { &mut *self.inner };
        // Do a quick check first up
        if inner.auto_reset == 1 {
            busy_wait_auto(&mut inner.signal, timeout)
        } else {
            busy_wait_manual(&mut inner.signal, timeout)
        }
    }

    fn set(&self, state: EventState) -> Result<()> {
        let inner = unsafe { &mut *self.inner };
        match state {
            EventState::Clear => {
                //trace!("ResetEvent({:p})", self.inner);
                inner.signal.store(0, Ordering::Relaxed);
            }
            EventState::Signaled => {
                //trace!("SetEvent({:p})", self.inner);
                inner.signal.store(1, Ordering::Relaxed);
            }
        };

        Ok(())
    }
}
